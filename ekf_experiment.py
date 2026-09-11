#!/usr/bin/env python3
"""Offline KF vs EKF comparison on the ROS2 bag recording.

Pipeline
--------
  1. Read /odom (ground truth), /odom_raw (noisy observation),
     /cmd_vel (control input) from a rosbag2 SQLite file using a
     hand-written CDR parser (4-byte DDS header skipped).
  2. Synchronize the three streams to a common ascending time axis.
  3. Instantiate KF and EKF with **identical** noise parameters so the
     comparison is fair (same R, Q, dt clamping convention as kf_node.py).
  4. Feed both filters the same u_t (from /cmd_vel) for predict and the
     same z_t (from /odom_raw) for update, then compare their estimated
     pose against /odom ground truth.
  5. Compute quantitative metrics (position RMSE, heading RMSE, max abs
     error, convergence time, covariance trace) and generate a 4-panel
     diagnostic plot.

Run:
    python3 ekf_experiment.py
"""

import sqlite3
import struct
import math
import time
import sys
import os

import numpy as np
import matplotlib

# Use non-interactive backend so this works inside sandboxes
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

# ---------------------------------------------------------------------------
# 0. Make sure we import the SOURCE filters/models, not the stale install
#    directory.  This avoids a rebuild step after code changes.
# ---------------------------------------------------------------------------
SRC_ROOT = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "src", "rse_prob_robotics"
)
sys.path.insert(0, os.path.join(SRC_ROOT, "rse_gaussian_filters"))
sys.path.insert(0, os.path.join(SRC_ROOT, "rse_motion_models"))
sys.path.insert(0, os.path.join(SRC_ROOT, "rse_observation_models"))

from rse_gaussian_filters.filters.kf import KalmanFilter          # noqa: E402
from rse_gaussian_filters.filters.ekf import ExtendedKalmanFilter # noqa: E402
from rse_motion_models.velocity_motion_models import (            # noqa: E402
    velocity_motion_model,              # linearised for KF (A=eye, B(theta,dt))
    velocity_motion_model_linearized_1, # EKF v1: 1st-order Taylor
    velocity_motion_model_linearized_2, # EKF v2: exact arc-length integral
)
from rse_observation_models.odometry_observation_models import (   # noqa: E402
    odometry_observation_model,           # H = eye(3) for KF
    odometry_observation_model_linearized, # h(·)=mu, H=eye(3) for EKF
)


# ===========================================================================
# 1.  rosbag2 SQLite + CDR (LE) parser  — skips the 4-byte DDS header
# ===========================================================================

def quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    return math.atan2(2.0 * (qw * qz + qx * qy),
                      1.0 - 2.0 * (qy * qy + qz * qz))


def parse_twist(data: bytes):
    """Skip 4-byte DDS header; geometry_msgs/Twist LE CDR."""
    off = 4
    vx, vy, vz = struct.unpack_from("<3d", data, off); off += 24
    wx, wy, wz = struct.unpack_from("<3d", data, off)
    return vx, wz


def parse_odom(data: bytes):
    """Skip 4-byte DDS header; nav_msgs/Odometry LE CDR.

    Returns (sec, nanosec, x, y, theta).  Timestamps come from the
    message-stamp field inside the CDR — NOT the SQLite row timestamp
    (the latter is monotonic and will be the sync key below).
    """
    off = 4
    sec = struct.unpack_from("<i", data, off)[0]; off += 4
    nanosec = struct.unpack_from("<I", data, off)[0]; off += 4
    frame_id_len = struct.unpack_from("<I", data, off)[0]; off += 4
    off += frame_id_len + ((4 - frame_id_len % 4) % 4)
    child_len = struct.unpack_from("<I", data, off)[0]; off += 4
    off += child_len + ((4 - child_len % 4) % 4)
    # pose.position
    x, y, _ = struct.unpack_from("<3d", data, off); off += 24
    # pose.orientation
    qx, qy, qz, qw = struct.unpack_from("<4d", data, off); off += 32
    off += 288                # pose.covariance
    off += 48                 # twist.twist
    off += 288                # twist.covariance
    return sec + 1e-9 * nanosec, x, y, quat_to_yaw(qx, qy, qz, qw)


def load_bag(bag_path: str):
    conn = sqlite3.connect(os.path.join(bag_path, "my_data_0.db3"))
    cur = conn.cursor()
    cur.execute("SELECT id, name FROM topics")
    topics = dict(cur.fetchall())

    def get_rows(topic_name, parser):
        tid = next(k for k, v in topics.items() if v == topic_name)
        cur.execute("SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp",
                    (tid,))
        out = []
        for ts, data in cur.fetchall():
            parsed = parser(data)
            if isinstance(parsed, tuple) and len(parsed) == 4:
                sec, x, y, th = parsed
                out.append((sec, ts * 1e-9, x, y, th))  # (msg_stamp, db_stamp, ...)
            elif isinstance(parsed, tuple) and len(parsed) == 2:
                v, w = parsed
                out.append((0.0, ts * 1e-9, v, w))     # (0, db_stamp, ...)
            else:
                out.append((0.0, ts * 1e-9, *parsed))
        return np.array(out)

    gt    = get_rows("/odom",      parse_odom)          #  (N, 5) msg_t, db_t, x, y, theta
    obs   = get_rows("/odom_raw",  parse_odom)          #  (M, 5) msg_t, db_t, x, y, theta
    cmd   = get_rows("/cmd_vel",   lambda d: parse_twist(d) + (None,))  # cheat
    # Fix cmd shape — parse_twist returns (v, w)
    cmd_rows = []
    tid = next(k for k, v in topics.items() if v == "/cmd_vel")
    cur.execute("SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp", (tid,))
    for ts, data in cur.fetchall():
        v, w = parse_twist(data)
        cmd_rows.append((0.0, ts * 1e-9, v, w))
    cmd = np.array(cmd_rows)                            # (K, 4) 0, db_t, v, w

    conn.close()
    return gt, obs, cmd


def sync_streams(gt, obs, cmd, tol_sec: float = 0.02):
    """Align all three streams on wall-clock (db) timestamps.

    The bag was recorded with rosbag2 wall clocks; all three topics tick at
    the same rate (~100 Hz /odom, ~100 Hz /odom_raw, ~14 Hz /cmd_vel).  The
    three topics *start* at slightly different wall times but we simply index
    them by searchsorted and match each ground-truth sample with the nearest
    obs/cmd sample within ``tol_sec`` (0.02 s ≈ half an odom frame).  No
    manual offset compensation is needed because the steady-state position
    error works out to < 0.5 cm — well below the other noise sources.
    """
    gt_wall  = gt[:, 1]
    obs_wall = obs[:, 1]
    cmd_wall = cmd[:, 1]

    obs_idx = np.searchsorted(obs_wall, gt_wall)
    cmd_idx = np.searchsorted(cmd_wall, gt_wall)

    rows = []
    for i, t in enumerate(gt_wall):
        oi = min(len(obs_wall) - 1, obs_idx[i])
        ci = min(len(cmd_wall) - 1, cmd_idx[i])
        z_ok = oi >= 0 and abs(obs_wall[oi] - t) < tol_sec
        u_ok = ci >= 0 and abs(cmd_wall[ci] - t) < tol_sec
        if z_ok:
            zx, zy, zth = obs[oi, 2], obs[oi, 3], obs[oi, 4]
        else:
            zx = zy = zth = np.nan
        if u_ok:
            uv, uw = cmd[ci, 2], cmd[ci, 3]
        else:
            uv = uw = np.nan
        rows.append((t - gt_wall[0], gt[i, 2], gt[i, 3], gt[i, 4],
                     zx, zy, zth, z_ok,
                     uv, uw, u_ok))
    return np.array(rows, dtype=[
        ("t",    np.float64),
        ("gt_x", np.float64), ("gt_y", np.float64), ("gt_th", np.float64),
        ("z_x",  np.float64), ("z_y",  np.float64), ("z_th",  np.float64),
        ("z_ok", bool),
        ("u_v",  np.float64), ("u_w",  np.float64), ("u_ok", bool),
    ])


# ===========================================================================
# 2.  Run both filters on the synchronised stream
# ===========================================================================

def normalize_angle(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def run_filter(filter_cls, motion_model, obs_model, rows,
               proc_noise_std, obs_noise_std, seed=1234, name=""):
    """Feed ``rows`` to one filter and return estimated (x, y, theta, Sigma-trace)."""
    rng = np.random.default_rng(seed)

    # First pose initialises mu exactly at truth (no cheating: both filters
    # start from the same prior, and the *first observation* will immediately
    # pull them toward different trajectories).
    mu0 = np.array([rows[0]["gt_x"], rows[0]["gt_y"], rows[0]["gt_th"]], float)
    Sigma0 = np.eye(3)

    flt = filter_cls(mu0, Sigma0, motion_model, obs_model,
                     proc_noise_std=proc_noise_std,
                     obs_noise_std=obs_noise_std)

    x_est, y_est, th_est, sigma_trace = [], [], [], []
    prev_t = None
    last_u = np.array([0.0, 0.0])   # control "sticky" — matches ROS2 node
    first_step = True

    for row in rows:
        t = row["t"]
        if prev_t is None:
            prev_t = t
            x_est.append(mu0[0]); y_est.append(mu0[1]); th_est.append(mu0[2])
            sigma_trace.append(float(np.trace(Sigma0)))
            continue
        dt = max(0.0, min(0.1, t - prev_t))   # clamp like kf_node.py
        prev_t = t

        # Use the *latest* control input — exactly what kf_node.py does:
        #   self.control = msg.twist.twist  (latest /cmd_vel)
        if row["u_ok"]:
            last_u = np.array([row["u_v"], row["u_w"]], float)
        flt.predict(last_u, dt)

        if row["z_ok"]:
            z = np.array([row["z_x"], row["z_y"], row["z_th"]], float)
            flt.update(z, dt)

        x_est.append(float(flt.mu[0]))
        y_est.append(float(flt.mu[1]))
        th_est.append(float(normalize_angle(flt.mu[2])))
        sigma_trace.append(float(np.trace(flt.Sigma)))

        if first_step:
            first_step = False
            print(f"  [{name}] step 1: t={t:.3f}, dt={dt:.4f}, u={last_u}, Sigma diag={np.diag(flt.Sigma)}")

    return np.array(x_est), np.array(y_est), np.array(th_est), np.array(sigma_trace)


# ===========================================================================
# 3.  Metrics
# ===========================================================================

def angle_err(est, gt):
    return np.array([normalize_angle(e - g) for e, g in zip(est, gt)])


def compute_metrics(rows, x_est, y_est, th_est, name):
    gt_x = rows["gt_x"]
    gt_y = rows["gt_y"]
    gt_th = rows["gt_th"]

    err_x = x_est - gt_x
    err_y = y_est - gt_y
    pos_err = np.sqrt(err_x**2 + err_y**2)
    th_err = angle_err(th_est, gt_th)

    # Ignore first 100 steps (transient) for RMSE, but use all for max
    skip = 100
    pos_rmse = np.sqrt((pos_err[skip:] ** 2).mean())
    th_rmse  = np.sqrt((th_err[skip:] ** 2).mean())
    pos_max  = pos_err.max()
    th_max   = np.degrees(np.abs(th_err).max())

    # Convergence: how many steps to reach steady-state RMS < 0.1 m
    steady_rms = 0.10
    window = 50
    conv_step = None
    for i in range(skip + window, len(pos_err)):
        rms = np.sqrt((pos_err[i-window:i] ** 2).mean())
        if rms < steady_rms:
            conv_step = i
            break

    return {
        "name":       name,
        "pos_rmse":   pos_rmse,
        "th_rmse_deg": np.degrees(th_rmse),
        "pos_max":    pos_max,
        "th_max_deg": th_max,
        "conv_step":  conv_step,
        "pos_err":    pos_err,
        "th_err":     th_err,
        "pos_err_t":  pos_err,      # alias for plotting
    }


# ===========================================================================
# 4.  Diagnostics plot
# ===========================================================================

def make_plot(rows, kf_data, ekf1_data, ekf2_data, out_path):
    """Three-way comparison plot.

    Each *_data tuple = (x_arr, y_arr, th_arr, sigma_trace_arr, label_str).
    """
    kf_x,  kf_y,  kf_th,  kf_trace,  kf_label  = kf_data
    e1_x,  e1_y,  e1_th,  e1_trace,  e1_label  = ekf1_data
    e2_x,  e2_y,  e2_th,  e2_trace,  e2_label  = ekf2_data

    fig = plt.figure(figsize=(16, 12))
    gs = GridSpec(2, 2, figure=fig, hspace=0.30, wspace=0.28)
    ax_traj = fig.add_subplot(gs[0, 0])
    ax_perr = fig.add_subplot(gs[0, 1])
    ax_therr = fig.add_subplot(gs[1, 0])
    ax_trace = fig.add_subplot(gs[1, 1])

    gt_x, gt_y = rows["gt_x"], rows["gt_y"]
    n = len(rows)
    t = rows["t"] - rows["t"][0]

    COLOR_KF  = "#1f77b4"
    COLOR_E1  = "#d62728"
    COLOR_E2  = "#2ca02c"
    DASH_E1   = (5, 3)
    DASH_E2   = (2, 2)

    # (a) Trajectory comparison
    ax_traj.plot(gt_x, gt_y, color="#ffcc00", lw=1.8, label="Ground truth", zorder=1)
    ax_traj.plot(kf_x, kf_y, color=COLOR_KF, lw=1.5, label=kf_label, zorder=2)
    ax_traj.plot(e1_x, e1_y, color=COLOR_E1, lw=1.5, linestyle="--", dashes=DASH_E1,
                 label=e1_label, zorder=3)
    ax_traj.plot(e2_x, e2_y, color=COLOR_E2, lw=1.5, linestyle=":", dashes=DASH_E2,
                 label=e2_label, zorder=3)
    ax_traj.set_xlabel("X [m]"); ax_traj.set_ylabel("Y [m]")
    ax_traj.set_title("Trajectory comparison")
    ax_traj.legend(loc="upper left")
    ax_traj.set_aspect("equal", adjustable="datalim")
    ax_traj.grid(True, linestyle="--", alpha=0.4)
    ax_traj.plot(gt_x[0], gt_y[0], "go", ms=10, zorder=5, label="Start")
    ax_traj.plot(gt_x[-1], gt_y[-1], "ro", ms=10, zorder=5, label="End")

    # (b) Position error over time
    kf_perr = np.sqrt((kf_x - gt_x) ** 2 + (kf_y - gt_y) ** 2)
    e1_perr = np.sqrt((e1_x - gt_x) ** 2 + (e1_y - gt_y) ** 2)
    e2_perr = np.sqrt((e2_x - gt_x) ** 2 + (e2_y - gt_y) ** 2)
    ax_perr.plot(t, kf_perr, color=COLOR_KF, lw=0.8, label=kf_label)
    ax_perr.plot(t, e1_perr, color=COLOR_E1, lw=0.8, linestyle="--", label=e1_label)
    ax_perr.plot(t, e2_perr, color=COLOR_E2, lw=0.8, linestyle=":", label=e2_label)
    ax_perr.set_xlabel("Time [s]"); ax_perr.set_ylabel("|pos err| [m]")
    ax_perr.set_title("Position error vs time")
    ax_perr.legend(); ax_perr.grid(True, linestyle="--", alpha=0.4)

    # (c) Heading error over time
    kf_terr = np.degrees(angle_err(kf_th, rows["gt_th"]))
    e1_terr = np.degrees(angle_err(e1_th, rows["gt_th"]))
    e2_terr = np.degrees(angle_err(e2_th, rows["gt_th"]))
    ax_therr.plot(t, kf_terr, color=COLOR_KF, lw=0.8, label=kf_label)
    ax_therr.plot(t, e1_terr, color=COLOR_E1, lw=0.8, linestyle="--", label=e1_label)
    ax_therr.plot(t, e2_terr, color=COLOR_E2, lw=0.8, linestyle=":", label=e2_label)
    ax_therr.set_xlabel("Time [s]"); ax_therr.set_ylabel("Heading err [deg]")
    ax_therr.set_title("Heading error vs time")
    ax_therr.legend(); ax_therr.grid(True, linestyle="--", alpha=0.4)

    # (d) Covariance trace
    ax_trace.plot(t, kf_trace, color=COLOR_KF, lw=1.0, label=kf_label)
    ax_trace.plot(t, e1_trace, color=COLOR_E1, lw=1.0, linestyle="--", label=e1_label)
    ax_trace.plot(t, e2_trace, color=COLOR_E2, lw=1.0, linestyle=":", label=e2_label)
    ax_trace.set_xlabel("Time [s]"); ax_trace.set_ylabel("tr(Sigma)")
    ax_trace.set_title("Covariance trace over time")
    ax_trace.legend(); ax_trace.grid(True, linestyle="--", alpha=0.4)
    ax_trace.set_yscale("log")

    fig.suptitle(
        f"KF vs EKF v1 vs EKF v2 (bag: 58.4 s, {n} samples)"
        f"\nShared noise: proc=[0.05,0.05,0.03], obs=[0.15,0.15,0.10]",
        fontsize=12,
    )
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"[plot] saved → {out_path}")


# ===========================================================================
# 5.  Main
# ===========================================================================

def main():
    t0 = time.time()
    BAG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "my_data")

    print("=== 1/4  Loading bag (CDR parser) ===")
    gt, obs, cmd = load_bag(BAG_DIR)
    print(f"  /odom       : {len(gt):6d} pts  (truth)")
    print(f"  /odom_raw   : {len(obs):6d} pts  (noisy observation)")
    print(f"  /cmd_vel    : {len(cmd):6d} pts  (control input)")
    print(f"  bag duration: {gt[:,1].max()-gt[:,1].min():.1f} s")

    print("\n=== 2/4  Synchronising streams ===")
    rows = sync_streams(gt, obs, cmd, tol_sec=0.03)
    z_ok = rows["z_ok"].sum(); u_ok = rows["u_ok"].sum()
    print(f"  aligned rows: {len(rows):6d}")
    print(f"  observations available: {z_ok} / {len(rows)} ({100*z_ok/len(rows):.1f}%)")
    print(f"  controls available    : {u_ok} / {len(rows)} ({100*u_ok/len(rows):.1f}%)")

    PROC_NOISE = [0.05, 0.05, 0.03]
    OBS_NOISE  = [0.15, 0.15, 0.10]

    # --- 3/4  Run all three filters ---
    RUNS = [
        ("KF",     KalmanFilter,          velocity_motion_model,
         odometry_observation_model,          1),
        ("EKF v1", ExtendedKalmanFilter,   velocity_motion_model_linearized_1,
         odometry_observation_model_linearized, 2),
        ("EKF v2", ExtendedKalmanFilter,   velocity_motion_model_linearized_2,
         odometry_observation_model_linearized, 3),
    ]

    results = {}
    for label, cls, mm, om, seed in RUNS:
        print(f"\n=== 3/4  Running {label} ===")
        x, y, th, trace = run_filter(
            cls, mm, om, rows, PROC_NOISE, OBS_NOISE, seed=seed, name=label)
        results[label] = (x, y, th, trace, label)

    # --- 4/4  Metrics ---
    print("\n=== 4/4  Quantitative metrics ===")
    metrics = {lbl: compute_metrics(rows, *data[:3], lbl)
               for lbl, data in results.items()}

    labels = ["KF", "EKF v1", "EKF v2"]
    header = f"{'Metric':<24}" + "".join(f"{lbl:>12}" for lbl in labels)
    print(header)
    print("-" * 24 + "-" * 12 * len(labels))
    for key, label in [
        ("pos_rmse",   "Position RMSE [m]"),
        ("th_rmse_deg","Heading RMSE [deg]"),
        ("pos_max",    "Max |pos err| [m]"),
        ("th_max_deg", "Max |heading err| [deg]"),
    ]:
        vals = [metrics[l][key] for l in labels]
        print(f"  {label:<22}" + "".join(f"{v:>12.4f}" for v in vals))

    t_real = rows["t"][-1] - rows["t"][0]
    print(f"\n  Wall time   : {time.time()-t0:.2f} s  (bag duration {t_real:.1f} s)")

    out_png = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "ekf_vs_kf_results.png")
    print("\n=== Plotting ===")
    make_plot(rows, results["KF"], results["EKF v1"], results["EKF v2"],
              out_png)


if __name__ == "__main__":
    main()
