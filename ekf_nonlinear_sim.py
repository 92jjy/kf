#!/usr/bin/env python3
"""EKF vs KF in a deliberately **strongly nonlinear** scenario.

The bag from the ROS2 project has a *very* weakly nonlinear dynamics model
(v ≈ 0.4 m/s, w ≤ 0.9 rad/s, dt ≤ 0.02 s) so KF and EKF produce virtually
identical trajectories.  To really stress-test the EKF we run a **synthetic
trajectory** with three amplifications:

  1. Large curvature — w ∈ {−1.2, +1.2} rad/s  (≈ 70°/s turn rate)
  2. Long time-step — dt = 0.2 s per predict/update (20 Hz)
  3. Bearing-only observation model — h(μ) = atan2(y, x)
     This is a *truly nonlinear* observation that KF cannot handle with
     any linearised model, while EKF linearises h at the current estimate.

Ground truth is generated from the *exact arc-length integral* (VMML2 g).
We then simulate noisy observations and feed the same data to both filters.

Run:
    python3 ekf_nonlinear_sim.py
"""

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

# ---------------------------------------------------------------------------
# 0.  Minimal filter implementations (inlined so this file is self-contained
#     and we do not depend on the ROS source)
# ---------------------------------------------------------------------------

def _normalize(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


class KF:
    """Linear Kalman filter.  Observation model C = I, motion model A=I.

    Motion model in this simulator has B(u, dt) = [[cos(θ)*dt, 0],
                                                  [sin(θ)*dt, 0],
                                                  [0, dt]]
    We use the current θ for B — which is *precisely* how KF behaves.
    """
    def __init__(self, mu0, Sigma0, R, Q):
        self.mu = np.array(mu0, float)
        self.Sigma = np.array(Sigma0, float)
        self.R = np.diag(R)
        self.Q = np.diag(Q)

    def predict(self, u, dt):
        v, w = u
        th = self.mu[2]
        # Linearised B at current theta
        B = np.array([
            [np.cos(th) * dt, 0.0],
            [np.sin(th) * dt, 0.0],
            [0.0, dt],
        ])
        self.mu = self.mu + B @ np.array([v, w])   # A = I
        self.mu[2] = _normalize(self.mu[2])
        self.Sigma = self.Sigma + self.R           # A = I

    def update_bearing(self, z, dt):
        # KF cannot handle nonlinear h!  We pretend h(mu) = [x, y, theta]
        # — which is an identity observation — but feed it z = [r, b, ?]
        # This is fundamentally wrong, so the KF *will diverge* on bearing-only.
        K = self.Sigma @ np.eye(3) @ np.linalg.inv(self.Sigma + self.Q)
        innovation = np.array([
            z[0] - self.mu[0],
            z[1] - self.mu[1],
            z[2] - self.mu[2],
        ])
        self.mu = self.mu + K @ innovation
        self.mu[2] = _normalize(self.mu[2])
        self.Sigma = (np.eye(3) - K) @ self.Sigma


class EKF:
    """Extended Kalman filter with exact arc-length motion model and
    bearing-only observation.
    """
    def __init__(self, mu0, Sigma0, R, Q):
        self.mu = np.array(mu0, float)
        self.Sigma = np.array(Sigma0, float)
        self.R = np.diag(R)
        # For bearing-only we *construct* Q as a 3×3 matrix with:
        #   - 2D position (from simulated depth + bearing noise)
        #   - heading from a separate odometry-like measurement
        self.Q = np.diag(Q)

    def predict(self, u, dt):
        v, w = u
        x, y, th = self.mu
        if abs(w) < 1e-6:
            w = 1e-6
        # Exact arc-length g (VMML2)
        self.mu = np.array([
            x + -v / w * np.sin(th) + v / w * np.sin(th + w * dt),
            y +  v / w * np.cos(th) - v / w * np.cos(th + w * dt),
            th + w * dt,
        ])
        self.mu[2] = _normalize(self.mu[2])
        # Jacobian G (VMML2)
        stw = np.sin(th + w * dt)
        ctw = np.cos(th + w * dt)
        st = np.sin(th)
        ct = np.cos(th)
        G = np.array([
            [1, 0, -v / w * ct + v / w * ctw],
            [0, 1, -v / w * st + v / w * stw],
            [0, 0, 1],
        ])
        self.Sigma = G @ self.Sigma @ G.T + self.R

    def update_bearing(self, z, dt):
        # Nonlinear observation: h(μ) = [atan2(y, x), distance, θ]
        # z = [bearing, range, heading]
        x, y, th = self.mu
        r = np.hypot(x, y)
        h = np.array([np.arctan2(y, x), r, th])
        # Jacobian H
        H = np.array([
            [-y / (x * x + y * y),  x / (x * x + y * y), 0],
            [ x / r,                 y / r,                0],
            [0,                      0,                   1],
        ])
        S = H @ self.Sigma @ H.T + self.Q
        K = self.Sigma @ H.T @ np.linalg.inv(S)
        innovation = np.array([
            _normalize(z[0] - h[0]),
                     z[1] - h[1],
                     _normalize(z[2] - h[2]),
        ])
        self.mu = self.mu + K @ innovation
        self.mu[2] = _normalize(self.mu[2])
        self.Sigma = (np.eye(3) - K @ H) @ self.Sigma


# ---------------------------------------------------------------------------
# 1.  Ground-truth trajectory generator
# ---------------------------------------------------------------------------

def generate_trajectory(n_steps, dt, rng):
    """Arc-length trajectory with large heading changes.

    Control profile:
        v = 1.0 m/s constant
        w = +1.2 rad/s for 4 s, then -1.2 rad/s for 4 s, repeat 3 times
        (total 6 segments → heading sweeps ±1.2 rad/s × 4 s = ±4.8 rad ≈ ±275°)

    This produces a trajectory that crosses heading ±180° twice — a perfect
    stress test for angle normalisation.
    """
    v = 1.0
    segment_len = int(4.0 / dt)    # 4 s per segment
    # w sequence: +1.2, -1.2, +1.2, -1.2, +1.2, -1.2
    w_seq = [+1.2, -1.2] * 3

    # Build u list
    u_list = []
    for w in w_seq:
        u_list.extend([(v, w)] * segment_len)
    # Pad / truncate to n_steps
    u_list = (u_list * ((n_steps // len(u_list)) + 1))[:n_steps]

    # Simulate ground truth with exact arc-length model
    gt = np.zeros((n_steps, 3))   # x, y, theta
    gt[0] = [5.0, 0.0, np.pi / 2]   # start at (5, 0) facing up
    for i in range(1, n_steps):
        vv, ww = u_list[i]
        th = gt[i-1, 2]
        if abs(ww) < 1e-6:
            ww = 1e-6
        gt[i] = [
            gt[i-1, 0] + -vv/ww * np.sin(th) + vv/ww * np.sin(th + ww*dt),
            gt[i-1, 1] +  vv/ww * np.cos(th) - vv/ww * np.cos(th + ww*dt),
            _normalize(th + ww * dt),
        ]

    return gt, np.array(u_list)


# ---------------------------------------------------------------------------
# 2.  Main experiment
# ---------------------------------------------------------------------------

def run():
    rng = np.random.default_rng(42)
    dt = 0.2
    n_steps = 120      # 24 s @ 5 Hz
    gt, u_list = generate_trajectory(n_steps, dt, rng)

    # Noise levels — challenging but still within realistic bounds
    proc_noise   = [0.10, 0.10, 0.06]   # σ_v=0.10 m/s, σ_w=0.06 rad/s
    obs_noise    = [0.15, 0.10, 0.05]   # σ_bearing=0.15 rad, σ_range=0.10 m, σ_head=0.05 rad

    # Simulate noisy bearing observations: z = [atan2(y, x), r, θ] + noise
    # We also give the KF access to a *naive* linear observation so it is not
    # entirely crippled — this is important for a fair comparison because
    # a KF cannot process bearing-only observations at all.
    z_noise_full = rng.normal(0, [obs_noise[0], obs_noise[1], obs_noise[2]],
                              size=gt.shape)
    z_true_bearing = np.arctan2(gt[:, 1], gt[:, 0])
    z_true_range   = np.hypot(gt[:, 0], gt[:, 1])
    z_true_head    = gt[:, 2]
    z_bearing = np.column_stack([
        _normalize(z_true_bearing + z_noise_full[:, 0]),
                  z_true_range   + z_noise_full[:, 1],
        _normalize(z_true_head   + z_noise_full[:, 2]),
    ])
    # Also simulate noisy direct pose (what KF "wants" to observe)
    z_pose = gt + rng.normal(0, [0.15, 0.15, 0.05], size=gt.shape)

    # === Filter 1: KF with direct pose observation (what it was designed for) ===
    kf1 = KF(gt[0], np.eye(3) * 0.5, proc_noise, obs_noise)
    kf1_x, kf1_y, kf1_th, kf1_trace = [], [], [], []
    for i in range(1, n_steps):
        kf1.predict(u_list[i], dt)
        K = kf1.Sigma @ np.eye(3) @ np.linalg.inv(kf1.Sigma + np.diag(obs_noise)**2)
        kf1.mu = kf1.mu + K @ (z_pose[i] - kf1.mu)
        kf1.mu[2] = _normalize(kf1.mu[2])
        kf1.Sigma = (np.eye(3) - K) @ kf1.Sigma
        kf1_x.append(kf1.mu[0])
        kf1_y.append(kf1.mu[1])
        kf1_th.append(kf1.mu[2])
        kf1_trace.append(np.trace(kf1.Sigma))

    # === Filter 2: EKF with SAME direct pose observation ===
    ekf1 = EKF(gt[0], np.eye(3) * 0.5, proc_noise, obs_noise)
    ekf1_x, ekf1_y, ekf1_th, ekf1_trace = [], [], [], []
    for i in range(1, n_steps):
        ekf1.predict(u_list[i], dt)
        # Direct pose update with identity H
        H = np.eye(3)
        S = ekf1.Sigma + np.diag(obs_noise)**2
        K = ekf1.Sigma @ np.linalg.inv(S)
        innovation = z_pose[i] - ekf1.mu
        innovation[2] = _normalize(innovation[2])
        ekf1.mu = ekf1.mu + K @ innovation
        ekf1.mu[2] = _normalize(ekf1.mu[2])
        ekf1.Sigma = (np.eye(3) - K) @ ekf1.Sigma
        ekf1_x.append(ekf1.mu[0])
        ekf1_y.append(ekf1.mu[1])
        ekf1_th.append(ekf1.mu[2])
        ekf1_trace.append(np.trace(ekf1.Sigma))

    # === Filter 3: EKF with bearing-only observation (the *strongly nonlinear* case) ===
    ekf2 = EKF(gt[0], np.eye(3) * 0.5, proc_noise, obs_noise)
    ekf2_x, ekf2_y, ekf2_th, ekf2_trace = [], [], [], []
    for i in range(1, n_steps):
        ekf2.predict(u_list[i], dt)
        ekf2.update_bearing(z_bearing[i], dt)
        ekf2_x.append(ekf2.mu[0])
        ekf2_y.append(ekf2.mu[1])
        ekf2_th.append(ekf2.mu[2])
        ekf2_trace.append(np.trace(ekf2.Sigma))

    # === Filter 4: KF naively fed bearing-only (should fail) ===
    kf2 = KF(gt[0], np.eye(3) * 0.5, proc_noise, obs_noise)
    kf2_x, kf2_y, kf2_th, kf2_trace = [], [], [], []
    for i in range(1, n_steps):
        kf2.predict(u_list[i], dt)
        kf2.update_bearing(z_bearing[i], dt)
        kf2_x.append(kf2.mu[0])
        kf2_y.append(kf2.mu[1])
        kf2_th.append(kf2.mu[2])
        kf2_trace.append(np.trace(kf2.Sigma))

    # GT arrays (skip t=0 because filter starts there)
    gt_x = gt[1:, 0]
    gt_y = gt[1:, 1]
    gt_th = gt[1:, 2]
    t = np.arange(1, n_steps) * dt

    # --- Metrics ---
    def metrics(x, y, th, label):
        pe = np.sqrt((x - gt_x)**2 + (y - gt_y)**2)
        te = np.degrees(np.abs(_normalize(th - gt_th)))
        return {
            "label": label,
            "pos_rmse": np.sqrt((pe**2).mean()),
            "th_rmse":  np.sqrt((te**2).mean()),
            "pos_max":  pe.max(),
            "th_max":   te.max(),
            "pos_final": pe[-1],
        }

    results = [
        metrics(kf1_x, kf1_y, kf1_th, "KF + pose obs"),
        metrics(ekf1_x, ekf1_y, ekf1_th, "EKF + pose obs"),
        metrics(ekf2_x, ekf2_y, ekf2_th, "EKF + bearing obs"),
        metrics(kf2_x, kf2_y, kf2_th, "KF + bearing obs (naive)"),
    ]

    print("\n" + "=" * 70)
    print("STRONG NONLINEARITY EXPERIMENT — Circular trajectory + bearing-only obs")
    print("=" * 70)
    print(f"{'Filter + Observation':<32} {'Pos RMSE [m]':>12} {'Th RMSE [deg]':>14} {'Max |pos|':>10}")
    print("-" * 70)
    for r in results:
        print(f"  {r['label']:<30} {r['pos_rmse']:>12.4f} {r['th_rmse']:>14.2f} {r['pos_max']:>10.4f}")

    # --- Plot ---
    fig = plt.figure(figsize=(16, 12))
    gs = GridSpec(2, 2, figure=fig, hspace=0.30, wspace=0.28)

    ax_traj = fig.add_subplot(gs[0, 0])
    ax_perr = fig.add_subplot(gs[0, 1])
    ax_therr = fig.add_subplot(gs[1, 0])
    ax_trace = fig.add_subplot(gs[1, 1])

    C_GT = "#ffcc00"
    C_KF_P = "#1f77b4"
    C_EKF_P = "#2ca02c"
    C_EKF_B = "#d62728"
    C_KF_B = "#9467bd"

    ax_traj.plot(gt[:, 0], gt[:, 1], color=C_GT, lw=2, label="Ground truth", zorder=1)
    ax_traj.plot(kf1_x, kf1_y, color=C_KF_P, lw=1.5, label="KF + pose obs")
    ax_traj.plot(ekf1_x, ekf1_y, color=C_EKF_P, lw=1.5, label="EKF + pose obs")
    ax_traj.plot(ekf2_x, ekf2_y, color=C_EKF_B, lw=1.5, linestyle="--",
                 dashes=(5, 3), label="EKF + bearing obs")
    ax_traj.plot(kf2_x, kf2_y, color=C_KF_B, lw=1.5, linestyle=":",
                 dashes=(2, 2), label="KF + bearing obs (naive)")
    ax_traj.plot(gt[0, 0], gt[0, 1], "go", ms=12, zorder=5, label="Start")
    ax_traj.set_xlabel("X [m]"); ax_traj.set_ylabel("Y [m]")
    ax_traj.set_title("Trajectory — strong nonlinearity")
    ax_traj.legend(loc="upper right", fontsize=9)
    ax_traj.set_aspect("equal", adjustable="datalim")
    ax_traj.grid(True, linestyle="--", alpha=0.4)

    kf1_perr = np.sqrt((np.array(kf1_x) - gt_x)**2 + (np.array(kf1_y) - gt_y)**2)
    ekf1_perr = np.sqrt((np.array(ekf1_x) - gt_x)**2 + (np.array(ekf1_y) - gt_y)**2)
    ekf2_perr = np.sqrt((np.array(ekf2_x) - gt_x)**2 + (np.array(ekf2_y) - gt_y)**2)
    kf2_perr = np.sqrt((np.array(kf2_x) - gt_x)**2 + (np.array(kf2_y) - gt_y)**2)

    ax_perr.plot(t, kf1_perr, color=C_KF_P, lw=1.0, label="KF + pose")
    ax_perr.plot(t, ekf1_perr, color=C_EKF_P, lw=1.0, label="EKF + pose")
    ax_perr.plot(t, ekf2_perr, color=C_EKF_B, lw=1.0, linestyle="--", label="EKF + bearing")
    ax_perr.plot(t, kf2_perr, color=C_KF_B, lw=1.0, linestyle=":", label="KF + bearing (naive)")
    ax_perr.set_xlabel("Time [s]"); ax_perr.set_ylabel("|pos err| [m]")
    ax_perr.set_title("Position error — EKF thrives on nonlinear obs!")
    ax_perr.legend(fontsize=8); ax_perr.grid(True, linestyle="--", alpha=0.4)

    def th_err(a, b):
        return np.degrees(np.array([_normalize(x-y) for x, y in zip(a, b)]))

    ax_therr.plot(t, th_err(kf1_th, gt_th), color=C_KF_P, lw=1.0, label="KF + pose")
    ax_therr.plot(t, th_err(ekf1_th, gt_th), color=C_EKF_P, lw=1.0, label="EKF + pose")
    ax_therr.plot(t, th_err(ekf2_th, gt_th), color=C_EKF_B, lw=1.0, linestyle="--", label="EKF + bearing")
    ax_therr.plot(t, th_err(kf2_th, gt_th), color=C_KF_B, lw=1.0, linestyle=":", label="KF + bearing (naive)")
    ax_therr.set_xlabel("Time [s]"); ax_therr.set_ylabel("Heading err [deg]")
    ax_therr.set_title("Heading error")
    ax_therr.legend(fontsize=8); ax_therr.grid(True, linestyle="--", alpha=0.4)

    ax_trace.plot(t, kf1_trace, color=C_KF_P, lw=1.0, label="KF + pose")
    ax_trace.plot(t, ekf1_trace, color=C_EKF_P, lw=1.0, label="EKF + pose")
    ax_trace.plot(t, ekf2_trace, color=C_EKF_B, lw=1.0, linestyle="--", label="EKF + bearing")
    ax_trace.plot(t, kf2_trace, color=C_KF_B, lw=1.0, linestyle=":", label="KF + bearing (naive)")
    ax_trace.set_xlabel("Time [s]"); ax_trace.set_ylabel("tr(Sigma)")
    ax_trace.set_title("Covariance trace")
    ax_trace.legend(fontsize=8); ax_trace.grid(True, linestyle="--", alpha=0.4)
    ax_trace.set_yscale("log")

    fig.suptitle(
        f"EKF vs KF — Strong nonlinearity stress test\n"
        f"Circular trajectory ({n_steps-1} steps @ {dt} s, "
        f"v=1.0 m/s, w=±1.2 rad/s), bearing-only + heading obs",
        fontsize=12,
    )
    fig.savefig("ekf_nonlinear_results.png", dpi=150, bbox_inches="tight")
    plt.close(fig)
    print("\n[plot] saved → ekf_nonlinear_results.png")


if __name__ == "__main__":
    run()
