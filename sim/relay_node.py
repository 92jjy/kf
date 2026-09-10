#!/usr/bin/env python3
"""Relay /odom -> /odom_raw and /imu/data -> /imu/data_raw with realistic
wheel-odometry noise/drift.

The KF node subscribes to 'odom_raw' (controls + observation) and
'/imu/data_raw'; Gazebo publishes ideal 'odom' and 'imu_plugin/out'.

Noise model (simulating real wheel odometry):
  * velocities have white Gaussian noise plus constant calibration biases
    (wheel-radius error -> linear bias, track-width error -> angular bias);
  * a dead-reckoned pose is integrated from the noisy velocities starting
    at the first true pose, so drift accumulates naturally over time;
  * the published pose gets extra white position/heading measurement noise.

All noise levels are ROS parameters (defaults tuned to give visible drift
within ~1 minute without breaking tracking).
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

from rse_common_utils.sensor_utils import get_yaw_from_quaternion
from rse_common_utils.helper_utils import normalize_angle


def yaw_to_quaternion(theta):
    """Return (qx, qy, qz, qw) quaternion for a pure rotation about z."""
    half = 0.5 * theta
    return (0.0, 0.0, math.sin(half), math.cos(half))


class SensorRelay(Node):

    def __init__(self):
        super().__init__('sensor_relay')

        # ---- Noise parameters (declared so they can be tuned at launch) ----
        # Tuned for a teaching demo: clearly visible drift over ~1 min of
        # motion (~0.5-1 m position, ~5-10 deg heading) while the filter can
        # still track the observation.
        self.declare_parameter('bias_v', 0.008)       # constant linear bias [m/s]
        self.declare_parameter('bias_w', 0.005)       # constant angular bias [rad/s]
        self.declare_parameter('sigma_v', 0.01)       # velocity white noise std [m/s]
        self.declare_parameter('sigma_w', 0.008)      # angular rate white noise std [rad/s]
        self.declare_parameter('sigma_pos', 0.01)     # pose position white noise std [m]
        self.declare_parameter('sigma_theta', 0.01)   # pose heading white noise std [rad]
        self.declare_parameter('sigma_imu_w', 0.005)  # imu angular-rate noise std [rad/s]
        self.declare_parameter('enable_noise', True)  # master switch

        # Constant calibration biases, drawn once at startup
        self.bias_v = float(self.get_parameter('bias_v').value)
        self.bias_w = float(self.get_parameter('bias_w').value)
        self.sigma_v = float(self.get_parameter('sigma_v').value)
        self.sigma_w = float(self.get_parameter('sigma_w').value)
        self.sigma_pos = float(self.get_parameter('sigma_pos').value)
        self.sigma_theta = float(self.get_parameter('sigma_theta').value)
        self.sigma_imu_w = float(self.get_parameter('sigma_imu_w').value)
        self.enable_noise = bool(self.get_parameter('enable_noise').value)

        self.pub_odom_raw = self.create_publisher(Odometry, 'odom_raw', 20)
        self.pub_imu_raw = self.create_publisher(Imu, 'imu/data_raw', 50)
        self.create_subscription(Odometry, 'odom', self.odom_cb, 20)
        # Gazebo classic imu plugin publishes on <plugin_name>/out despite remaps
        self.create_subscription(Imu, 'imu_plugin/out', self.imu_cb, 50)
        self.create_subscription(Imu, 'imu/data', self.imu_cb, 50)

        # Dead-reckoning state (initialised from the first true pose)
        self._dr_initialized = False
        self._dr_x = 0.0
        self._dr_y = 0.0
        self._dr_theta = 0.0
        self._last_time = None

    # -------------------------------------------------------------- odometry
    def odom_cb(self, msg):
        if not self.enable_noise:
            self.pub_odom_raw.publish(msg)
            return

        # Time delta from message stamp (fallback to node clock)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp == 0.0:
            stamp = self.get_clock().now().nanoseconds * 1e-9
        dt = 0.0 if self._last_time is None else max(stamp - self._last_time, 0.0)
        self._last_time = stamp

        # True velocity from Gazebo
        v = msg.twist.twist.linear.x
        w = msg.twist.twist.angular.z

        # Noisy velocity: white noise + constant calibration bias
        v_noisy = v + self.bias_v + np.random.normal(0.0, self.sigma_v)
        w_noisy = w + self.bias_w + np.random.normal(0.0, self.sigma_w)

        if not self._dr_initialized:
            # Seed dead-reckoning from the first true pose
            true_theta = get_yaw_from_quaternion(msg.pose.pose.orientation)
            self._dr_x = msg.pose.pose.position.x
            self._dr_y = msg.pose.pose.position.y
            self._dr_theta = true_theta
            self._dr_initialized = True
        else:
            # Integrate noisy velocities (dead reckoning -> accumulated drift)
            self._dr_x += v_noisy * math.cos(self._dr_theta) * dt
            self._dr_y += v_noisy * math.sin(self._dr_theta) * dt
            self._dr_theta = normalize_angle(self._dr_theta + w_noisy * dt)

        # Add white measurement noise on top of the drifted pose
        out_x = self._dr_x + np.random.normal(0.0, self.sigma_pos)
        out_y = self._dr_y + np.random.normal(0.0, self.sigma_pos)
        out_theta = normalize_angle(
            self._dr_theta + np.random.normal(0.0, self.sigma_theta))

        # Build the noisy odometry message
        noisy = Odometry()
        noisy.header = msg.header
        noisy.child_frame_id = msg.child_frame_id
        noisy.pose.pose.position.x = out_x
        noisy.pose.pose.position.y = out_y
        noisy.pose.pose.position.z = msg.pose.pose.position.z
        qx, qy, qz, qw = yaw_to_quaternion(out_theta)
        noisy.pose.pose.orientation.x = qx
        noisy.pose.pose.orientation.y = qy
        noisy.pose.pose.orientation.z = qz
        noisy.pose.pose.orientation.w = qw
        noisy.pose.covariance = msg.pose.covariance

        noisy.twist.twist.linear.x = v_noisy
        noisy.twist.twist.linear.y = msg.twist.twist.linear.y
        noisy.twist.twist.linear.z = msg.twist.twist.linear.z
        noisy.twist.twist.angular.x = msg.twist.twist.angular.x
        noisy.twist.twist.angular.y = msg.twist.twist.angular.y
        noisy.twist.twist.angular.z = w_noisy
        noisy.twist.covariance = msg.twist.covariance

        self.pub_odom_raw.publish(noisy)

    # ------------------------------------------------------------------- imu
    def imu_cb(self, msg):
        if not self.enable_noise:
            self.pub_imu_raw.publish(msg)
            return

        noisy = Imu()
        noisy.header = msg.header

        # Angular-rate white noise
        noisy.angular_velocity = msg.angular_velocity
        noisy.angular_velocity.z = (
            msg.angular_velocity.z + np.random.normal(0.0, self.sigma_imu_w))
        noisy.angular_velocity_covariance = msg.angular_velocity_covariance

        # Orientation: add corresponding integrated-rate heading jitter
        true_theta = get_yaw_from_quaternion(msg.orientation)
        out_theta = normalize_angle(
            true_theta + np.random.normal(0.0, self.sigma_theta))
        qx, qy, qz, qw = yaw_to_quaternion(out_theta)
        noisy.orientation = msg.orientation
        noisy.orientation.x = qx
        noisy.orientation.y = qy
        noisy.orientation.z = qz
        noisy.orientation.w = qw
        noisy.orientation_covariance = msg.orientation_covariance

        # Linear acceleration: relay untouched (no bias needed for this demo)
        noisy.linear_acceleration = msg.linear_acceleration
        noisy.linear_acceleration_covariance = msg.linear_acceleration_covariance

        self.pub_imu_raw.publish(noisy)


def main(args=None):
    rclpy.init(args=args)
    node = SensorRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
