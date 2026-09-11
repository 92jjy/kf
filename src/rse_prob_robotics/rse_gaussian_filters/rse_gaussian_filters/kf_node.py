# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.


from rclpy.node import Node

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

import numpy as np

from rse_common_utils.sensor_utils import odom_to_pose2D, get_normalized_pose2D, rotate_pose2D, get_yaw_from_quaternion
from rse_common_utils.helper_utils import normalize_angle
from rse_common_utils.visualization import Visualizer

from .filters.kf import KalmanFilter
from .filters.ekf import ExtendedKalmanFilter
from .filters.ukf import UnscentedKalmanFilter


class KalmanFilterBaseNode(Node):
    def __init__(self, kf):
        super().__init__('kalman_filter_node')

        self.odom_gt_subscription = self.create_subscription(
            Odometry,
            'odom',  # 'wheel_odom',  # Ground Truth
            self.odom_gt_callback,
            10)

        self.odom_raw_subscription = self.create_subscription(
            Odometry,
            'odom_raw',  # 'odom_raw',         # For controls
            self.odom_raw_callback,
            10)

        self.imu_subscriber = self.create_subscription(
            Imu,
            '/imu/data_raw',  # '/imu',
            self.imu_callback,
            10)

        self.kf = kf

        # ---- 纯直线测试：非直线运动失效检测机制 ----
        # straight_test=true 时，节点以 IMU 角速度为独立参考监测转弯：
        # 若 |yaw_rate| 超过阈值则判定为"非直线状态"，记录告警与违规计数。
        # （转弯段内直线KF假设失效，必须在测试报告中标记/剔除这些数据）
        self.declare_parameter('straight_test', False)
        self.declare_parameter('angular_rate_threshold', 0.05)  # rad/s ≈ 2.9°/s
        self.straight_test = bool(self.get_parameter('straight_test').value)
        self.angular_rate_threshold = float(
            self.get_parameter('angular_rate_threshold').value)

        # ---- 观测坐标系安装偏角（坐标系约定）----
        # 约定：ROS/Gazebo 右手系（x 前、y 左、z 上），yaw 绕 z 右手定则(CCW)为正。
        # rotate_pose2D(pose, deg) 为 active 旋转（向量在同一系内绕 z 转 deg）。
        # frame_rotation_deg = 观测源(odom_raw)坐标系相对标准 odom 坐标系的安装偏角，
        # 施加该旋转将观测变换到 odom 系，与真值(/odom，始终在 odom 系，不旋转)对比。
        #   * Gazebo 仿真：relay 直接转发 /odom -> /odom_raw，两话题同坐标系 -> 0°
        #   * linkou 实车包：轮式里程计系与 odom 系相差 90° -> 运行时传 -90
        self.declare_parameter('frame_rotation_deg', 0.0)
        self.turn_violation_count = 0   # 检测到转弯的样本数
        self.straight_sample_count = 0  # 总样本数
        self._last_warn_time = 0.0      # 日志节流用

        # Initialize the visualizer to see the results
        if isinstance(self.kf, (UnscentedKalmanFilter)):
            self.visualizer = Visualizer("alpha = %s, beta = %s, kappa = %s"%(self.kf.alpha, self.kf.beta, self.kf.kappa))
        else:
            self.visualizer = Visualizer()

        # Create a ROS 2 timer for the visualizer updates
        self.visualizer_timer = self.create_timer(0.1, self.update_visualizer)

        self.mu = None
        self.Sigma = None
        self.u = None
        self.z = None
        self.prev_time = None  # previous prediction time, used to compute the delta_t

        # Variables to normalize the pose (always start at the origin)
        self.initial_pose = None
        self.normalized_pose = (0.0, 0.0, 0.0) 

        # IMU data
        self.initial_imu_theta = None
        self.normalized_imu_theta = 0.0
        self.imu_w = 0.0
        self.imu_a_x = 0.0
        self.imu_a_y = 0.0

        self.prev_normalized_pose = (0.0, 0.0, 0.0)
        self.prev_pose_set = None 

        self.initial_gt_pose = None
        self.normalized_gt_pose = (0.0, 0.0, 0.0)

        print("KF ready!")

    def update_visualizer(self):
        # Call the visualizer update asynchronously
        if self.mu is not None and self.Sigma is not None:
            if self.z is not None:
                self.visualizer.update(self.normalized_gt_pose, self.mu, self.Sigma, self.z, step="update")
            else:
                self.visualizer.update(self.normalized_gt_pose, self.mu, self.Sigma, step="predict")

    def odom_raw_callback(self, msg):

        # 观测坐标系安装偏角（仿真同系=0；实车包按需传 -90 等）
        frame_rot = float(self.get_parameter('frame_rotation_deg').value)

        # Set the initial pose
        if not self.initial_pose:
            initial_pose = odom_to_pose2D(msg)

            self.initial_pose = rotate_pose2D(initial_pose, frame_rot)

        # Get and normalize the pose
        current_pose = odom_to_pose2D(msg)
        rotated_pose = rotate_pose2D(current_pose, frame_rot)
        self.normalized_pose = np.array(get_normalized_pose2D(self.initial_pose, rotated_pose))

        self.control = msg.twist.twist

        # Get the control inputs for velocity model
        self.set_control()

        # Compute dt with clamping to prevent spikes from clock jitter
        curr_time = self.get_clock().now().nanoseconds
        if self.prev_time:
            dt = (curr_time - self.prev_time) / 1e9  # Convert nanoseconds to seconds
            dt = max(dt, 0.0)  # Guard against negative dt (clock rollback)
            dt = min(dt, 0.1)   # Clamp to 100ms max (prevents huge jumps on stalls)
        else:
            dt = 0.01

        if isinstance(self.kf, (KalmanFilter, ExtendedKalmanFilter, UnscentedKalmanFilter)):
            self.mu, self.Sigma = self.kf.predict(self.u, dt)
        else:
            inf_vector, inf_matrix = self.kf.predict(self.u, dt)
            self.Sigma = np.linalg.inv(inf_matrix)
            self.mu = self.Sigma @ inf_vector

        self.prev_time = curr_time

        self.set_observation()

        if isinstance(self.kf, (KalmanFilter, ExtendedKalmanFilter, UnscentedKalmanFilter)):
            self.mu, self.Sigma = self.kf.update(self.z, dt)
        else:
            inf_vector, inf_matrix = self.kf.update(self.z, dt)
            self.Sigma = np.linalg.inv(inf_matrix)
            self.mu = self.Sigma @ inf_vector

        # print(f"mu: {self.mu}, Sigma: {self.Sigma}")

        # 非直线运动失效检测（仅在纯直线测试模式下生效）
        # 动态读取参数：允许节点构造后再通过 set_parameters 注入配置
        if self.get_parameter('straight_test').value:
            self._detect_non_straight_motion(dt)

        self.prev_normalized_pose = self.normalized_pose

    def _detect_non_straight_motion(self, dt):
        """检测非直线运动状态（直线测试场景下的算法失效保护）。

        以 IMU 角速度为独立参考（不依赖可能已漂移的里程计）：
        |yaw_rate| 持续超过阈值即判定机器人正在转弯，此时直线运动假设
        失效，记录告警并累计违规数，供测试报告剔除/标记该时段数据。
        """
        self.straight_sample_count += 1
        yaw_rate = abs(self.imu_w)
        threshold = self.get_parameter('angular_rate_threshold').value
        is_turning = yaw_rate > threshold
        if is_turning:
            self.turn_violation_count += 1
            # 日志节流：每 2 秒最多告警一次，避免刷屏
            now = self.get_clock().now().nanoseconds * 1e-9
            if now - self._last_warn_time > 2.0:
                self._last_warn_time = now
                ratio = (self.turn_violation_count /
                         max(self.straight_sample_count, 1)) * 100.0
                self.get_logger().warn(
                    f'[直线测试] 检测到非直线运动! |yaw_rate|={yaw_rate:.3f} '
                    f'rad/s > 阈值 {threshold:.3f}; '
                    f'转弯样本占比 {ratio:.1f}% —— 该时段KF直线假设失效')
        # 每 500 个样本（约 10s）输出一次汇总
        if self.straight_sample_count % 500 == 0:
            ratio = (self.turn_violation_count /
                     self.straight_sample_count) * 100.0
            self.get_logger().info(
                f'[直线测试] 直行合规率: {100.0 - ratio:.1f}% '
                f'({self.straight_sample_count - self.turn_violation_count}'
                f'/{self.straight_sample_count} 样本为直线)')

    def imu_callback(self, msg):

        self.imu_msg = msg

        # Extract the linear acceleration data from the IMU message
        self.imu_a_x = msg.linear_acceleration.x
        self.imu_a_y = msg.linear_acceleration.y

        # Extract the angular velocity data from the IMU message
        self.imu_w = msg.angular_velocity.z

        # Compute the yaw from the IMU quaternion
        imu_theta = get_yaw_from_quaternion(msg.orientation)

        # Calculate the fake theta
        if not self.initial_imu_theta:
            self.initial_imu_theta = imu_theta
        else:
            # Calculate the difference in yaw
            delta_theta = imu_theta - self.initial_imu_theta

            # Unwrap the delta_yaw to avoid issues with angles wrapping around at 2*pi radians
            self.normalized_imu_theta = normalize_angle(delta_theta)

    def odom_gt_callback(self, msg):
        # Set the initial pose
        if not self.initial_gt_pose:

            initial_pose = odom_to_pose2D(msg)  

            self.initial_gt_pose = initial_pose

        # Get and normalize the pose
        current_pose = odom_to_pose2D(msg)
        self.normalized_gt_pose = np.array(get_normalized_pose2D(self.initial_gt_pose, current_pose))

    def set_control(self):
        raise NotImplementedError("This function has to be implemented by a child class")

    def set_observation(self):
        raise NotImplementedError("This function has to be implemented by a child class")


class KalmanFilterNode(KalmanFilterBaseNode):
    def set_control(self):
        self.u = np.asarray([self.control.linear.x, self.control.angular.z])

        # Get the control inputs for odometry model
        ''' if self.prev_pose_set:
            self.u = np.asarray([self.prev_normalized_pose, self.normalized_pose])
        else:
            self.prev_normalized_pose = self.normalized_pose
            self.prev_pose_set = True
            return
        '''

    def set_observation(self):
        self.z = self.normalized_pose


class KalmanFilterFusionNode(KalmanFilterNode):

    def set_observation(self):
        self.z = np.array([[self.normalized_pose[0]], [self.normalized_pose[1]], [self.normalized_pose[2]], [self.normalized_imu_theta], [self.imu_w], [self.imu_a_x], [self.imu_a_y]])