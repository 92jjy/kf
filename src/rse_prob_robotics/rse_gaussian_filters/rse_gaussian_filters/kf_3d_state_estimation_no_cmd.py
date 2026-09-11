# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

import rclpy

import numpy as np

from rclpy.node import Node

from rse_motion_models.velocity_motion_models import velocity_motion_model
from rse_observation_models.odometry_observation_models import odometry_observation_model

from .filters.kf import KalmanFilter
from .kf_node import KalmanFilterNode


def main(args=None):
    rclpy.init(args=args)

    # 噪声参数通过 ROS 参数暴露，便于在不同测试场景下在线调参，
    # 无需修改源码（直线测试可经 --ros-args -p proc_noise_std:=[...] 覆盖）。
    # 注意：kf.py 内部变量名 R=过程噪声、Q=观测噪声（与教科书命名相反），
    # 但入口参数语义按数学惯例：proc_noise_std=过程噪声，obs_noise_std=观测噪声。
    param_node = Node('kf_param_loader')
    param_node.declare_parameter(
        'proc_noise_std', [0.05, 0.05, 0.03])   # 过程噪声 [x, y, theta]
    param_node.declare_parameter(
        'obs_noise_std', [0.15, 0.15, 0.10])    # 观测噪声 [x, y, theta]
    param_node.declare_parameter('straight_test', False)
    param_node.declare_parameter('angular_rate_threshold', 0.05)

    proc_noise_std = list(param_node.get_parameter('proc_noise_std').value)
    obs_noise_std = list(param_node.get_parameter('obs_noise_std').value)
    straight_test = bool(param_node.get_parameter('straight_test').value)
    angular_rate_threshold = float(
        param_node.get_parameter('angular_rate_threshold').value)
    param_node.destroy_node()

    # Initialize the Kalman Filter
    mu0 = np.zeros(3)
    Sigma0 = np.eye(3)
    # 过程噪声：速度积分（运动模型）的不确定度
    # 观测噪声：轮式里程计位姿测量的不确定度
    # 历史问题：曾把两者倒置（obs_std=1000 → 增益K≈0 → 开环发散）

    kf = KalmanFilter(mu0, Sigma0,
                      velocity_motion_model,
                      odometry_observation_model,
                      proc_noise_std=proc_noise_std,
                      obs_noise_std=obs_noise_std)

    kalman_filter_node = KalmanFilterNode(kf)
    # 将直线测试/失效检测参数传递给滤波节点
    kalman_filter_node.set_parameters([
        rclpy.parameter.Parameter('straight_test',
                                 rclpy.Parameter.Type.BOOL, straight_test),
        rclpy.parameter.Parameter('angular_rate_threshold',
                                 rclpy.Parameter.Type.DOUBLE,
                                 angular_rate_threshold),
    ])
    rclpy.spin(kalman_filter_node)
    kalman_filter_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
