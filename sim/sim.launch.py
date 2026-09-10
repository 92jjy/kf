
"""One-click Gazebo simulation launch.

Starts: Gazebo (world) + robot_state_publisher (TF chain) +
robot spawn + sensor relay (/odom -> /odom_raw, IMU -> /imu/data_raw).

Run:
    ros2 launch /home/jjy/ros2_ws/sim/sim.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

SIM_DIR = os.path.dirname(os.path.abspath(__file__))


def generate_launch_description():
    world_file = os.path.join(SIM_DIR, 'test_world.world')
    urdf_file = os.path.join(SIM_DIR, 'test_robot.urdf')
    relay_file = os.path.join(SIM_DIR, 'relay_node.py')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'),
                         'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description, 'use_sim_time': True}
        ],
    )

    # Publishes /joint_states for wheel joints (diff_drive plugin does not),
    # keeping the TF tree complete in RViz.
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )
    
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=[
            '-file', urdf_file,
            '-entity', 'test_robot',
            '-z', '0.15',
        ],
    )

    # /odom -> /odom_raw, /imu_plugin/out -> /imu/data_raw (KF node inputs).
    # relay_node imports rse_common_utils for angle helpers, so make sure the
    # workspace install path is on PYTHONPATH even if the workspace was not
    # sourced in the terminal that ran `ros2 launch`.
    relay_pythonpath = os.path.join(
        SIM_DIR, '..', 'install', 'rse_common_utils',
        'lib', 'python3.10', 'site-packages')
    relay = ExecuteProcess(
        cmd=['python3', relay_file],
        output='screen',
        additional_env={
            'PYTHONPATH': os.path.abspath(relay_pythonpath)
                          + os.pathsep + os.environ.get('PYTHONPATH', '')
        },
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        joint_state_publisher,
        spawn,
        relay,
    ])
