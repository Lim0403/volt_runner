# volt_runner_navigation/launch/slam_only.launch.py

import os

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    EnvironmentVariable,
)
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    # slam_toolbox에서 제공하는 공식 launch
    slam_launch_path = PathJoinSubstitution(
        [FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py']
    )

    # 네 패키지의 slam.yaml 사용
    slam_config_path = PathJoinSubstitution(
        [FindPackageShare('volt_runner_navigation'), 'config', 'slam.yaml']
    )

    # RViz 설정 (원하는 거 쓰면 됨)
    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('volt_runner_navigation'), 'rviz', 'slam.rviz']
    )

    # ROS_DISTRO 에 따라 인자 이름 달라지는 처리 (linorobot 방식)
    lc = LaunchContext()
    ros_distro = EnvironmentVariable('ROS_DISTRO').perform(lc)
    slam_param_name = 'slam_params_file'
    if ros_distro == 'foxy':
        slam_param_name = 'params_file'

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('rviz')

    return LaunchDescription([
        # --- Args ---
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time (true for Gazebo)'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Launch RViz2 together with slam_toolbox'
        ),

        # --- slam_toolbox (online_async_launch) ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch_path),
            launch_arguments={
                'use_sim_time': use_sim_time,
                slam_param_name: slam_config_path,
            }.items()
        ),

        # --- RViz2 (옵션) ---
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_slam_only',
            output='screen',
            arguments=['-d', rviz_config_path],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(use_rviz),
        ),
    ])
