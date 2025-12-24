# volt_runner_navigation/launch/slam.launch.py

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 패키지 경로
    pkg_nav = get_package_share_directory('volt_runner_navigation')
    pkg_gz = get_package_share_directory('volt_runner_gazebo')

    # use_sim_time 인자
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true',
    )

    # RViz 설정 파일 (이건 설치되어 있다고 가정)
    rviz_config_path = os.path.join(pkg_nav, 'rviz', 'slam.rviz')

    # ---------- Gazebo + volt_runner ----------
    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gz, 'launch', 'gazebo_tb3_world.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
    )

    # ---------- SLAM Toolbox ----------
    # ✅ 여기서 YAML 안 쓰고, 파라미터를 직접 지정한다.
    # 이 상태에서 ros2 param get /slam_toolbox map_frame 했을 때
    # 값이 안 나오면, 진짜로 이 파일이 안 쓰이고 있는 거다.
    start_slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'odom_frame': 'odom',
            'map_frame': 'map',
            'base_frame': 'base_footprint',
            'scan_topic': '/scan',
            'mode': 'mapping',
            'transform_publish_period': 0.05,  # > 0 이어야 TF 나옴
            'map_publish_period': 1.0,
            'debug_marker_from_launch': 'volt_runner_slam_launch_applied'
        }],
    )

    # ---------- RViz ----------
    start_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d',
            rviz_config_path,
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(start_gazebo)
    ld.add_action(start_slam)
    ld.add_action(start_rviz)

    return ld
