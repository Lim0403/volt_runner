import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    # 패키지 기본 경로
    pkg_nav = get_package_share_directory('volt_runner_navigation')

    # ----- Launch Arguments -----
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_rviz = LaunchConfiguration('rviz')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # 기본 map: 패키지 안 maps/map_tb3.yaml
    declare_map = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(
            pkg_nav,
            'maps',
            'map_tb3.yaml'
        ),
        description='Full path to map YAML file'
    )

    # 기본 params: 패키지 안 config/navigation.yaml (mecanum 설정 통합)
    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            pkg_nav,
            'config',
            'navigation.yaml'
        ),
        description='Full path to Nav2 params YAML file'
    )

    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz2 if true'
    )

    # ----- Nav2 Bringup 래핑 -----
    nav2_bringup_launch = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'bringup_launch.py'
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_bringup_launch),
        launch_arguments={
            'map': map_yaml,
            'params_file': params_file,
            'use_sim_time': use_sim_time,
            'use_composition': 'False',
            'autostart': 'True',
        }.items(),
    )

    # ----- (옵션) RViz -----
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_map,
        declare_params,
        declare_rviz,
        nav2_bringup,
        rviz_node,
    ])
