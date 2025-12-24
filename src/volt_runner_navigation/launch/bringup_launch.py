#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_name = 'volt_runner_navigation'
    pkg_share = get_package_share_directory(package_name)

    # ===== 기본 파일 경로 =====
    default_params_file = os.path.join(pkg_share, 'config', 'navigation.yaml')
    default_map_file = os.path.join(pkg_share, 'maps', 'map_tb3.yaml')

    # ===== Launch Arguments =====
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    map_yaml = LaunchConfiguration('map')

    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock'
    )

    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically start lifecycle nodes'
    )

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to Nav2 params YAML file'
    )

    declare_map_yaml = DeclareLaunchArgument(
        'map',
        default_value=default_map_file,
        description='Full path to map YAML file (static map)'
    )

    # ===== Nav2 Core Nodes =====
    # navigation.yaml에 각 노드 설정이 들어있고,
    # 여기서는 그 파일을 공통 params로 넘겨준다.
    common_params = [params_file, {'use_sim_time': use_sim_time}]

    # --- Map Server (static map) ---
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace=namespace,
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'yaml_filename': map_yaml},
            params_file,
        ],
    )

    # --- AMCL ---
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        namespace=namespace,
        output='screen',
        parameters=common_params,
    )

    # --- Planner Server ---
    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        namespace=namespace,
        output='screen',
        parameters=common_params,
    )

    # --- Controller Server ---
    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        namespace=namespace,
        output='screen',
        parameters=common_params,
    )

    # --- Smoother Server ---
    smoother_server_node = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        namespace=namespace,
        output='screen',
        parameters=common_params,
    )

    # --- Behavior Server ---
    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        namespace=namespace,
        output='screen',
        parameters=common_params,
    )

    # --- BT Navigator ---
    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        namespace=namespace,
        output='screen',
        parameters=common_params,
    )

    # --- Waypoint Follower ---
    waypoint_follower_node = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        namespace=namespace,
        output='screen',
        parameters=common_params,
    )

    # --- Velocity Smoother ---
    velocity_smoother_node = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        namespace=namespace,
        output='screen',
        parameters=common_params,
    )

    # --- Collision Monitor ---
    collision_monitor_node = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        namespace=namespace,
        output='screen',
        parameters=common_params,
    )

    # ===== Lifecycle Managers =====
    # navigation.yaml 의 lifecycle_manager_* 설정과 node_names를 맞춘 상태여야 함.
    lifecycle_manager_localization_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        namespace=namespace,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': ['map_server', 'amcl'],
        }],
    )

    lifecycle_manager_navigation_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        namespace=namespace,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': [
                'controller_server',
                'planner_server',
                'smoother_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
                'velocity_smoother',
                'collision_monitor',
            ],
        }],
    )

    return LaunchDescription([
        declare_namespace,
        declare_use_sim_time,
        declare_autostart,
        declare_params_file,
        declare_map_yaml,

        map_server_node,
        amcl_node,
        planner_server_node,
        controller_server_node,
        smoother_server_node,
        behavior_server_node,
        bt_navigator_node,
        waypoint_follower_node,
        velocity_smoother_node,
        collision_monitor_node,
        lifecycle_manager_localization_node,
        lifecycle_manager_navigation_node,
    ])
