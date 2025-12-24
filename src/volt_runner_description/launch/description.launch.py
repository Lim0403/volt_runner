#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare



def generate_launch_description():
    pkg_share = FindPackageShare('volt_runner_description').find('volt_runner_description')
    default_model = PathJoinSubstitution([pkg_share, 'urdf', 'robots', 'mecanum.urdf.xacro'])
    default_rviz  = PathJoinSubstitution([pkg_share, 'rviz', 'description.rviz'])

    declare_model = DeclareLaunchArgument(
        'model',
        default_value=default_model,
        description='Absolute path to URDF/Xacro file',
    )
    declare_publish_joints = DeclareLaunchArgument(
        'publish_joints',
        default_value='true',
        description='Run joint_state_publisher',
    )
    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Open RViz',
    )
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock',
    )

    joint_state_pub = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=IfCondition(LaunchConfiguration('publish_joints')),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen',
    )

    robot_state_pub = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    name='robot_state_publisher',
    output='screen',
    parameters=[{
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        'robot_description': ParameterValue(
            Command([FindExecutable(name='xacro'), ' ', LaunchConfiguration('model')]),
            value_type=str
        )
    }],
    )


    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    return LaunchDescription([
        declare_model,
        declare_publish_joints,
        declare_rviz,
        declare_use_sim_time,
        joint_state_pub,
        robot_state_pub,
        rviz,
    ])
