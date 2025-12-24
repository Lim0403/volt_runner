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

    # 기본 경로들
    mecanum_model = PathJoinSubstitution([pkg_share, 'urdf', 'robots', 'mecanum.urdf.xacro'])
    car_model     = PathJoinSubstitution([pkg_share, 'urdf', 'mech', 'car.urdf.xacro'])
    default_rviz  = PathJoinSubstitution([pkg_share, 'rviz', 'description.rviz'])

    # ===== Args =====
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation clock')
    declare_open_rviz    = DeclareLaunchArgument('rviz',          default_value='true',  description='Open RViz')
    # 자동차 표시 온/오프
    declare_show_car     = DeclareLaunchArgument('show_car',      default_value='true',  description='Visualize car model in RViz')
    # 자동차 배치 오프셋(메카넘 기준 +x 2.0m)
    declare_car_offset_x = DeclareLaunchArgument('car_offset_x',  default_value='2.0')
    declare_car_offset_y = DeclareLaunchArgument('car_offset_y',  default_value='0.0')
    declare_car_offset_z = DeclareLaunchArgument('car_offset_z',  default_value='0.15')
    # 메카넘 루트 프레임 이름(환경에 맞게 base_footprint 또는 base_link)
    declare_parent_frame = DeclareLaunchArgument('parent_frame',  default_value='base_footprint')
    # 자동차 루트(당신의 URDF 기준)
    declare_car_root     = DeclareLaunchArgument('car_root',      default_value='car_base_footprint')

    # ===== Nodes =====
    # (1) 메카넘 robot_state_publisher (기존과 동일한 로직; 별도 joint_state_publisher는 기존 런치 파일에 그대로 둔다)
    mecanum_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='mecanum_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_description': ParameterValue(
                Command([FindExecutable(name='xacro'), ' ', mecanum_model]),
                value_type=str
            )
        }],
    )

    # (2) 자동차 robot_state_publisher (네임스페이스 /car → /car/robot_description)
    car_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='car',
        name='car_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_description': ParameterValue(
                Command([FindExecutable(name='xacro'), ' ', car_model]),
                value_type=str
            )
        }],
        condition=IfCondition(LaunchConfiguration('show_car')),
    )

    # (NEW) car용 joint_state_publisher
    # (NEW) car용 joint_state_publisher
    car_joint_state_pub = Node(
    package='joint_state_publisher',
    executable='joint_state_publisher',
    namespace='car',
    name='car_joint_state_publisher',
    output='screen',
    parameters=[{
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        # JSP도 car의 URDF를 읽어야 함
        'robot_description': ParameterValue(
            Command([FindExecutable(name='xacro'), ' ',car_model]),
            value_type=str
        ),
        # 필요 시 기본자세 발행 주기(Hz). 없으면 기본값 사용
        'rate': 30.0
    }],
    # car 표시를 끌 땐 JSP도 같이 끔
    condition=IfCondition(LaunchConfiguration('show_car')),
)



    # (3) 메카넘 ↔ 자동차 정적 TF (x=+2m, 회전 0) — 회전은 0이라 순서 이슈 없음
    static_tf_car = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='car_static_tf',
        output='screen',
        arguments=[
            LaunchConfiguration('car_offset_x'),
            LaunchConfiguration('car_offset_y'),
            LaunchConfiguration('car_offset_z'),
            '1.57', '0', '0',  # roll pitch yaw (0으로 고정)
            'odom',
            # LaunchConfiguration('parent_frame'),
            LaunchConfiguration('car_root'),
        ],
        condition=IfCondition(LaunchConfiguration('show_car')),
    )

    # (4) RViz
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
        # Args
        declare_use_sim_time,
        declare_open_rviz,
        declare_show_car,
        declare_car_offset_x,
        declare_car_offset_y,
        declare_car_offset_z,
        declare_parent_frame,
        declare_car_root,
        # Nodes
        mecanum_state_pub,
        car_joint_state_pub,
        car_state_pub,
        static_tf_car,
        rviz,
    ])
