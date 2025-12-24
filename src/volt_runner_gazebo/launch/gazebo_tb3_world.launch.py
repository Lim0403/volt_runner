import os
import xacro
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,          # ✅ 추가
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    # --- 1. 경로 및 런치 인자 설정 ---
    world_arg = DeclareLaunchArgument(
        'world',
        # [수정] 월드 기본값을 'turtlebot3_world.sdf'로 변경
        default_value='turtlebot3_world.sdf', 
        description='World file name (in volt_runner_gazebo/worlds)'
    )
    # [수정] TB3 월드에 맞게 로봇 스폰 위치 변경 (기존 파일 참고)
    spawn_x_arg = DeclareLaunchArgument('spawn_x', default_value='-1.5')
    spawn_y_arg = DeclareLaunchArgument('spawn_y', default_value='-1.0')
    spawn_z_arg = DeclareLaunchArgument('spawn_z', default_value='0.15')
    spawn_yaw_arg = DeclareLaunchArgument('spawn_yaw', default_value='0.0', description='Yaw (rad)')

    gz_pkg_path   = get_package_share_directory('ros_gz_sim')
    vr_pkg_path   = get_package_share_directory('volt_runner_gazebo')
    desc_pkg_path = get_package_share_directory('volt_runner_description') 
    
    world_path  = PathJoinSubstitution([vr_pkg_path, 'worlds', LaunchConfiguration('world')])
    
    ekf_config_path = PathJoinSubstitution(
        [vr_pkg_path, "config", "ekf.yaml"]
    )

    # --- 2. volt_runner (Mecanum) URDF 처리 ---
    robot_xacro = os.path.join(desc_pkg_path, 'urdf', 'robots', 'mecanum.urdf.xacro')
    robot_xml   = xacro.process_file(robot_xacro).toxml()

    # --- 3. CAR (Static Prop) URDF 처리 ---
    car_xacro_file = os.path.join(desc_pkg_path, 'urdf', 'mech', 'car.urdf.xacro')
    car_xml = xacro.process_file(car_xacro_file).toxml()
    car_root_frame = 'my_car/car_base_footprint' 

    # --- 4. Gazebo 런치 파일 실행 ---
    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gz_pkg_path, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': world_path}.items()
    )

    # --- 5. volt_runner용 Robot State Publisher (RSP) ---
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_xml, 'use_sim_time': True}],
        output='screen'
    )

    # --- 5b. CAR용 Robot State Publisher (RSP) ---
    car_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='car_state_publisher', 
        namespace='my_car', 
        parameters=[
            {'robot_description': car_xml, 'use_sim_time': True},
            {'frame_prefix': 'my_car/'} 
        ],
        output='screen'
    )

    # --- [수정] 5c. CAR의 고정 위치 발행 (Static TF) ---
    static_car_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_car_tf_publisher',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.15',
            '--yaw', '0.0',
            '--pitch', '0',
            '--roll', '0',
            '--frame-id', 'odom',
            '--child-frame-id', car_root_frame
        ]
    )

    # --- 6. volt_runner 스폰 (topic 사용) ---
    create_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'volt_runner',
            '-topic', 'robot_description', 
            '-x', LaunchConfiguration('spawn_x'),
            '-y', LaunchConfiguration('spawn_y'),
            '-z', LaunchConfiguration('spawn_z'),
            '-Y', LaunchConfiguration('spawn_yaw'),
        ],
        output='screen'
    )
    
    # --- [수정] 7. CAR 스폰 (string 사용) ---
    create_car = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'my_car',
            '-string', car_xml, 
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.15',
            '-Y', '0.0',
        ],
        output='screen'
    )
    
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name='gz_parameter_bridge',
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/cmd_vel_out@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/odom/unfiltered@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan", 
            "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
        ],
        remappings=[]
    )
    
    # --- 9. EKF 노드 (TF 발행용) ---
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            {'use_sim_time': True}, 
            ekf_config_path
        ],
        remappings=[("odometry/filtered", "/odom")] 
    )

    # --- LiDAR frame alias ---
    lidar_tf_fix = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='volt_runner_lidar_tf_fix',
        arguments=[
            '-0.05', '0.0', '0.5',
            '0.0', '0.0', '0.0',
            'base_footprint',
            'volt_runner/base_footprint/base_scan'
        ],
        output='screen'
    )

    # --- Twist Mux ---
    twist_mux_config_path = PathJoinSubstitution(
        [vr_pkg_path, 'config', 'twist_mux.yaml']
    )
    
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            twist_mux_config_path
        ],
    )

    # ✅ coil_pose_publisher: 일반 Python 모듈로 실행
    #   python3 -m volt_runner_gazebo.coil_pose_publisher
    coil_pose_process = ExecuteProcess(
        cmd=[
            'python3',
            '-m',
            'volt_runner_gazebo.coil_pose_publisher',
        ],
        output='screen'
    )

    # --- 10. 런치 파일 반환 ---
    return LaunchDescription([
        world_arg, spawn_x_arg, spawn_y_arg, spawn_z_arg, spawn_yaw_arg,
        
        gz,
        rsp,
        car_rsp,
        static_car_tf,
        create_robot,
        gz_bridge,
        create_car,
        lidar_tf_fix,
        robot_localization_node,
        twist_mux_node,
        coil_pose_process,   # ✅ 여기서 coil_pose_publisher 같이 실행
    ])
