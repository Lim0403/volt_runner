import os
import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # --- 1. 경로 및 런치 인자 설정 ---
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty.sdf',
        description='World file name (in volt_runner_gazebo/worlds)'
    )
    spawn_x_arg = DeclareLaunchArgument('spawn_x', default_value='0.0')
    spawn_y_arg = DeclareLaunchArgument('spawn_y', default_value='0.0')
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
    # [가정] car.urdf.xacro의 최상위(root) 링크 이름이 'base_link'라고 가정합니다.
    # 'frame_prefix'가 'my_car/'이므로, ROS TF 상의 이름은 'my_car/base_link'가 됩니다.
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

    # --- [신규 추가] 5c. CAR의 고정 위치 발행 (Static TF) ---
    # Gazebo 스폰 위치(7번 항목)와 동일하게 odom 프레임에 my_car의 위치를 고정합니다.
    static_car_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_car_tf_publisher',
        arguments=[
            '--x', '2.0',
            '--y', '0.0',
            '--z', '0.15',
            '--yaw', '1.57',
            '--pitch', '0',
            '--roll', '0',
            '--frame-id', 'odom',           # 부모 프레임 (odom)
            '--child-frame-id', car_root_frame  # 자식 프레임 (my_car/base_link)
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
    
    # --- 7. CAR 스폰 (string 사용) ---
    create_car = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'my_car',
            '-string', car_xml, 
            '-x', '2.0', # static_car_tf(5c)와 동일
            '-y', '0.0', # static_car_tf(5c)와 동일
            '-z', '0.15', # static_car_tf(5c)와 동일
            '-Y', '1.57', # static_car_tf(5c)와 동일
        ],
        output='screen'
    )
    
    # --- 8. volt_runner용 Parameter Bridge ---
    # --- 8. volt_runner용 Parameter Bridge (모든 기능 포함) ---
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name='gz_parameter_bridge',
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/odom/unfiltered@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
            
            # --- [수정됨] /scan 토픽에 QoS 설정을 강제로 추가 ---
            # Gazebo(Best Effort)와 ROS 2(Reliable)의 QoS 불일치 문제를 해결합니다.
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan" +
            "?reliability=best_effort" +
            "&durability=volatile" +
            "&history=keep_last" +
            "&depth=1"
        ],
        remappings=[]
    )
    # gz_bridge = Node(
    #     package="ros_gz_bridge",
    #     executable="parameter_bridge",
    #     name='gz_parameter_bridge',
    #     arguments=[
    #         "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
    #         "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
    #         "/odom/unfiltered@nav_msgs/msg/Odometry[gz.msgs.Odometry",
    #         "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
    #         "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
    #     ],
    #     remappings=[]
    # )
    
    # --- 9. EKF 노드 ---
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

    # --- 10. 런치 파일 반환 ---
    return LaunchDescription([
        world_arg, spawn_x_arg, spawn_y_arg, spawn_z_arg, spawn_yaw_arg,
        
        gz,          
        rsp,         
        car_rsp,     
        static_car_tf, # <-- [추가] 고정 TF 발행기
        create_robot, 
        gz_bridge,   
        create_car,
        
        robot_localization_node
    ])