from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    # World arg
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty.sdf',
        description='World file name (in volt_runner_gazebo/worlds)'
    )

    # Spawn pose args
    spawn_x_arg = DeclareLaunchArgument('spawn_x', default_value='0.0')
    spawn_y_arg = DeclareLaunchArgument('spawn_y', default_value='0.0')
    spawn_z_arg = DeclareLaunchArgument('spawn_z', default_value='0.15')
    spawn_yaw_arg = DeclareLaunchArgument('spawn_yaw', default_value='0.0', description='Yaw (rad)')

    gz_pkg   = get_package_share_directory('ros_gz_sim')
    vr_pkg   = get_package_share_directory('volt_runner_gazebo')
    desc_pkg = get_package_share_directory('volt_runner_description')

    world_path  = PathJoinSubstitution([vr_pkg, 'worlds', LaunchConfiguration('world')])
    robot_xacro = os.path.join(desc_pkg, 'urdf', 'robots', 'mecanum.urdf.xacro')
    robot_xml   = xacro.process_file(robot_xacro).toxml()

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gz_pkg, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': world_path}.items()
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_xml, 'use_sim_time': True}],
        output='screen'
    )

    create = Node(
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
    
    # ====================================================
    # (수정) Linorobot 방식의 수동 파라미터 브릿지
    # ====================================================
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name='gz_parameter_bridge',
        arguments=[
            # 플러그인들이 ROS 2 토픽을 직접 발행/구독하므로 
            # 토픽 이름이 겹치지 않도록 ROS 2 쪽 토픽 이름 변경 (예: /odom -> /odom_gz)
            # 하지만 여기서는 플러그인 토픽 이름 자체를 ROS 2와 일치시킴
            
            # 1. Clock (필수)
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            
            # 2. Control (Mecanum 플러그인이 사용)
            "cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",

            # 3. Feedback (Odometry 플러그인이 발행)
            "odom/unfiltered@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            
            # 4. JointStates (JointState 플러그인이 발행)
            "joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
            
            # 5. Laser (로봇의 Lidar 센서)
            # (주의: laser.urdf.xacro 플러그인의 <topic>이 '/scan'이어야 함)
            "scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",

            # 6. TF (Odometry 플러그인이 발행)
            # (주의: odometry 플러그인의 <publish_tf>가 true여야 함)
            "tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V"
        ],
        
        # Gazebo 플러그인이 ROS 2와 동일한 토픽 이름을 사용하므로 
        # remapping은 일단 비워둡니다.
        remappings=[]
    )

    return LaunchDescription([
        world_arg, spawn_x_arg, spawn_y_arg, spawn_z_arg, spawn_yaw_arg,
        gz, rsp, create, gz_bridge # gz_bridge 추가
    ])