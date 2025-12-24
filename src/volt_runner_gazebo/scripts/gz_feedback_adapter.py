#!/usr/bin/python3
# 파일 경로: volt_runner/src/volt_runner_gazebo/volt_runner_gazebo/gz_feedback_adapter.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
import math

# --- ⭐주의: 이 설정은 URDF 파일과 고객님 OdomPublisher의 방향 설정에 일치해야 합니다. ⭐---
# URDF에 정의된 조인트 이름
WHEEL_JOINT_NAMES = [
    'front_left_wheel_joint', 
    'front_right_wheel_joint', 
    'rear_left_wheel_joint', 
    'rear_right_wheel_joint'
]
# wheel_feedback의 순서: [FL, FR, RL, RR]에 맞춰 RPM에 적용할 방향 보정
RPM_DIRECTION_CORRECTION = [-1, 1, -1, 1] 
# -----------------------------------------------------------------------------------

class GZFeedbackAdapter(Node):
    def __init__(self):
        super().__init__('gz_feedback_adapter')
        
        # 1. ROS 2로 브릿징된 JointState 토픽 구독
        self.subscription = self.create_subscription(
            JointState,
            '/ros_joint_states', # ros_gz_bridge에서 받은 JointState 토픽
            self.joint_state_callback,
            10
        )
        
        # 2. 고객님의 OdomPublisher가 구독할 RPM 토픽 발행
        self.publisher = self.create_publisher(
            Float32MultiArray, 
            '/wheel_feedback', 
            10
        )
        
        # 휠 이름과 인덱스 매핑 (JointState 메시지 처리를 위해)
        self.joint_name_to_index = {name: i for i, name in enumerate(WHEEL_JOINT_NAMES)}
        
        self.get_logger().info('GZ Feedback Adapter Node Started. Ready to bridge rad/s to RPM.')


    def joint_state_callback(self, msg):
        
        # JointState 메시지에서 4개 휠의 속도(rad/s)를 추출합니다.
        wheel_speeds_rad_s = [0.0] * 4 
        
        try:
            for joint_name, target_index in self.joint_name_to_index.items():
                
                try:
                    # JointState의 이름 리스트에서 조인트 이름의 인덱스를 찾습니다.
                    msg_index = msg.name.index(joint_name)
                    # 해당 인덱스의 속도(velocity, rad/s)를 가져와 배열에 저장합니다.
                    wheel_speeds_rad_s[target_index] = msg.velocity[msg_index]
                except ValueError:
                    self.get_logger().warn(f"Joint {joint_name} not found in JointState message.")
                    return # 데이터가 불완전하므로 처리 중단
                    
        except Exception as e:
            self.get_logger().error(f"Error processing JointState: {e}")
            return
            
        
        # 3. rad/s를 RPM으로 변환 및 방향 보정
        rpm_list = []
        for i, rad_s in enumerate(wheel_speeds_rad_s):
            # rad/s -> RPM 변환: (rad/s * 60) / (2 * pi)
            rpm = rad_s * 60.0 / (2 * math.pi)
            
            # 방향 보정 적용 (고객님의 OdomPublisher 로직과 일치하도록)
            rpm_corrected = rpm * RPM_DIRECTION_CORRECTION[i]
            
            rpm_list.append(rpm_corrected)

        # 4. /wheel_feedback (Float32MultiArray) 메시지 발행
        msg_out = Float32MultiArray()
        msg_out.data = rpm_list
        self.publisher.publish(msg_out)


def main(args=None):
    rclpy.init(args=args)
    node = GZFeedbackAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()