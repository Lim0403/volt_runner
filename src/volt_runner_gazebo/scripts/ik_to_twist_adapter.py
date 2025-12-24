# 파일 경로: volt_runner/src/volt_runner_gazebo/volt_runner_gazebo/ik_to_twist_adapter.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
import math

# --- ⭐주의: 이 파라미터는 고객님의 MecanumIKNode의 파라미터와 정확히 일치해야 합니다. ⭐---
# 임시 값입니다. 실제 로봇의 설계에 맞춰 수정해야 합니다.
WHEEL_RADIUS = 0.04       # 바퀴 반지름 (미터)
HALF_BASE_WIDTH = 0.1     # 로봇 중심에서 좌우 바퀴 중심까지의 거리 (l)
HALF_BASE_LENGTH = 0.088    # 로봇 중심에서 앞뒤 바퀴 중심까지의 거리 (w)
L_PLUS_W = HALF_BASE_WIDTH + HALF_BASE_LENGTH
# -----------------------------------------------------------------------------------

# 휠 순서: [FL, FR, RL, RR] (고객님의 Float32MultiArray 순서에 맞춰야 함)
# 방향 보정: FK 계산 시 RPM이 로봇 좌표계와 일치하도록 보정 (IK 노드의 역순)
RPM_DIRECTION_CORRECTION = [-1.0, 1.0, 1.0, -1.0] 

class IkToTwistAdapter(Node):
    def __init__(self):
        super().__init__('ik_to_twist_adapter')
        
        # 1. 고객님의 IK Node가 발행하는 목표 RPM 구독
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/wheel_target_rpm',
            self.rpm_callback,
            10
        )
        
        # 2. Gazebo가 구독할 최종 명령 Twist 토픽 발행
        self.publisher = self.create_publisher(
            Twist, 
            '/cmd_vel_gz', # Gazebo 브릿지로 전달될 토픽
            10
        )
        
        self.get_logger().info('IK to Twist Adapter Node Started. Ready to perform Forward Kinematics.')

    def rpm_to_rad_s(self, rpm):
        """RPM을 rad/s로 변환합니다."""
        return rpm * (2.0 * math.pi / 60.0)

    def rpm_callback(self, msg):
        
        if len(msg.data) != 4:
            self.get_logger().error("RPM array must contain 4 values.")
            return

        # 1. 목표 RPM에 방향 보정 적용
        target_rpm_corrected = [msg.data[i] * RPM_DIRECTION_CORRECTION[i] for i in range(4)]
        
        # 2. RPM -> rad/s 변환
        omega = [self.rpm_to_rad_s(r) for r in target_rpm_corrected]
        
        # 휠 순서: omega[0]=FL, omega[1]=FR, omega[2]=RL, omega[3]=RR

        # 3. 순기구학 (FK) 계산: 목표 RPM이 발생시키는 이상적인 Twist
        # Vx = (R/4) * (wFL + wFR + wRL + wRR)
        # Vy = (R/4) * (-wFL + wFR + wRL - wRR)
        # Wz = (R/(4*(l+w))) * (-wFL + wFR - wRL + wRR)
        
        Vx = WHEEL_RADIUS / 4.0 * (omega[0] + omega[1] + omega[2] + omega[3])
        Vy = WHEEL_RADIUS / 4.0 * (-omega[0] + omega[1] + omega[2] - omega[3])
        Wz = WHEEL_RADIUS / (4.0 * L_PLUS_W) * (-omega[0] + omega[1] - omega[2] + omega[3])

        # 4. Twist 메시지 발행
        twist_msg = Twist()
        twist_msg.linear.x = Vx
        twist_msg.linear.y = Vy
        twist_msg.angular.z = Wz
        self.publisher.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = IkToTwistAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()