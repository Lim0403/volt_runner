#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from volt_runner_performance.msg import PerfCommand

class PerfPublisher(Node):  # 클래스 이름 변경
    def __init__(self):
        super().__init__('perf_publisher')  # 노드 이름 변경
        
        # 1. 조종 명령 수신 (Nav2, Manual, RL)
        self.create_subscription(Twist, '/cmd_vel_nav2', self.cb_nav2, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cb_manual, 10)
        self.create_subscription(Twist, '/cmd_vel_rl', self.cb_rl, 10)
        
        # 2. 성능 데이터 발행 (/perf)
        self.pub_perf = self.create_publisher(PerfCommand, '/perf', 10)
        
        self.get_logger().info("PerfPublisher Started. Forwarding cmd_vel with Timestamp...")

    def send_perf_packet(self, msg, source_name):
        new_msg = PerfCommand()
        new_msg.cmd_vel = msg
        new_msg.source = source_name
        
        # [핵심] 출발 시간(Ts) 찍기
        new_msg.time_sent = self.get_clock().now().to_msg()
        
        self.pub_perf.publish(new_msg)

    def cb_nav2(self, msg): self.send_perf_packet(msg, "nav2")
    def cb_manual(self, msg): self.send_perf_packet(msg, "manual")
    def cb_rl(self, msg): self.send_perf_packet(msg, "rl")

def main(args=None):
    rclpy.init(args=args)
    node = PerfPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
