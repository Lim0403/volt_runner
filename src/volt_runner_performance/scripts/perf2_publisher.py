#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from volt_runner_performance.msg import Perf2Command


class Perf2Publisher(Node):
    """
    PC에서 실행.
    - /cmd_vel_nav2, /cmd_vel, /cmd_vel_rl 을 받아서
    - Perf2Command로 감싸 /perf2 로 publish
    - 이때 seq 증가 + t_send_pc(Tsend)만 채워서 보냄
    """
    def __init__(self):
        super().__init__('perf2_publisher')

        # 입력(cmd_vel) 구독
        self.create_subscription(Twist, '/cmd_vel_nav2', self.cb_nav2, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cb_manual, 10)
        self.create_subscription(Twist, '/cmd_vel_rl', self.cb_rl, 10)

        # 출력(perf2) 발행
        self.pub_perf2 = self.create_publisher(Perf2Command, '/perf2', 10)

        # seq (메시지 매칭용)
        self.seq = 0

        self.get_logger().info("Perf2Publisher Started. Forwarding cmd_vel as /perf2 with seq + t_send_pc.")

    def send_perf2_packet(self, twist_msg: Twist, source_name: str):
        pkt = Perf2Command()
        pkt.cmd_vel = twist_msg
        pkt.source = source_name

        # seq 증가
        self.seq += 1
        pkt.seq = self.seq

        # PC 송신 시각(Tsend)
        pkt.t_send_pc = self.get_clock().now().to_msg()

        # 나머지 시간 필드는 RPI/PC 수신 노드가 채울 예정이므로 건드리지 않음
        self.pub_perf2.publish(pkt)

    def cb_nav2(self, msg: Twist):
        self.send_perf2_packet(msg, "nav2")

    def cb_manual(self, msg: Twist):
        self.send_perf2_packet(msg, "manual")

    def cb_rl(self, msg: Twist):
        self.send_perf2_packet(msg, "rl")


def main(args=None):
    rclpy.init(args=args)
    node = Perf2Publisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
