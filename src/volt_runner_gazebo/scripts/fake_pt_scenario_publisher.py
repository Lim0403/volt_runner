#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class FakePtScenarioPublisher(Node):
    def __init__(self):
        super().__init__("fake_pt_scenario_publisher")

        self.pub = self.create_publisher(Float32, "/coil_efficiency", 10)

        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.loop)  # 10 Hz

        self.get_logger().info("Fake Pt scenario started")

    def elapsed(self):
        now = self.get_clock().now()
        return (now - self.start_time).nanoseconds * 1e-9

    def loop(self):
        t = self.elapsed()

        msg = Float32()

        if t < 10.0:
            # 아직 송신 코일을 못 찾은 상태
            msg.data = 0.05

        elif t < 18.0:
            # raster가 송신 코일 근처를 찾아서 RL로 넘어가는 상태
            msg.data = 0.35

        elif t < 25.0:
            # RL 정렬이 진행되면서 Pt가 좋아지는 상태
            msg.data = 0.75

        else:
            # 성공 상태
            msg.data = 0.95

        self.pub.publish(msg)


def main():
    rclpy.init()
    node = FakePtScenarioPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
