#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32


class RasterSearchNode(Node):
    def __init__(self):
        super().__init__("raster_search_node")

        self.declare_parameter("pt_topic", "/coil_efficiency")
        self.declare_parameter("cmd_topic", "/cmd_vel_raster")

        self.declare_parameter("search_vx", 0.12)
        self.declare_parameter("search_vy", 0.20)

        self.declare_parameter("lane_time", 2.0)
        self.declare_parameter("forward_time", 0.25)

        self.declare_parameter("pt_detect_threshold", 0.15)
        self.declare_parameter("control_hz", 10.0)

        pt_topic = self.get_parameter("pt_topic").value
        cmd_topic = self.get_parameter("cmd_topic").value

        self.search_vx = float(self.get_parameter("search_vx").value)
        self.search_vy = float(self.get_parameter("search_vy").value)
        self.lane_time = float(self.get_parameter("lane_time").value)
        self.forward_time = float(self.get_parameter("forward_time").value)
        self.pt_detect_threshold = float(self.get_parameter("pt_detect_threshold").value)

        self.latest_pt = None
        self.detected = False

        self.phase = "lane"
        self.direction = -1.0
        self.phase_start_time = None
        self.started = False

        self.sub_pt = self.create_subscription(Float32, pt_topic, self.pt_callback, 10)
        self.pub_cmd = self.create_publisher(Twist, cmd_topic, 10)

        period = 1.0 / float(self.get_parameter("control_hz").value)
        self.timer = self.create_timer(period, self.loop)

        self.get_logger().info(f"Subscribe Pt: {pt_topic}")
        self.get_logger().info(f"Publish raster cmd: {cmd_topic}")
        self.get_logger().info(
            f"Raster config: search_vx={self.search_vx}, search_vy={self.search_vy}, "
            f"lane_time={self.lane_time}, forward_time={self.forward_time}, "
            f"pt_detect_threshold={self.pt_detect_threshold}"
        )

    def pt_callback(self, msg):
        self.latest_pt = float(msg.data)

        if not self.started:
            self.started = True
            self.phase = "lane"
            self.direction = -1.0
            self.phase_start_time = self.get_clock().now()
            self.get_logger().info("Raster started on first Pt message")

        self.detected = self.latest_pt >= self.pt_detect_threshold

    def elapsed(self):
        if self.phase_start_time is None:
            return 0.0
        now = self.get_clock().now()
        return (now - self.phase_start_time).nanoseconds * 1e-9

    def switch_phase(self, phase):
        self.phase = phase
        self.phase_start_time = self.get_clock().now()

    def loop(self):
        cmd = Twist()

        if not self.started:
            self.pub_cmd.publish(cmd)
            return

        if self.detected:
            self.pub_cmd.publish(cmd)
            return

        if self.phase == "lane":
            cmd.linear.x = 0.0
            cmd.linear.y = self.direction * self.search_vy
            cmd.angular.z = 0.0

            if self.elapsed() >= self.lane_time:
                self.switch_phase("forward")

        elif self.phase == "forward":
            cmd.linear.x = self.search_vx
            cmd.linear.y = 0.0
            cmd.angular.z = 0.0

            if self.elapsed() >= self.forward_time:
                self.direction *= -1.0
                self.switch_phase("lane")

        self.pub_cmd.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = RasterSearchNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
