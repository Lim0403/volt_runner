#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from std_msgs.msg import Float32


class VirtualPtFromOdomNode(Node):
    def __init__(self):
        super().__init__("virtual_pt_from_odom_node")

        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("pt_topic", "/coil_efficiency")

        # 로봇 시작 위치 기준 가상 코일 위치
        self.declare_parameter("target_x", 0.20)
        self.declare_parameter("target_y", 0.00)
        self.declare_parameter("sigma", 0.02)
        self.declare_parameter("active_radius", 0.08)

        odom_topic = self.get_parameter("odom_topic").value
        pt_topic = self.get_parameter("pt_topic").value

        self.target_x = float(self.get_parameter("target_x").value)
        self.target_y = float(self.get_parameter("target_y").value)
        self.sigma = float(self.get_parameter("sigma").value)
        self.active_radius = float(self.get_parameter("active_radius").value)

        self.have_origin = False
        self.origin_x = 0.0
        self.origin_y = 0.0

        self.sub_odom = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10,
        )

        self.pub_pt = self.create_publisher(
            Float32,
            pt_topic,
            10,
        )

        self.get_logger().info("Virtual Pt from real odom started")
        self.get_logger().info(f"Subscribe odom: {odom_topic}")
        self.get_logger().info(f"Publish Pt: {pt_topic}")
        self.get_logger().info(
            f"target=({self.target_x:.3f}, {self.target_y:.3f}), "
            f"sigma={self.sigma:.3f}, active_radius={self.active_radius:.3f}"
        )

    def odom_callback(self, msg):
        raw_x = float(msg.pose.pose.position.x)
        raw_y = float(msg.pose.pose.position.y)

        if not self.have_origin:
            self.origin_x = raw_x
            self.origin_y = raw_y
            self.have_origin = True
            self.get_logger().info(
                f"Odom origin set: ({self.origin_x:.3f}, {self.origin_y:.3f})"
            )

        x = raw_x - self.origin_x
        y = raw_y - self.origin_y

        dx = x - self.target_x
        dy = y - self.target_y
        dist = math.sqrt(dx * dx + dy * dy)

        if dist <= self.active_radius:
            pt = math.exp(-(dist * dist) / (2.0 * self.sigma * self.sigma))
        else:
            pt = 0.0

        msg_pt = Float32()
        msg_pt.data = float(pt)
        self.pub_pt.publish(msg_pt)

        self.get_logger().info(
            f"odom_rel=({x:.3f}, {y:.3f}), "
            f"target=({self.target_x:.3f}, {self.target_y:.3f}), "
            f"dist={dist:.3f}, pt={pt:.3f}"
        )


def main():
    rclpy.init()
    node = VirtualPtFromOdomNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
