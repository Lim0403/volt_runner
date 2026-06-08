#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class VirtualPtWorldNode(Node):
    def __init__(self):
        super().__init__("virtual_pt_world_node")

        self.declare_parameter("cmd_topic", "/cmd_vel")
        self.declare_parameter("pt_topic", "/coil_efficiency")
        self.declare_parameter("odom_topic", "/odom")

        # raster가 약 10초 뒤에 Pt 영역에 들어오도록 둔 가상 코일 위치
        self.declare_parameter("target_x", 0.09)
        self.declare_parameter("target_y", 0.00)

        # Pt 분포 폭
        self.declare_parameter("sigma", 0.02)

        self.declare_parameter("dt", 0.1)

        self.cmd_topic = self.get_parameter("cmd_topic").value
        self.pt_topic = self.get_parameter("pt_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value

        self.target_x = float(self.get_parameter("target_x").value)
        self.target_y = float(self.get_parameter("target_y").value)
        self.sigma = float(self.get_parameter("sigma").value)
        self.dt = float(self.get_parameter("dt").value)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0

        self.sub_cmd = self.create_subscription(
            Twist,
            self.cmd_topic,
            self.cmd_callback,
            10,
        )

        self.pub_pt = self.create_publisher(Float32, self.pt_topic, 10)
        self.pub_odom = self.create_publisher(Odometry, self.odom_topic, 10)

        self.timer = self.create_timer(self.dt, self.loop)

        self.get_logger().info("Virtual Pt world started")
        self.get_logger().info(
            f"target=({self.target_x:.3f}, {self.target_y:.3f}), sigma={self.sigma:.3f}"
        )

    def cmd_callback(self, msg):
        self.vx = float(msg.linear.x)
        self.vy = float(msg.linear.y)
        self.wz = float(msg.angular.z)

    def loop(self):
        # /cmd_vel is interpreted as body-frame velocity.
        # Convert body-frame velocity to world-frame velocity.
        world_vx = math.cos(self.yaw) * self.vx - math.sin(self.yaw) * self.vy
        world_vy = math.sin(self.yaw) * self.vx + math.cos(self.yaw) * self.vy

        self.x += world_vx * self.dt
        self.y += world_vy * self.dt
        self.yaw += self.wz * self.dt

        dx = self.x - self.target_x
        dy = self.y - self.target_y
        d2 = dx * dx + dy * dy

        pt = math.exp(-d2 / (2.0 * self.sigma * self.sigma))

        pt_msg = Float32()
        pt_msg.data = float(pt)
        self.pub_pt.publish(pt_msg)

        odom = Odometry()
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.wz
        self.pub_odom.publish(odom)

        self.get_logger().info(
            f"x={self.x:.3f}, y={self.y:.3f}, pt={pt:.3f}, "
            f"vx={self.vx:.3f}, vy={self.vy:.3f}, wz={self.wz:.3f}"
        )


def main():
    rclpy.init()
    node = VirtualPtWorldNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
