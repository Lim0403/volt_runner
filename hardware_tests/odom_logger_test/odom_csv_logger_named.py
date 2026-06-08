#!/usr/bin/env python3

import csv
import math
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray


def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class OdomCsvLogger(Node):
    def __init__(self, label):
        super().__init__("odom_csv_logger")

        log_dir = Path.home() / "robot_test_logs"
        log_dir.mkdir(parents=True, exist_ok=True)

        stamp = time.strftime("%Y%m%d_%H%M%S")
        self.filename = str(log_dir / f"{label}_test_{stamp}.csv")

        self.last_cmd = Twist()
        self.last_target = [0.0, 0.0, 0.0, 0.0]
        self.last_feedback = [0.0, 0.0, 0.0, 0.0]

        self.file = open(self.filename, "w", newline="")
        self.writer = csv.writer(self.file)

        self.writer.writerow([
            "time_sec",
            "x_m", "y_m", "yaw_rad", "yaw_deg",
            "vx_mps", "vy_mps", "wz_radps",
            "cmd_x", "cmd_y", "cmd_wz",
            "target_rr", "target_rl", "target_fr", "target_fl",
            "feedback_rr", "feedback_rl", "feedback_fr", "feedback_fl",
        ])

        self.create_subscription(Odometry, "/odom", self.odom_cb, 10)
        self.create_subscription(Twist, "/cmd_vel", self.cmd_cb, 10)
        self.create_subscription(Float32MultiArray, "/wheel_target_rpm", self.target_cb, 10)
        self.create_subscription(Float32MultiArray, "/wheel_feedback_rpm", self.feedback_cb, 10)

        self.get_logger().info(f"Saving CSV to: {self.filename}")

    def cmd_cb(self, msg):
        self.last_cmd = msg

    def target_cb(self, msg):
        if len(msg.data) >= 4:
            self.last_target = list(msg.data[:4])

    def feedback_cb(self, msg):
        if len(msg.data) >= 4:
            self.last_feedback = list(msg.data[:4])

    def odom_cb(self, msg):
        now = self.get_clock().now().nanoseconds / 1e9

        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        tw = msg.twist.twist
        yaw = yaw_from_quat(q)

        self.writer.writerow([
            f"{now:.6f}",
            f"{p.x:.6f}",
            f"{p.y:.6f}",
            f"{yaw:.6f}",
            f"{math.degrees(yaw):.3f}",
            f"{tw.linear.x:.6f}",
            f"{tw.linear.y:.6f}",
            f"{tw.angular.z:.6f}",
            f"{self.last_cmd.linear.x:.6f}",
            f"{self.last_cmd.linear.y:.6f}",
            f"{self.last_cmd.angular.z:.6f}",
            *[f"{v:.6f}" for v in self.last_target],
            *[f"{v:.6f}" for v in self.last_feedback],
        ])

        self.file.flush()


def main():
    label = sys.argv[1] if len(sys.argv) > 1 else "unnamed"

    rclpy.init()
    node = OdomCsvLogger(label)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.file.close()
    print(f"\nSaved CSV: {node.filename}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
