#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, String
from geometry_msgs.msg import Twist


class RasterRlManagerNode(Node):
    def __init__(self):
        super().__init__("raster_rl_manager_node")

        self.declare_parameter("pt_topic", "/coil_efficiency")
        self.declare_parameter("raster_cmd_topic", "/cmd_vel_raster")
        self.declare_parameter("rl_cmd_topic", "/cmd_vel_rl")
        self.declare_parameter("cmd_topic", "/cmd_vel")
        self.declare_parameter("mode_topic", "/align_mode")

        self.declare_parameter("pt_switch_on", 0.15)
        self.declare_parameter("pt_success", 0.90)
        self.declare_parameter("success_hold_steps", 3)
        self.declare_parameter("lock_rl_once_detected", True)

        self.declare_parameter("control_hz", 10.0)

        pt_topic = self.get_parameter("pt_topic").value
        raster_cmd_topic = self.get_parameter("raster_cmd_topic").value
        rl_cmd_topic = self.get_parameter("rl_cmd_topic").value
        cmd_topic = self.get_parameter("cmd_topic").value
        mode_topic = self.get_parameter("mode_topic").value

        self.pt_switch_on = float(self.get_parameter("pt_switch_on").value)
        self.pt_success = float(self.get_parameter("pt_success").value)
        self.success_hold_steps = int(self.get_parameter("success_hold_steps").value)
        self.lock_rl_once_detected = bool(self.get_parameter("lock_rl_once_detected").value)

        self.latest_pt = 0.0
        self.latest_raster_cmd = Twist()
        self.latest_rl_cmd = Twist()

        self.mode = "RASTER"
        self.rl_locked = False
        self.success_count = 0

        self.sub_pt = self.create_subscription(Float32, pt_topic, self.pt_callback, 10)
        self.sub_raster = self.create_subscription(Twist, raster_cmd_topic, self.raster_callback, 10)
        self.sub_rl = self.create_subscription(Twist, rl_cmd_topic, self.rl_callback, 10)

        self.pub_cmd = self.create_publisher(Twist, cmd_topic, 10)
        self.pub_mode = self.create_publisher(String, mode_topic, 10)

        period = 1.0 / float(self.get_parameter("control_hz").value)
        self.timer = self.create_timer(period, self.loop)

        self.get_logger().info("Raster-RL manager started")
        self.get_logger().info(
            f"pt_switch_on={self.pt_switch_on}, pt_success={self.pt_success}, "
            f"success_hold_steps={self.success_hold_steps}, "
            f"lock_rl_once_detected={self.lock_rl_once_detected}"
        )

    def pt_callback(self, msg):
        self.latest_pt = float(msg.data)

    def raster_callback(self, msg):
        self.latest_raster_cmd = msg

    def rl_callback(self, msg):
        self.latest_rl_cmd = msg

    def loop(self):
        if self.latest_pt >= self.pt_success:
            self.success_count += 1
        else:
            self.success_count = 0

        if self.success_count >= self.success_hold_steps:
            self.mode = "STOP"
            out = Twist()

        else:
            if self.latest_pt >= self.pt_switch_on:
                self.rl_locked = True

            if self.rl_locked and self.lock_rl_once_detected:
                self.mode = "RL"
                out = self.latest_rl_cmd
            else:
                self.mode = "RASTER"
                out = self.latest_raster_cmd

        self.pub_cmd.publish(out)

        mode_msg = String()
        mode_msg.data = self.mode
        self.pub_mode.publish(mode_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RasterRlManagerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
