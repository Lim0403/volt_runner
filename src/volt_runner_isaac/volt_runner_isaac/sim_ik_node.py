#!/usr/bin/env python3

import math
import functools

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray


def inverse_kinematics(vx, vy, omega, r, lx, ly):
    """
    Mecanum IK
    return order: [FL, FR, RL, RR] in rad/s
    """
    l_sum = lx + ly
    w_fl = (1.0 / r) * (vx - vy - l_sum * omega)
    w_fr = (1.0 / r) * (vx + vy + l_sum * omega)
    w_rl = (1.0 / r) * (vx + vy - l_sum * omega)
    w_rr = (1.0 / r) * (vx - vy + l_sum * omega)
    return [w_fl, w_fr, w_rl, w_rr]


class SimIKNode(Node):
    def __init__(self):
        super().__init__('sim_ik_node')

        # ---- base params: original mecanum_ik_node reference ----
        self.declare_parameter('wheel_radius', 0.04)
        self.declare_parameter('lx', 0.10)
        self.declare_parameter('ly', 0.088)

        # ---- gains: original values as defaults ----
        self.declare_parameter('gain_x', 0.32)
        self.declare_parameter('gain_y', 0.87)
        self.declare_parameter('gain_z', 0.89)

        # ---- deadzone / clamp ----
        self.declare_parameter('deadzone_linear', 0.01)
        self.declare_parameter('deadzone_angular', 0.01)
        self.declare_parameter('enable_clamp', True)
        self.declare_parameter('min_linear_x', 0.3)
        self.declare_parameter('min_linear_y', 0.0)
        self.declare_parameter('min_angular_z', 0.3)

        # ---- limits for sim / RL ----
        self.declare_parameter('vx_max', 0.15)
        self.declare_parameter('vy_max', 0.10)
        self.declare_parameter('wz_max', 0.40)
        self.declare_parameter('wheel_rpm_max', 9999.0)

        # ---- sign conventions ----
        self.declare_parameter('flip_omega_sign', True)

        # ---- topics ----
        self.declare_parameter('cmd_topic_manual', '/cmd_vel')
        self.declare_parameter('cmd_topic_nav2', '/cmd_vel_nav2')
        self.declare_parameter('cmd_topic_rl', '/cmd_vel_rl')
        self.declare_parameter('output_topic', '/wheel_target_rpm')

        self.r = float(self.get_parameter('wheel_radius').value)
        self.lx = float(self.get_parameter('lx').value)
        self.ly = float(self.get_parameter('ly').value)

        self.gain_x = float(self.get_parameter('gain_x').value)
        self.gain_y = float(self.get_parameter('gain_y').value)
        self.gain_z = float(self.get_parameter('gain_z').value)

        self.deadzone_linear = float(self.get_parameter('deadzone_linear').value)
        self.deadzone_angular = float(self.get_parameter('deadzone_angular').value)
        self.enable_clamp = bool(self.get_parameter('enable_clamp').value)
        self.min_linear_x = float(self.get_parameter('min_linear_x').value)
        self.min_linear_y = float(self.get_parameter('min_linear_y').value)
        self.min_angular_z = float(self.get_parameter('min_angular_z').value)

        self.vx_max = float(self.get_parameter('vx_max').value)
        self.vy_max = float(self.get_parameter('vy_max').value)
        self.wz_max = float(self.get_parameter('wz_max').value)
        self.wheel_rpm_max = float(self.get_parameter('wheel_rpm_max').value)

        self.flip_omega_sign = bool(self.get_parameter('flip_omega_sign').value)

        topic_manual = str(self.get_parameter('cmd_topic_manual').value)
        topic_nav2 = str(self.get_parameter('cmd_topic_nav2').value)
        topic_rl = str(self.get_parameter('cmd_topic_rl').value)
        output_topic = str(self.get_parameter('output_topic').value)

        self.sub_manual = self.create_subscription(
            Twist, topic_manual, functools.partial(self.cmd_vel_callback, source='manual'), 10
        )
        self.sub_nav2 = self.create_subscription(
            Twist, topic_nav2, functools.partial(self.cmd_vel_callback, source='nav2'), 10
        )
        self.sub_rl = self.create_subscription(
            Twist, topic_rl, functools.partial(self.cmd_vel_callback, source='rl'), 10
        )

        self.publisher = self.create_publisher(Float32MultiArray, output_topic, 10)

        self.last_source = None
        self.get_logger().info(
            f"sim_ik_node started | gains X:{self.gain_x}, Y:{self.gain_y}, Z:{self.gain_z}"
        )

    def cmd_vel_callback(self, msg: Twist, source: str):
        self.last_source = source
        self.apply_cmd(msg)

    def apply_cmd(self, msg: Twist):
        raw_vx = float(msg.linear.x)
        raw_vy = float(msg.linear.y)
        raw_wz = float(msg.angular.z)

        # 1) deadzone
        vx = raw_vx if abs(raw_vx) > self.deadzone_linear else 0.0
        vy = raw_vy if abs(raw_vy) > self.deadzone_linear else 0.0
        wz = raw_wz if abs(raw_wz) > self.deadzone_angular else 0.0

        # original IK used omega sign flip
        if self.flip_omega_sign:
            wz = -wz

        # 2) clip by robot-level max limits
        vx = max(min(vx, self.vx_max), -self.vx_max)
        vy = max(min(vy, self.vy_max), -self.vy_max)
        wz = max(min(wz, self.wz_max), -self.wz_max)

        # 3) clamp minimum drive if enabled
        if self.enable_clamp:
            if abs(vx) > 0.0 and abs(vx) < self.min_linear_x:
                vx = math.copysign(self.min_linear_x, vx)
            if abs(vy) > 0.0 and abs(vy) < self.min_linear_y:
                vy = math.copysign(self.min_linear_y, vy)
            if abs(wz) > 0.0 and abs(wz) < self.min_angular_z:
                wz = math.copysign(self.min_angular_z, wz)

        # 4) apply tuned gains
        tuned_vx = vx * self.gain_x
        tuned_vy = vy * self.gain_y
        tuned_wz = wz * self.gain_z

        # 5) IK
        wheel_rad_s = inverse_kinematics(
            tuned_vx, tuned_vy, tuned_wz,
            self.r, self.lx, self.ly
        )

        # 6) rad/s -> rpm
        rpm_list = [w * 60.0 / (2.0 * math.pi) for w in wheel_rad_s]

        # 7) wheel rpm clip
        rpm_list = [
            max(min(rpm, self.wheel_rpm_max), -self.wheel_rpm_max)
            for rpm in rpm_list
        ]

        out = Float32MultiArray()
        out.data = rpm_list
        self.publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = SimIKNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()