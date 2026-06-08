#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
import tf2_ros


def forward_kinematics(w_fl, w_fr, w_rl, w_rr, r, lx, ly):
    """
    Mecanum FK
    input order: [FL, FR, RL, RR] in rad/s
    returns: vx, vy, wz in base_link frame
    """
    l_sum = lx + ly
    vx = (r / 4.0) * (w_fl + w_fr + w_rl + w_rr)
    vy = (r / 4.0) * (-w_fl + w_fr + w_rl - w_rr)
    wz = (r / 4.0) * (-w_fl + w_fr - w_rl + w_rr) / l_sum
    return vx, vy, wz


def quaternion_from_yaw(yaw: float) -> Quaternion:
    half = yaw * 0.5
    return Quaternion(
        x=0.0,
        y=0.0,
        z=math.sin(half),
        w=math.cos(half),
    )


class SimFKNode(Node):
    def __init__(self):
        super().__init__('sim_fk_node')

        # ---- same base params as original FK ----
        self.declare_parameter('wheel_radius', 0.04)
        self.declare_parameter('lx', 0.10)
        self.declare_parameter('ly', 0.088)

        # ---- frame names ----
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        # ---- correction ----
        self.declare_parameter('correction_linear', 1.0)
        self.declare_parameter('correction_lateral', 1.0)
        self.declare_parameter('correction_angular', 1.0)

        # ---- tf ----
        self.declare_parameter('publish_tf', True)

        # ---- topics ----
        self.declare_parameter('input_topic', '/wheel_feedback_rpm')
        self.declare_parameter('output_topic', '/odom')

        self.r = float(self.get_parameter('wheel_radius').value)
        self.lx = float(self.get_parameter('lx').value)
        self.ly = float(self.get_parameter('ly').value)

        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)

        self.correction_linear = float(self.get_parameter('correction_linear').value)
        self.correction_lateral = float(self.get_parameter('correction_lateral').value)
        self.correction_angular = float(self.get_parameter('correction_angular').value)

        self.publish_tf = bool(self.get_parameter('publish_tf').value)

        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)

        self.subscription = self.create_subscription(
            Float32MultiArray, input_topic, self.feedback_callback, 10
        )
        self.odom_pub = self.create_publisher(Odometry, output_topic, 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = self.get_clock().now().nanoseconds / 1e9

        self.get_logger().info('sim_fk_node started')

    def feedback_callback(self, msg: Float32MultiArray):
        now = self.get_clock().now().nanoseconds / 1e9
        dt = now - self.last_time
        self.last_time = now

        rpm_raw = list(msg.data)
        if len(rpm_raw) != 4:
            self.get_logger().warn(f'/wheel_feedback_rpm length != 4: {rpm_raw}')
            return

        # incoming order from adapter_feedback: [FL, FR, RR, RL]
        rpm_fl = rpm_raw[0]
        rpm_fr = rpm_raw[1]
        rpm_rr = rpm_raw[2]
        rpm_rl = rpm_raw[3]

        # reorder for FK: [FL, FR, RL, RR]
        rpm_for_fk = [rpm_fl, rpm_fr, rpm_rl, rpm_rr]

        ws = [v * 2.0 * math.pi / 60.0 for v in rpm_for_fk]
        w_fl, w_fr, w_rl, w_rr = ws

        vx, vy, wz = forward_kinematics(w_fl, w_fr, w_rl, w_rr, self.r, self.lx, self.ly)

        vx *= self.correction_linear
        vy *= self.correction_lateral
        wz *= self.correction_angular

        dx = vx * math.cos(self.th) - vy * math.sin(self.th)
        dy = vx * math.sin(self.th) + vy * math.cos(self.th)

        self.x += dx * dt
        self.y += dy * dt
        self.th += wz * dt

        q = quaternion_from_yaw(self.th)

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = q

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz

        self.odom_pub.publish(odom)

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = odom.header.stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation = q
            self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = SimFKNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()