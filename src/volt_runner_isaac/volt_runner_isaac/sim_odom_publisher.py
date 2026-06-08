#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
import tf2_ros


class SimOdomPublisher(Node):
    def __init__(self):
        super().__init__('sim_odom_publisher')

        # Pi odom_publisher 파라미터를 참고하되, 시뮬용이라 보정계수는 1.0으로 시작
        self.declare_parameter('wheel_radius', 0.0485)
        self.declare_parameter('wheel_separation_width', 0.17)
        self.declare_parameter('wheel_separation_length', 0.20)
        self.declare_parameter('correction_factor_linear', 1.0)
        self.declare_parameter('correction_factor_angular', 1.0)
        self.declare_parameter('publish_tf', True)

        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.wheel_separation_width = float(self.get_parameter('wheel_separation_width').value)
        self.wheel_separation_length = float(self.get_parameter('wheel_separation_length').value)
        self.correction_factor_linear = float(self.get_parameter('correction_factor_linear').value)
        self.correction_factor_angular = float(self.get_parameter('correction_factor_angular').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)

        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/wheel_feedback_rpm',
            self.feedback_callback,
            10
        )

        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = self.get_clock().now()

        self.get_logger().info('sim_odom_publisher started')

    def feedback_callback(self, msg: Float32MultiArray):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        rpm = list(msg.data)

        # 현재 adapter_feedback 출력 순서: [FL, FR, RR, RL]
        if len(rpm) < 4:
            return

        rpm_fl = rpm[0]
        rpm_fr = rpm[1]
        rpm_rr = rpm[2]
        rpm_rl = rpm[3]

        # RPM -> wheel tangential velocity (m/s)
        v_fl = (rpm_fl * 2.0 * math.pi * self.wheel_radius) / 60.0
        v_fr = (rpm_fr * 2.0 * math.pi * self.wheel_radius) / 60.0
        v_rl = (rpm_rl * 2.0 * math.pi * self.wheel_radius) / 60.0
        v_rr = (rpm_rr * 2.0 * math.pi * self.wheel_radius) / 60.0

        k = self.wheel_separation_width + self.wheel_separation_length

        # Mecanum forward kinematics
        linear_x = (v_fl + v_fr + v_rl + v_rr) / 4.0
        linear_y = (-v_fl + v_fr + v_rl - v_rr) / 4.0
        angular_z = -(-v_fl + v_fr - v_rl + v_rr) / (4.0 * k)

        linear_x *= self.correction_factor_linear
        linear_y *= self.correction_factor_linear
        angular_z *= self.correction_factor_angular

        delta_x = (linear_x * math.cos(self.th) - linear_y * math.sin(self.th)) * dt
        delta_y = (linear_x * math.sin(self.th) + linear_y * math.cos(self.th)) * dt
        delta_th = angular_z * dt

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        q = self.euler_to_quaternion(0.0, 0.0, self.th)

        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = q

        odom.twist.twist.linear.x = linear_x
        odom.twist.twist.linear.y = linear_y
        odom.twist.twist.angular.z = angular_z

        self.odom_publisher.publish(odom)

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = current_time.to_msg()
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation = q
            self.tf_broadcaster.sendTransform(t)

    @staticmethod
    def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
        qx = math.sin(roll / 2.0) * math.cos(pitch / 2.0) * math.cos(yaw / 2.0) - math.cos(roll / 2.0) * math.sin(pitch / 2.0) * math.sin(yaw / 2.0)
        qy = math.cos(roll / 2.0) * math.sin(pitch / 2.0) * math.cos(yaw / 2.0) + math.sin(roll / 2.0) * math.cos(pitch / 2.0) * math.sin(yaw / 2.0)
        qz = math.cos(roll / 2.0) * math.cos(pitch / 2.0) * math.sin(yaw / 2.0) - math.sin(roll / 2.0) * math.sin(pitch / 2.0) * math.cos(yaw / 2.0)
        qw = math.cos(roll / 2.0) * math.cos(pitch / 2.0) * math.cos(yaw / 2.0) + math.sin(roll / 2.0) * math.sin(pitch / 2.0) * math.sin(yaw / 2.0)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)


def main(args=None):
    rclpy.init(args=args)
    node = SimOdomPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
