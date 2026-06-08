#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState


class AdapterCmd(Node):
    def __init__(self):
        super().__init__('adapter_cmd')

        self.sub = self.create_subscription(
            Float32MultiArray,
            '/wheel_target_rpm',
            self.cb_rpm,
            10
        )

        self.pub = self.create_publisher(
            JointState,
            '/joint_command',
            10
        )

        # 현재 네 Isaac용 조인트 이름 기준
        self.joint_names = [
            'front_left_wheel_joint',
            'front_right_wheel_joint',
            'rear_left_wheel_joint',
            'rear_right_wheel_joint',
        ]

        self.get_logger().info('adapter_cmd started')

    def cb_rpm(self, msg: Float32MultiArray):
        data = list(msg.data)

        if len(data) != 4:
            self.get_logger().warn(f'/wheel_target_rpm length != 4: {data}')
            return

        # 입력 순서: [FL, FR, RL, RR]
        vel_rad_s = [x * 2.0 * math.pi / 60.0 for x in data]

        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name = self.joint_names
        out.velocity = vel_rad_s

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = AdapterCmd()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
