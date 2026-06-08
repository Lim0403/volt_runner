#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray


class AdapterFeedback(Node):
    def __init__(self):
        super().__init__('adapter_feedback')

        self.sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.cb_joint_states,
            10
        )

        self.pub = self.create_publisher(
            Float32MultiArray,
            '/wheel_feedback_rpm',
            10
        )

        self.fl_joint = 'front_left_wheel_joint'
        self.fr_joint = 'front_right_wheel_joint'
        self.rl_joint = 'rear_left_wheel_joint'
        self.rr_joint = 'rear_right_wheel_joint'

        self.get_logger().info('adapter_feedback started')

    def cb_joint_states(self, msg: JointState):
        if not msg.name or not msg.velocity:
            return

        name_to_vel = {}
        for i, name in enumerate(msg.name):
            if i < len(msg.velocity):
                name_to_vel[name] = msg.velocity[i]

        required = [self.fl_joint, self.fr_joint, self.rl_joint, self.rr_joint]
        missing = [j for j in required if j not in name_to_vel]
        if missing:
            self.get_logger().warn(f'missing joints in /joint_states: {missing}')
            return

        # rad/s -> RPM
        fl_rpm = name_to_vel[self.fl_joint] * 60.0 / (2.0 * math.pi)
        fr_rpm = name_to_vel[self.fr_joint] * 60.0 / (2.0 * math.pi)
        rl_rpm = name_to_vel[self.rl_joint] * 60.0 / (2.0 * math.pi)
        rr_rpm = name_to_vel[self.rr_joint] * 60.0 / (2.0 * math.pi)

        out = Float32MultiArray()

        # 기존 odom 체인 호환용 순서: [FL, FR, RR, RL]
        out.data = [fl_rpm, fr_rpm, rr_rpm, rl_rpm]

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = AdapterFeedback()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
