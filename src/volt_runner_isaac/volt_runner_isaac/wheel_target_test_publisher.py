#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class WheelTargetTestPublisher(Node):
    """
    Publish /wheel_target_rpm in [FL, FR, RL, RR] order.

    Modes:
      - stop
      - forward
      - backward
      - turn_left
      - turn_right
      - strafe_left
      - strafe_right
    """

    def __init__(self):
        super().__init__('wheel_target_test_publisher')

        self.declare_parameter('mode', 'forward')
        self.declare_parameter('rpm', 120.0)
        self.declare_parameter('hz', 10.0)

        self.pub = self.create_publisher(Float32MultiArray, '/wheel_target_rpm', 10)

        hz = float(self.get_parameter('hz').value)
        period = 1.0 / hz if hz > 0.0 else 0.1
        self.timer = self.create_timer(period, self.timer_callback)

        self.get_logger().info('wheel_target_test_publisher started')

    def timer_callback(self):
        mode = str(self.get_parameter('mode').value)
        rpm = float(self.get_parameter('rpm').value)

        # output order: [FL, FR, RL, RR]
        if mode == 'forward':
            data = [ rpm,  rpm,  rpm,  rpm]
        elif mode == 'backward':
            data = [-rpm, -rpm, -rpm, -rpm]
        elif mode == 'turn_left':
            data = [-rpm,  rpm, -rpm,  rpm]
        elif mode == 'turn_right':
            data = [ rpm, -rpm,  rpm, -rpm]
        elif mode == 'strafe_left':
            data = [-rpm,  rpm,  rpm, -rpm]
        elif mode == 'strafe_right':
            data = [ rpm, -rpm, -rpm,  rpm]
        else:
            data = [0.0, 0.0, 0.0, 0.0]

        msg = Float32MultiArray()
        msg.data = data
        self.pub.publish(msg)

    def destroy_node(self):
        self.get_logger().info('wheel_target_test_publisher stopping')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WheelTargetTestPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()