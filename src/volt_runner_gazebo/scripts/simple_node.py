import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('simple_node')
        self.get_logger().info('My Simple Node has started!')

def main(args=None):
    rclpy.init(args=args)
    
    my_node = MyNode()
    
    try:
        rclpy.spin(my_node)
    except KeyboardInterrupt:
        pass
    finally:
        my_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()