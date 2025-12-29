import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelMonitor(Node):
    def __init__(self):
        super().__init__('cmd_vel_monitor')
        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.callback,
            10
        )

    def callback(self, msg):
        self.get_logger().info(
            f'Linear: {msg.linear.x:.2f}, Angular: {msg.angular.z:.2f}'
        )

def main():
    rclpy.init()
    node = CmdVelMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
