#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, TwistStamped

class CmdVelRelayNode(Node):
    def __init__(self):
        super().__init__('cmd_vel_relay')

        # Create subscriber to cmd_vel_stamped (TwistStamped)
        self.subscription = self.create_subscription(
            TwistStamped,
            'cmd_vel_stamped',
            self.cmd_vel_stamped_callback,
            10
        )

        # Create publisher to cmd_vel (Twist)
        self.publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        self.get_logger().info('cmd_vel_relay node has been started.')

    def cmd_vel_stamped_callback(self, msg: TwistStamped):
        # Extract and publish only the Twist message
        twist_msg = msg.twist
        self.publisher.publish(twist_msg)
        self.get_logger().debug('Published Twist from TwistStamped.')

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelRelayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
