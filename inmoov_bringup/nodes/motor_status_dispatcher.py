#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from inmoov_msgs.msg import MotorStatus

class MotorStatusDispatcher(Node):

    def __init__(self):
        super().__init__('motor_status_dispatcher')
        self.subscription = self.create_subscription(
            MotorStatus,
            'motor_status',
            self.motor_status_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def motor_status_callback(self, msg):
        self.get_logger().info('Received motor status: %s' % msg)

def main(args=None):
    rclpy.init(args=args)
    motor_status_dispatcher = MotorStatusDispatcher()
    try:
        rclpy.spin(motor_status_dispatcher)
    except KeyboardInterrupt:
        pass
    finally:
        motor_status_dispatcher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
