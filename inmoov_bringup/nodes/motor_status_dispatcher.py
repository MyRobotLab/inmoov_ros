#!/usr/bin/env python3
# licensed under BSD-3

import rclpy
from rclpy.node import Node
from inmoov_msgs.msg import MotorStatus
from std_msgs.msg import Header
import os
from os.path import dirname, abspath
import sys

# Hacky way to add include directory to sys path
sys.path.append(os.path.join(dirname(dirname(dirname(abspath(__file__)))), 'include'))

from constants import PROTOCOL
from servos import Servo
from load_config_from_param import load_config_from_param

class MotorStatusDispatcher(Node):
    def __init__(self):
        super().__init__('motor_status_dispatcher')
        self.servos = load_config_from_param(self)

        self.lookup = {}
        self.joints = {}
        self.bus = {}

        # Load lookup name by (bus*255+servo id)
        for n, s in self.servos.items():
            try:
                key = (int(s.bus) * 255) + int(s.servoPin)
                self.lookup[key] = n
                self.get_logger().info(f'key: {str(key)}')
            except ValueError:
                self.get_logger().warn(f'motor_status_dispatcher: unknown servo at bus: {str(s.bus)} servo: {str(s.servoPin)}')

        self.publisher = self.create_publisher(MotorStatus, "motor_status", 10)

        for j, b in self.get_parameter('/joints').get_parameter_value().items():
            # Create motorstatus bus name
            number = self.get_parameter(f'/joints/{j}/bus').get_parameter_value().integer_value
            busname = f'/servobus/{str(number).zfill(2)}/motorstatus'

            if number not in self.bus:
                self.bus[number] = self.create_subscription(MotorStatus, busname, lambda msg, bus=number: self.dispatcher(msg, bus), 10)
                self.get_logger().info(f'adding: {busname}')

    def dispatcher(self, data, bus):
        try:
            jointname = self.lookup[(int(bus) * 255) + int(data.id)]
            data.joint = jointname
            data.bus = bus
            self.publisher.publish(data)
        except KeyError:
            self.get_logger().warn(f'motor_status_dispatcher: unknown servo at bus: {str(bus)} servo: {str(data.id)}')

def main(args=None):
    rclpy.init(args=args)
    node = MotorStatusDispatcher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
