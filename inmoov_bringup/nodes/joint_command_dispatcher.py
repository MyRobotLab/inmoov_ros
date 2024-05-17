#!/usr/bin/env python3
# licensed under BSD-3

import rclpy
from rclpy.node import Node

from inmoov_msgs.msg import MotorStatus, MotorCommand
from inmoov_msgs.srv import MotorParameter
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

from constants import PROTOCOL
from servos import Servo
from load_config_from_param import load_config_from_param

class JointCommandDispatcher(Node):

    def __init__(self):
        super().__init__('joint_command_dispatcher')
        self.servos = load_config_from_param(self)
        self.joints = {}  # dict of joint names and position values
        self.bus = {}  # dict of motorcommand busses indexed by ordinal

        self.subscription = self.create_subscription(
            JointState,
            'joint_command',
            self.dispatcher,
            10
        )

        joints_param = self.get_parameter_or('joints', {}).get_parameter_value().string_value
        for j, b in joints_param.items():
            number = self.get_parameter_or(f'joints/{j}/bus', 0).get_parameter_value().integer_value
            busname = f'/servobus/{str(number).zfill(2)}/motorcommand'

            if number not in self.bus:
                self.bus[number] = self.create_publisher(MotorCommand, busname, 10)
                self.get_logger().info(f'adding: {busname}')

        self.timer = self.create_timer(0.05, self.publish_motor_commands)

    def dispatcher(self, js):
        for x in range(len(js.name)):
            self.joints[js.name[x]] = js.position[x]
            print("YATZEE")

    def publish_motor_commands(self):
        for j, p in self.joints.items():
            try:
                motorcommand = MotorCommand()
                motorcommand.id = int(self.servos[j].servoPin)
                motorcommand.parameter = PROTOCOL.GOAL
                motorcommand.value = p

                self.bus[self.servos[j].bus].publish(motorcommand)
            except KeyError:
                self.get_logger().warn(f'joint_command_dispatcher: unknown joint: {j}')
        
        self.joints.clear()

def main(args=None):
    rclpy.init(args=args)
    joint_command_dispatcher = JointCommandDispatcher()
    try:
        rclpy.spin(joint_command_dispatcher)
    except KeyboardInterrupt:
        pass
    finally:
        joint_command_dispatcher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
