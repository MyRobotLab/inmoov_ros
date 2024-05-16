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

class JointStatusDispatcher(Node):

    def __init__(self):
        super().__init__('joint_status_dispatcher')
        self.servos = load_config_from_param(self)
        self.lookup = {}
        self.joints = {}
        self.bus = {}

        for n, s in self.servos.items():
            key = ((int(s.bus) * 255) + int(s.servoPin))
            self.lookup[key] = n
            print(f'key: {key}')

        self.publisher = self.create_publisher(JointState, "joint_status", 10)

        joints_param = self.get_parameter_or('joints', {}).get_parameter_value().string_value
        for j, b in joints_param.items():
            number = self.get_parameter_or(f'joints/{j}/bus', 0).get_parameter_value().integer_value
            busname = f'/servobus/{str(number).zfill(2)}/motorstatus'

            if number not in self.bus:
                self.bus[number] = self.create_subscription(
                    MotorStatus, busname, lambda msg, num=number: self.dispatcher(msg, num), 10)
                self.get_logger().info(f'adding: {busname}')

        self.timer = self.create_timer(0.025, self.publish_joint_status)

    def dispatcher(self, data, bus):
        try:
            key = self.lookup[((int(bus) * 255) + int(data.id))]
            self.joints[key] = data.position
        except KeyError:
            self.get_logger().warn(f'joint_status_dispatcher: unknown servo at bus:{bus} servo:{data.id}')

    def publish_joint_status(self):
        jointstatus = JointState()
        jointstatus.header = Header()
        jointstatus.header.stamp = self.get_clock().now().to_msg()
        jointstatus.name = []
        jointstatus.position = []
        jointstatus.velocity = []
        jointstatus.effort = []

        for j, p in self.joints.items():
            jointstatus.name.append(j)
            jointstatus.position.append(p)

        if len(jointstatus.name) > 0:
            self.publisher.publish(jointstatus)

        self.joints.clear()

def main(args=None):
    rclpy.init(args=args)
    joint_status_dispatcher = JointStatusDispatcher()
    try:
        rclpy.spin(joint_status_dispatcher)
    except KeyboardInterrupt:
        pass
    finally:
        joint_status_dispatcher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
