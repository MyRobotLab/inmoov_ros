#!/usr/bin/env python3
# licensed under BSD-3

import rclpy
from rclpy.node import Node

from inmoov_msgs.msg import MotorStatus, MotorCommand
from inmoov_msgs.srv import MotorParameter
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

import os
import sys
from os.path import dirname, abspath

from PyQt5 import QtWidgets, QtCore, uic
from PyQt5.QtWidgets import QPushButton, QMessageBox

from threading import Thread

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget

# Hacky way to add include directory to sys path
sys.path.append(os.path.join(dirname(dirname(dirname(abspath(__file__)))),'include'))

from constants import PROTOCOL
from servos import Servo
from load_config_from_param import load_config_from_param

PI = 3.1415926539

# https://github.com/ColinDuquesnoy/QDarkStyleSheet
# import qdarkstyle

# https://www.safaribooksonline.com/blog/2014/01/22/create-basic-gui-using-pyqt/
gui = os.path.join(os.path.dirname(__file__), 'rviz_manager.ui')
form_class = uic.loadUiType(gui)[0]

class RVizManager(Node):
    def __init__(self):
        super().__init__('rviz_command_dispatcher')

        # Subscribers
        self.subscriber1 = self.create_subscription(
            JointState,
            'joint_command',
            self.command_dispatcher,
            10
        )
        self.subscriber2 = self.create_subscription(
            JointState,
            'joint_state',
            self.state_dispatcher,
            10
        )

        # Publisher
        self.publisher = self.create_publisher(JointState, 'rviz_command', 10)

        load_config_from_param()

    def command_dispatcher(self, data):
        if self.get_parameter('bringup.angles').get_parameter_value().string_value == 'degrees':
            p = list(data.position)
            for x in range(len(p)):
                p[x] = PI * p[x] / 180.0
            data.position = tuple(p)
        self.publisher.publish(data)

    def state_dispatcher(self, data):
        if self.get_parameter('bringup.angles').get_parameter_value().string_value == 'degrees':
            p = list(data.position)
            for x in range(len(p)):
                p[x] = PI * p[x] / 180.0
            data.position = tuple(p)
        self.publisher.publish(data)


def main(args=None):
    rclpy.init(args=args)
    node = RVizManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
