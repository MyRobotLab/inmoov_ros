#!/usr/bin/env python3
# licensed under BSD-3

import sys
import os
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from inmoov_msgs.msg import MotorStatus, MotorCommand
from inmoov_msgs.srv import MotorParameter
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

from os.path import dirname, abspath

from PyQt5 import QtWidgets, QtCore, uic
from PyQt5.QtWidgets import QWidget, QCheckBox, QApplication, QVBoxLayout

from python_qt_binding import loadUi

# Hacky way to add include directory to sys path
sys.path.append(os.path.join(dirname(dirname(dirname(abspath(__file__)))), 'include'))

from constants import PROTOCOL
from servos import Servo
from load_config_from_param import load_config_from_param

PI = 3.1415926539

# Set up the GUI path relative to the installed directory
gui = os.path.join(dirname(abspath(__file__)), 'rviz_manager.ui')
form_class = uic.loadUiType(gui)[0]

class RVizManager(Node, QtWidgets.QMainWindow, form_class):
    def __init__(self):
        rclpy.init(args=None)
        Node.__init__(self, 'rviz_command_dispatcher')
        QtWidgets.QMainWindow.__init__(self)
        self.setupUi(self)

        self.declare_parameter('bringup.angles', 'radians')

        self.servos = load_config_from_param(self)

        self.bus = {}
        self.checkboxes = {}

        self.motorcommand = MotorCommand()
        self.jointcommand = JointState()

        self.jointNames = []

        print("INITIALIZED")

        joints = self.get_parameter('joints').get_parameter_value().string_value

        for j, b in joints.items():
            number = self.get_parameter(f'joints/{j}/bus').get_parameter_value().integer_value
            busname = f'/servobus/{str(number).zfill(2)}/motorcommand'

            if number not in self.bus:
                self.bus[number] = self.create_publisher(MotorCommand, busname, 40)
                self.get_logger().info(f'adding: {busname}')

        self.jointPublisher = self.create_publisher(JointState, "joint_command", 40)
        self.statusSubscriber = self.create_subscription(MotorStatus, "motor_status", self.statusListener, 10)

        self.btnEnableAll.clicked.connect(self.setEnableAll)
        self.btnDisableAll.clicked.connect(self.setDisableAll)

        print("INIT COMPLETE")

        for name, s in self.servos.items():
            print(name)
            chk = QCheckBox(name)
            chk.setText(name)
            chk.setStyleSheet(checkboxstylesheet)
            chk.stateChanged.connect(self.checkChanged)
            self.layout.addWidget(chk)
            self.checkboxes[name] = chk

    def statusListener(self, s):
        j = s.joint
        chk = self.checkboxes[j]
        chk.blockSignals(True)
        chk.setChecked(s.enabled)
        chk.blockSignals(False)

    def checkChanged(self):
        sender = self.sender()
        print(sender.text())

        s = self.servos[sender.text()]
        chk = self.checkboxes[sender.text()]

        motorcommand = MotorCommand()
        motorcommand.id = int(s.servoPin)
        motorcommand.parameter = PROTOCOL.ENABLE
        motorcommand.value = sender.isChecked()

        print(f'checkChanged: {sender.text()} to: {str(sender.isChecked())}')

        self.bus[s.bus].publish(motorcommand)

    def setEnableAll(self):
        for j, s in self.servos.items():
            motorcommand = MotorCommand()
            motorcommand.id = int(s.servoPin)
            motorcommand.parameter = PROTOCOL.ENABLE
            motorcommand.value = 1
            self.bus[s.bus].publish(motorcommand)

    def setDisableAll(self):
        for j, s in self.servos.items():
            motorcommand = MotorCommand()
            motorcommand.id = int(s.servoPin)
            motorcommand.parameter = PROTOCOL.ENABLE
            motorcommand.value = 0
            print(j)
            self.bus[s.bus].publish(motorcommand)

    def closeEvent(self, event):
        self.enabled = False
        self.random = False
        print("GOODBYE!")


checkboxstylesheet = (
    'QCheckBox::indicator {'
    'width: 12px;'
    'height: 12px;'
    'border-style: outset;'
    'border-width: 1px;'
    'border-radius: 4px;'
    'border-color: rgb(125, 125, 125);'
    'border: 1px solid rgb(0,0,0);'
    'background-color: rgb(130, 7, 7);'
    '}'
    'QCheckBox::indicator:unchecked {background-color: rgb(130, 7, 7);}'
    'QCheckBox::indicator:checked {background-color: rgb(11, 145, 1);}'
)


def main():
    app = QtWidgets.QApplication(sys.argv)
    form = RVizManager()
    form.show()
    rclpy.spin(form)
    app.exec_()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
