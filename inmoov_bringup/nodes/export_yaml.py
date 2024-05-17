#!/usr/bin/env python3
# licensed as BSD-3

import sys
import rclpy
from rclpy.node import Node
import yaml
import os
from os.path import dirname, abspath

from constants import PROTOCOL
from servos import Servo

def export_yaml(node: Node, filename: str):
    outfile = os.path.join(dirname(dirname(abspath(__file__))), 'config', filename)
    print(outfile)

    with open(outfile, 'w') as export:
        export.write('bringup:\n')
        export.write(str('  angles:').ljust(20) + node.get_parameter('/bringup/angles').get_parameter_value().string_value + '\n')
        export.write(str('  hz:').ljust(20) + str(node.get_parameter('/bringup/hz').get_parameter_value().integer_value) + '\n')
        export.write('\n')
        export.write('joints:\n')
        export.write('\n')

        joints_param = node.get_parameter('/joints').get_parameter_value().string_value
        for name in joints_param:
            print("updating yaml for:  " + name)
            key = '/joints/' + name + '/'

            export.write(str('  ' + name + ':').ljust(20) + '\n')
            val = node.get_parameter(key + 'bus').get_parameter_value().integer_value
            export.write(str('    bus:').ljust(20) + str(int(val)) + '\n')
            val = node.get_parameter(key + 'servoPin').get_parameter_value().integer_value
            export.write(str('    servoPin:').ljust(20) + str(int(val)) + '\n')
            val = node.get_parameter(key + 'minPulse').get_parameter_value().integer_value
            export.write(str('    minPulse:').ljust(20) + str(int(val)) + '\n')
            val = node.get_parameter(key + 'maxPulse').get_parameter_value().integer_value
            export.write(str('    maxPulse:').ljust(20) + str(int(val)) + '\n')
            val = node.get_parameter(key + 'minGoal').get_parameter_value().integer_value
            export.write(str('    minGoal:').ljust(20) + str(val) + '\n')
            val = node.get_parameter(key + 'maxGoal').get_parameter_value().integer_value
            export.write(str('    maxGoal:').ljust(20) + str(val) + '\n')
            val = node.get_parameter(key + 'rest').get_parameter_value().integer_value
            export.write(str('    rest:').ljust(20) + str(val) + '\n')
            val = node.get_parameter(key + 'maxSpeed').get_parameter_value().integer_value
            export.write(str('    maxSpeed:').ljust(20) + str(val) + '\n')
            val = node.get_parameter(key + 'smoothing').get_parameter_value().integer_value
            export.write(str('    smoothing:').ljust(20) + str(int(val)) + '\n')
            val = node.get_parameter(key + 'sensorPin').get_parameter_value().integer_value
            export.write(str('    sensorPin:').ljust(20) + str(int(val)) + '\n')
            val = node.get_parameter(key + 'minSensor').get_parameter_value().integer_value
            export.write(str('    minSensor:').ljust(20) + str(int(val)) + '\n')
            val = node.get_parameter(key + 'maxSensor').get_parameter_value().integer_value
            export.write(str('    maxSensor:').ljust(20) + str(int(val)) + '\n')

            export.write('\n')

    print("DONE")
