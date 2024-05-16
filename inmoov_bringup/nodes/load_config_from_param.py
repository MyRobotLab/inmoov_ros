#!/usr/bin/env python3
# licensed under BSD-3

import sys
import rclpy
from rclpy.node import Node
import yaml
import os
from os.path import dirname, abspath

from constants import PROTOCOL
from servos import Servo

servos = {}

def load_config_from_param(node: Node):

    # first, make sure parameter server is even loaded
    while not node.has_parameter("/joints"):
        node.get_logger().info("waiting for parameter server to load with joint definitions")
        node.create_rate(1).sleep()

    node.create_rate(1).sleep()

    joints = node.get_parameter("/joints").get_parameter_value().string_value
    for name in joints:
        node.get_logger().info("found: " + name)

        s = Servo()

        key = '/joints/' + name + '/'

        s.bus       = node.get_parameter(key + 'bus').get_parameter_value().integer_value
        s.servoPin  = node.get_parameter(key + 'servoPin').get_parameter_value().integer_value
        s.minPulse  = node.get_parameter(key + 'minPulse').get_parameter_value().integer_value
        s.maxPulse  = node.get_parameter(key + 'maxPulse').get_parameter_value().integer_value
        s.minGoal   = node.get_parameter(key + 'minGoal').get_parameter_value().integer_value
        s.maxGoal   = node.get_parameter(key + 'maxGoal').get_parameter_value().integer_value
        s.rest      = node.get_parameter(key + 'rest').get_parameter_value().integer_value
        s.maxSpeed  = node.get_parameter(key + 'maxSpeed').get_parameter_value().integer_value
        s.smoothing = node.get_parameter(key + 'smoothing').get_parameter_value().integer_value

        s.sensorpin = node.get_parameter(key + 'sensorPin').get_parameter_value().integer_value
        s.minSensor = node.get_parameter(key + 'minSensor').get_parameter_value().integer_value
        s.maxSensor = node.get_parameter(key + 'maxSensor').get_parameter_value().integer_value

        servos[name] = s

    return servos
