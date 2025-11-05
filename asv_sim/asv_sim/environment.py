#!/usr/bin/env python3

# Roland Arsenault
# Center for Coastal and Ocean Mapping
# University of New Hampshire
# Copyright 2017, All rights reserved.

import rclpy.node
from rcl_interfaces.msg import Parameter
from rcl_interfaces.msg import ParameterDescriptor
import random

from typing import List


class Environment(object):
    def __init__(self, node: rclpy.node.Node):
        node.add_post_set_parameters_callback(self.updateParameters)

        node.declare_parameter('environment.current.speed', 1.0, ParameterDescriptor(description = 'Speed of current, in m/s'))
        node.declare_parameter('environment.current.direction', 90.0, ParameterDescriptor(description = 'Direction current is flowing, in degrees'))


        node.declare_parameter('environment.current.noise.speed', 0.1, ParameterDescriptor(description = 'Jitter for current speed, gauss(speed*jitter)'))
        node.declare_parameter('environment.current.noise.direction', 0.25, ParameterDescriptor(description = 'Jitter added to current direction, gauss(jitter)'))

    def updateParameters(self, parameters: List[Parameter]):
        for param in parameters:
            if param.name == 'environment.current.speed':
                self.current_speed = param.value
            if param.name == 'environment.current.direction':
                self.current_direction = param.value

            if param.name == 'environment.current.noise.speed':
                self.current_speed_noise = param.value
            if param.name == 'environment.current.noise.direction':
                self.current_direction_noise = param.value

    def getCurrent(self, lat, long, apply_noise: bool):
        # plan to allow current to differ with location, uniform for now.
    
        current_speed = self.current_speed
        current_direction = self.current_direction

        if apply_noise:
            current_speed = random.gauss(current_speed, current_speed*self.current_speed_noise)
            current_direction += random.gauss(0.0, self.current_direction_noise)
        
        return {'speed': current_speed, 'direction':current_direction}
    
