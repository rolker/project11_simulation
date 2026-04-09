#!/usr/bin/env python3

# Roland Arsenault
# Center for Coastal and Ocean Mapping
# University of New Hampshire
# Copyright 2017, All rights reserved.

import math
import random
from typing import List

from rcl_interfaces.msg import Parameter
from rcl_interfaces.msg import ParameterDescriptor
import rclpy.node
import rclpy.time


# Portsmouth Harbor (NOAA Station 8423898) top-3 tidal constituents
_DEFAULT_TIDE_AMPLITUDES = [1.295, 0.289, 0.197]  # meters (M2, N2, S2)
_DEFAULT_TIDE_SPEEDS = [28.984, 28.440, 30.000]   # degrees/hour
_DEFAULT_TIDE_PHASES = [105.6, 73.4, 142.1]        # degrees GMT

# WGS84 ellipsoid to MLLW at Portsmouth Harbor from VDatum
_DEFAULT_ELLIPSOID_TO_MLLW = -28.104  # meters


class Environment(object):

    def __init__(self, node: rclpy.node.Node):
        node.add_post_set_parameters_callback(self.updateParameters)

        # Current parameters
        node.declare_parameter(
            'environment.current.speed', 1.0,
            ParameterDescriptor(
                description='Speed of current, in m/s'))
        node.declare_parameter(
            'environment.current.direction', 90.0,
            ParameterDescriptor(
                description='Direction current is flowing, in degrees'))

        node.declare_parameter(
            'environment.current.noise.speed', 0.1,
            ParameterDescriptor(
                description='Jitter for current speed, '
                'gauss(speed*jitter)'))
        node.declare_parameter(
            'environment.current.noise.direction', 0.25,
            ParameterDescriptor(
                description='Jitter added to current direction, '
                'gauss(jitter)'))

        # Tide parameters — parallel arrays defining harmonic constituents
        node.declare_parameter(
            'environment.tide.constituents.amplitudes',
            _DEFAULT_TIDE_AMPLITUDES,
            ParameterDescriptor(
                description='Tidal constituent amplitudes in meters'))
        node.declare_parameter(
            'environment.tide.constituents.speeds',
            _DEFAULT_TIDE_SPEEDS,
            ParameterDescriptor(
                description='Tidal constituent speeds in degrees/hour'))
        node.declare_parameter(
            'environment.tide.constituents.phases',
            _DEFAULT_TIDE_PHASES,
            ParameterDescriptor(
                description='Tidal constituent phases in degrees GMT'))
        node.declare_parameter(
            'environment.tide.ellipsoid_to_mllw',
            _DEFAULT_ELLIPSOID_TO_MLLW,
            ParameterDescriptor(
                description='Static offset from WGS84 ellipsoid to MLLW '
                'in meters (negative means MLLW is below ellipsoid)'))
        node.declare_parameter(
            'environment.tide.speed_factor', 1.0,
            ParameterDescriptor(
                description='Multiplier for constituent speeds — set >1 '
                'to accelerate tide for testing'))

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

            if param.name == 'environment.tide.constituents.amplitudes':
                self.tide_amplitudes = param.value
            if param.name == 'environment.tide.constituents.speeds':
                self.tide_speeds = param.value
            if param.name == 'environment.tide.constituents.phases':
                self.tide_phases = param.value
            if param.name == 'environment.tide.ellipsoid_to_mllw':
                self.ellipsoid_to_mllw = param.value
            if param.name == 'environment.tide.speed_factor':
                self.tide_speed_factor = param.value

    def getCurrent(self, lat, long, apply_noise: bool):
        # plan to allow current to differ with location, uniform for now.

        current_speed = self.current_speed
        current_direction = self.current_direction

        if apply_noise:
            current_speed = random.gauss(
                current_speed, current_speed * self.current_speed_noise)
            current_direction += random.gauss(
                0.0, self.current_direction_noise)

        return {'speed': current_speed, 'direction': current_direction}

    def getTide(self, timestamp: rclpy.time.Time) -> float:
        """Compute tide height above MLLW using harmonic constituents.

        :param timestamp: ROS time (used to derive hours since epoch)
        :returns: Tide height above MLLW in meters
        """
        nanoseconds = timestamp.nanoseconds
        hours = nanoseconds / 3.6e12

        tide = 0.0
        n = min(len(self.tide_amplitudes),
                len(self.tide_speeds),
                len(self.tide_phases))
        for i in range(n):
            speed_deg_per_hour = self.tide_speeds[i] * self.tide_speed_factor
            phase_rad = math.radians(self.tide_phases[i])
            speed_rad_per_hour = math.radians(speed_deg_per_hour)
            tide += self.tide_amplitudes[i] * math.cos(
                speed_rad_per_hour * hours - phase_rad)
        return tide

    def getEllipsoidalAltitude(self, timestamp: rclpy.time.Time) -> float:
        """Compute WGS84 ellipsoidal altitude for a surface vessel.

        :param timestamp: ROS time
        :returns: Ellipsoidal altitude in meters
        """
        return self.ellipsoid_to_mllw + self.getTide(timestamp)
