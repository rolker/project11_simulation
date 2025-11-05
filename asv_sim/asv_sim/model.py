#!/usr/bin/env python3

import rclpy.node
from rcl_interfaces.msg import Parameter
from rcl_interfaces.msg import ParameterDescriptor
import math

from typing import List

class Model:
    def __init__(self, name: str, node: rclpy.node.Node):
        self.name = name
        self.parameters_ready = False

        node.add_post_set_parameters_callback(self.updateParameters)

        node.declare_parameter(self.ns('max_rpm'), 3200.0, ParameterDescriptor(description = 'The maximum engine speed achieved using full throttle'))
        node.declare_parameter(self.ns('max_power'), 21300.0, ParameterDescriptor(description = 'Power in watts produced at max_rpm'))
        node.declare_parameter(self.ns('idle_rpm'), 750.0, ParameterDescriptor(description = 'Lowest rpm reached when throttle is at 0'))
        node.declare_parameter(self.ns('clutch_engagement_rpm'), 0.0, ParameterDescriptor(description = 'RPM at which the clutch engages. Should be set to 0 if always engaged.'))
        node.declare_parameter(self.ns('gearbox_ratio'), 1.0, ParameterDescriptor(description = 'Ratio of prop rpm relative to engine rpm'))
        node.declare_parameter(self.ns('propulsion_type'), 'jet', ParameterDescriptor(description = '"jet" or "prop"'))
        node.declare_parameter(self.ns('prop_pitch'), 20.0, ParameterDescriptor(description='Inches traveled assuming no slip for each revolution'))
        node.declare_parameter(self.ns('bucket_efficiency'), 0.2, ParameterDescriptor( description= 'For jet drive with reverse bucket, how much reverse thrust relative to forward'))
        node.declare_parameter(self.ns('bucket_change_rate'), 0.5, ParameterDescriptor( description= 'How fast the bucket can move in full deflections per second.'))
        node.declare_parameter(self.ns('max_rpm_change_rate'), 1000.0, ParameterDescriptor(description='Represents engine inertia by limiting how much rpms can change per second.'))
        node.declare_parameter(self.ns('max_speed'), 2.75, ParameterDescriptor(description='Maximum speed of the boat in m/s.'))
        node.declare_parameter(self.ns('mass'), 2000.0, ParameterDescriptor(description='Mass of the boat in kilograms'))
        node.declare_parameter(self.ns('max_rudder_angle'), 33.0, ParameterDescriptor(description='Maximum deflection of the rudder in degrees'))
        node.declare_parameter(self.ns('max_rudder_change_rate'), 30.0, ParameterDescriptor(description='How fast in degrees per second the rudder can turn'))
        node.declare_parameter(self.ns('max_turn_rate'), 25.0, ParameterDescriptor(description='Maximum rate the boat can turn in degrees per second'))
        node.declare_parameter(self.ns('rudder_distance'), 2.0, ParameterDescriptor(description="Distance between the rudder and the boat's center of mass in meters"))
        node.declare_parameter(self.ns('rudder_coefficient'), 0.25, ParameterDescriptor(description='Expresses rudder efficiency'))

        node.declare_parameter(self.ns('thrust_noise'), 0.1, ParameterDescriptor(description='Noise applied to calculated thrust values, gauss(thrust*noise)'))
        node.declare_parameter(self.ns('yaw_rate_noise'), 0.1, ParameterDescriptor(description = 'Noise applied to calculated yaw rate, gauss(yaw_rate*noise)'))
        node.declare_parameter(self.ns('drag_noise'), 0.1, ParameterDescriptor(description='Noise applied to calculated drag, gauss(drag*noise)'))

        self.parameters_ready = True
        self.update_coefficients()

    def ns(self, key: str) -> str:
        return 'models.'+self.name+'.'+key

    def updateParameters(self, parameters: List[Parameter]):
        need_update = False
        for param in parameters:
            if param.name == self.ns('max_rpm'):
                self.max_rpm = param.value
                need_update = True
            if param.name == self.ns('max_power'):
                self.max_power = param.value
                need_update = True
            if param.name == self.ns('idle_rpm'):
                self.idle_rpm = param.value
            if param.name == self.ns('clutch_engagement_rpm'):
                self.clutch_engagement_rpm = param.value
                need_update = True
            if param.name == self.ns('gearbox_ratio'):
                self.gearbox_ratio = param.value
                need_update = True
            if param.name == self.ns('propulsion_type'):
                self.propulsion_type = param.value
                need_update = True
            if param.name == self.ns('prop_pitch'):
                self.prop_pitch = param.value
                need_update = True
            if param.name == self.ns('bucket_efficiency'):
                self.bucket_efficiency = param.value
                need_update = True
            if param.name == self.ns('bucket_change_rate'):
                self.max_bucket_change_rate = param.value
                need_update = True
            if param.name == self.ns('max_rpm_change_rate'):
                self.max_rpm_change_rate = param.value
                need_update = True
            if param.name == self.ns('max_speed'):
                self.max_speed = param.value
                need_update = True
            if param.name == self.ns('mass'):
                self.mass = param.value
                need_update = True
            if param.name == self.ns('max_rudder_angle'):
                self.max_rudder_angle = param.value
                need_update = True
            if param.name == self.ns('max_rudder_change_rate'):
                self.max_rudder_change_rate = param.value
                need_update = True
            if param.name == self.ns('max_turn_rate'):
                self.max_turn_rate = param.value
                need_update = True
            if param.name == self.ns('rudder_distance'):
                self.rudder_distance = param.value
                need_update = True
            if param.name == self.ns('rudder_coefficient'):
                self.rudder_coefficient = param.value
                need_update = True
            if param.name == self.ns('thrust_noise'):
                self.thrust_noise = param.value
                need_update = True
            if param.name == self.ns('yaw_rate_noise'):
                self.yaw_rate_noise = param.value
                need_update = True
            if param.name == self.ns('drag_noise'):
                self.drag_noise = param.value
                need_update = True
        if need_update:
            self.update_coefficients()

    def update_coefficients(self) -> None:
        """Calculates coefficients that are used to balance out the various limits.
        
        The engine max power and the boat's top speed are used to calculate a maximum force.
        This maximum force is used to calculate a drag coefficient that results in forces
        being in equilibrium at top speed and rpm.
        """

        if not self.parameters_ready:
            return

        self.max_force = self.max_power/self.max_speed

        self.drag_coefficient = self.max_force /(self.max_speed**3)
        self.max_prop_rpm = self.max_rpm*self.gearbox_ratio

        if self.propulsion_type == 'prop':
            max_prop_speed = self.max_prop_rpm/self.prop_pitch
        
            self.prop_coefficient = self.max_force/(max_prop_speed**2-self.max_speed**2)

        if self.propulsion_type == 'jet':
            # power = a*rpm^3
            # a = power/rpm^3
            #self.jet_coefficient = self.model['max_power']/pow(self.max_prop_rpm,3)
            
            #https://www.engineeringtoolbox.com/jet-discharge-propulsion-force-d_1868.html
            # The propulsive force or thrust induced by the jet can be expressed as
            # F = ρ q (v2 - v1)
            # where v1 = jet velocity (m/s), v2 = velocity out of the jet (m/s), rho is density and q is volume
            
            max_turn_force = self.max_force*math.sin(math.radians(self.max_rudder_angle))

            # analagous to straight line drag coefficient, but to resist turning          
            # linear works better than cubic, this the **1
            self.go_straight_coefficient = max_turn_force / (math.radians(self.max_turn_rate)**1)

    
