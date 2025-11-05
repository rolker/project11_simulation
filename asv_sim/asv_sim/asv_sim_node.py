#!/usr/bin/env python3

# Roland Arsenault
# Center for Coastal and Ocean Mapping
# University of New Hampshire
# Copyright 2017, All rights reserved.

import rclpy
import rclpy.node

from std_msgs.msg import Bool

from rcl_interfaces.msg import ParameterDescriptor

import asv_sim.dynamics
import asv_sim.environment
import asv_sim.model
import asv_sim.platform


class AsvSim(rclpy.node.Node):
    def __init__(self):
        super().__init__('asv_sim')

        self.models = {}

        self.platforms_param = self.declare_parameter('platforms',['default'], ParameterDescriptor(description="List of platforms to enable"))
        
        self.environment = asv_sim.environment.Environment(self)

        self.platforms = []
        for platform_name in self.platforms_param.value:
            self.platforms.append(asv_sim.platform.Platform(platform_name, self))

        self.reset_subscriber = self.create_subscription(Bool, '~/sim_reset', self.reset_callback, 1)
        
        self.update_timer = self.create_timer(0.05, self.update)
        self.nav_time = self.create_timer(0.2, self.updateNav)
        
    def getModel(self, model_name: str) -> asv_sim.model.Model:
        if not model_name in self.models:
            self.models[model_name] = asv_sim.model.Model(model_name, self)
        return self.models[model_name]

    
    def reset_callback(self, data):
        for p in self.platforms:
            p.reset_callback(data)

    def update(self):
        for p in self.platforms:
            p.update()

    def updateNav(self):
        for p in self.platforms:
            p.updateNav()


def main(args=None):
    rclpy.init(args=args)
    sim = AsvSim()
    rclpy.spin(sim)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
