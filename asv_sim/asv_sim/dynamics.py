#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Roland Arsenault
# Center for Coastal and Ocean Mapping
# University of New Hampshire
# Copyright 2017, All rights reserved.

import math
import random
from . import geodesic, model, environment
import rclpy.constants

class Dynamics(object):
    """Simulate the dynamics of a boat.
    
    Using throttle and rudder inputs, calculate simplified forces acting on a boat changing its
    position, speed, and heading.
    """
    
    def __init__(self, model: model.Model, environment: environment.Environment =None, start=None):
        """Initialize the simulation with a model and optional environment.
        
        """

        # Portsmouth, New Hampshire, cod rock
        self.start_lat =  43.073397415457535
        self.start_lon = -70.71054174878898
        self.start_heading = 60.0

        if start is not None:
            if 'lat' in start:
                self.start_lat = start['lat']
            if 'lon' in start:
                self.start_lon = start['lon']
            if 'heading' in start:
                self.start_heading = start['heading']

        self.model = model
        self.environment = environment
        
        self.reset()
        self.last_update = None

        # jitters are used to add gaussian noise to components of the model to simulate small
        # disturbances that would be difficult to model.
        self.jitters = {'thrust':0.1, 'rudder':0.25, 'drag':0.1, 'current_speed':0.1, 'current_direction': 0.25}


    def set(self, lat, lon, heading):
        """Sets the boat to idle at the provided position and heading."""
        
        self.rpm = 0.0
        self.rudder_angle = 0.0
        self.bucket_position = 0.0
        self.speed = 0.0
        self.yaw_rate = 0.0
        self.longitude = math.radians(lon)
        self.latitude = math.radians(lat)

        self.heading = math.radians(heading)
        self.pitch = 0.0
        self.roll = 0.0

        self.sog = 0.0
        self.cog = 0.0

    def reset(self):
        """Resets the simulation to starting location and idle state."""

        self.set(self.start_lat, self.start_lon, self.start_heading)

            
    def update(self, throttle, rudder, timestamp, apply_noise = True):
        """Updates the simulation one step.
        
        The simulation does not impose an update rate or expect that
        updates are at regular intervals. It's the system using this
        model that is responsible for continuously calling this
        method at an adequate rate.
        
        :param throttle: Normalized throttle value in the -1 to 1 range.
        :param rudder: Normalized rudder value from -1 (left)  to 1 (right).
        :param timestamp: Time in seconds for the current iteration. It only 
            matters that the time reference is consistent, since it's only
            used to calculate elapsed time since the last update.
        :param  apply_noise: True if noise should be applied for simulating variability. 
        """
        
        # collect useful debugging info
        diagnostics = {'rudder':rudder}
        
        # clamp throttle input
        throttle = min(1.0,max(-1.0,throttle))
        reverse = throttle < 0
        throttle = abs(throttle)
        diagnostics['throttle'] = throttle
        
        # Figure out the time interval since the last update
        if self.last_update is None:
            delta_t = None
        else:
            delta_t = (timestamp-self.last_update).nanoseconds/float(rclpy.constants.S_TO_NS)
            if delta_t <= 0:
                delta_t = None
        self.last_update = timestamp

        # This implements the inertia of the engine as it changes RPMs.
        target_rpm = self.model.idle_rpm + throttle * (self.model.max_rpm-self.model.idle_rpm)
        diagnostics['target_rpm'] = target_rpm
        if(target_rpm != self.rpm):
            if delta_t is not None:
                rcr = (target_rpm - self.rpm)/delta_t
                max_rpm_change_rate = self.model.max_rpm_change_rate
                if abs(rcr) > max_rpm_change_rate:
                    if rcr < 0:
                        rcr = -max_rpm_change_rate
                    else:
                        rcr = max_rpm_change_rate
                self.rpm += rcr*delta_t
                
        diagnostics['rpm'] = self.rpm        

        # Apply the gearbox and clutch.
        if self.rpm < self.model.clutch_engagement_rpm:
            prop_rpm = 0.0
        else:
            prop_rpm = self.model.gearbox_ratio*self.rpm

        diagnostics['prop_rpm'] = prop_rpm

        target_rudder_angle = rudder*self.model.max_rudder_angle
        diagnostics['target_rudder_angle'] = target_rudder_angle
        
        if(self.rudder_angle != target_rudder_angle and delta_t is not None):
            possible_rudder_change = self.model.max_rudder_change_rate*delta_t
            if abs(target_rudder_angle-self.rudder_angle) < possible_rudder_change:
                self.rudder_angle = target_rudder_angle
            else:
                if (target_rudder_angle < self.rudder_angle):
                    self.rudder_angle -= possible_rudder_change
                else:
                    self.rudder_angle += possible_rudder_change

        diagnostics['rudder_angle'] = self.rudder_angle
        rudder_rads = math.radians(self.rudder_angle)

        if self.model.propulsion_type == 'prop':
            # Calculate the speed of the water moved by the prop.
            # TODO: figure out why this seems to differ from the definition
            # of pitch as being inches of distance traveled per revolution
            prop_speed = prop_rpm/self.model.prop_pitch

            # TODO: implement need to drop rpms to switch between fwd and rev
            if reverse:
                prop_speed = -prop_speed
            diagnostics['propwash_speed'] = prop_speed

            # Assuming the rudder is in the prop wash, determine the speed of the
            # water flowing by the rudder.
            if prop_speed > 0:
                rudder_speed = max(math.sqrt(prop_speed),self.speed)
            else:
                rudder_speed = self.speed

            # Calculate force created by the difference of the speed of
            # the water pushed by the prop and the speed of the boat
            # through water.
            #
            # From https://wright.nasa.gov/airplane/propth.html
            #
            # F = .5 * r * A * [Ve ^2 - V0 ^2]
            # 
            # Replace .5 * r * A by prop_coefficient to get the following
            thrust = self.model.prop_coefficient*(prop_speed**2-self.speed**2)
            diagnostics['prop_coefficient'] = self.model.prop_coefficient
            
        if self.model.propulsion_type == 'jet':

            # https://www.engineeringtoolbox.com/affinity-laws-d_408.html
            # flow is directly proportional to rpm
            
            #jet_speed = self.max_jet_speed*prop_rpm/self.max_prop_rpm
            #diagnostics['jet_speed'] = jet_speed
            
            # F = ρ q (v2 - v1)
            #thrust = max(0.0,self.rho_q*(jet_speed-self.speed))
            
            thrust = self.model.max_force*prop_rpm/self.model.max_prop_rpm
            
            # todo: model bucket movement
            if reverse:
                thrust = -thrust*self.model.bucket_efficiency
            
        if apply_noise:
            thrust = random.gauss(thrust,thrust*self.model.thrust_noise)
        diagnostics['thrust'] = thrust
        
        diagnostics['max_force'] = self.model.max_force


        # decompose the thrust into vectors that push the boat
        # forward and that rotates the boat
        thrust_fwd = thrust*math.cos(rudder_rads)
        diagnostics['thrust_fwd'] = thrust_fwd

        if self.model.propulsion_type == 'prop':
            rudder_speed_yaw = rudder_speed*math.sin(rudder_rads)
            # The distance between the center of mass and the rudder affect how quickly the boat can turn.
            self.yaw_rate = self.model.rudder_coefficient*rudder_speed_yaw/self.model.rudder_distance

        if self.model.propulsion_type == 'jet':
            yaw_thrust = thrust*math.sin(rudder_rads)
            
            yaw_drag = -(self.yaw_rate**1)*self.model.go_straight_coefficient
            
            diagnostics['yaw_thrust'] = yaw_thrust
            diagnostics['yaw_drag'] = yaw_drag
            
            torque = (yaw_thrust+yaw_drag)*self.model.rudder_distance
            
            # https://en.wikipedia.org/wiki/Moment_of_inertia
            # torque = I * angular acceleration
            
            # let's oversimplify the asv to a thin disc
            # I = 1/2 * m * r^2
            
            moment_of_inertia = self.model.mass*pow(self.model.rudder_distance,2)/2.0
            
            angular_acceleration = torque/moment_of_inertia
            
            if delta_t is not None:
                self.yaw_rate += angular_acceleration*delta_t
            
        if apply_noise:
            self.yaw_rate += random.gauss(0.0, self.model.yaw_rate_noise)

        diagnostics['yaw_rate'] = self.yaw_rate
            
        if delta_t is not None:
            self.heading += self.yaw_rate*delta_t
            self.heading = math.fmod(self.heading,math.radians(360))
            if self.heading < 0:
                self.heading += math.radians(360)

        # Calculate drag increasing with the cube of the speed.
        drag = (self.speed**3)*self.model.drag_coefficient
        if apply_noise:
            drag = random.gauss(drag,drag*self.model.drag_noise)

        diagnostics['drag'] = drag
        diagnostics['net_force'] = thrust_fwd-drag

        # calculate acceleration from f=ma.
        self.a = (thrust_fwd-drag)/self.model.mass

        if delta_t is not None:
            self.speed += self.a*delta_t

        # update position based on speed
        if delta_t is not None:
            last_lat = self.latitude
            last_long = self.longitude
            delta = self.speed*delta_t
            self.longitude,self.latitude = geodesic.direct(self.longitude,self.latitude,self.heading,delta)
            # apply currents if applicable
            if self.environment is not None:
                c = self.environment.getCurrent(self.latitude, self.longitude, apply_noise)
                if c is not None:
                    diagnostics['current_speed'] = c['speed']
                    diagnostics['current_direction'] = c['direction']
                    current_speed = c['speed']
                    current_direction = math.radians(c['direction'])

                    self.longitude,self.latitude = geodesic.direct(self.longitude,self.latitude,current_direction,current_speed*delta_t)
            self.cog, self.sog = geodesic.inverse(last_long, last_lat, self.longitude, self.latitude)
            self.sog /= delta_t
            
        return diagnostics
        
