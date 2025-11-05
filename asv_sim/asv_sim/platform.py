#!/usr/bin/env python3

import rclpy
import math

import asv_sim.asv_sim_node
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistWithCovarianceStamped
from std_msgs.msg import Float64
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from rcl_interfaces.msg import Parameter

from asv_sim_msgs.srv import SetPose

import transforms3d

from typing import List


class Platform:
    def __init__(self, name: str, node: 'asv_sim.asv_sim_node.AsvSim'):
        self.name = name
        self.throttle = 0.0
        self.rudder = 0.0
        self.last_command_timestamp = None
        self.node = node

        self.position_publisher = None
        self.orientation_publisher = None
        self.velocity_publisher = None
        self.start_params = {}

        self.throttle_subscriber = None
        self.rudder_subscriber = None

        node.add_post_set_parameters_callback(self.updateParameters)

        node.declare_parameter(self.ns('model'), 'default')

        model_name = node.get_parameter(self.ns('model')).value
        self.model = node.getModel(model_name)

        node.declare_parameter(self.ns("namespace"), '/'+name)
        
        node.declare_parameter(self.ns('start_lat'), 0.0)
        node.declare_parameter(self.ns('start_lon'), 0.0)
        node.declare_parameter(self.ns('start_heading'), 0.0)

        node.declare_parameter(self.ns('mru_frame'), 'mru')

        start = {}
        for k in self.start_params:
            start[k] = self.start_params[k]

        self.dynamics = asv_sim.dynamics.Dynamics(self.model, node.environment, start)
        
        self.diag_publishers = {}
        self.have_commands_pub = node.create_publisher(Bool, '~/'+self.name+'/have_commands', 5)

        self.reset_subscriber = node.create_subscription(Bool, '~/'+self.name+'/sim_reset', self.reset_callback, 5)

        self.set_pose_service = node.create_service(SetPose, '~/'+self.name+'/set_pose', self.set_pose)

        
    def updateParameters(self, parameters: List[Parameter]):
        for param in parameters:
            self.node.get_logger().info('param: {}: {}'.format(param.name, param.value))

            if param.name == self.ns('namespace'):
                self.namespace = param.value
                self.position_publisher = self.node.create_publisher(NavSatFix, self.namespace+'/position', 5)
                self.orientation_publisher = self.node.create_publisher(Imu, self.namespace+'/orientation', 5)
                self.velocity_publisher = self.node.create_publisher(TwistWithCovarianceStamped, self.namespace+'/velocity', 5)

                self.throttle_subscriber = self.node.create_subscription(Float32, self.namespace+'/throttle', self.throttle_callback, 5)
                self.rudder_subscriber = self.node.create_subscription(Float32, self.namespace+'/rudder', self.rudder_callback, 5)

            if param.name == self.ns('start_lat'):
                self.start_params['lat'] = param.value
            if param.name == self.ns('start_lon'):
                self.start_params['lon'] = param.value
            if param.name == self.ns('start_heading'):
                self.start_params['heading'] = param.value
            if param.name == self.ns('mru_frame'):
                self.mru_frame = param.value




    def ns(self, key: str) -> str:
        return 'platforms.'+self.name+'.'+key

    def reset_callback(self, data):
        lat = self.start_params['lat'].value
        lon = self.start_params['lon'].value
        heading = self.start_params['heading'].value
        self.dynamics.set(lat, lon, heading)

    def set_pose(self, req):
        self.dynamics.set(req.point.position.latitude, req.point.position.longitude, req.nav.orientation.heading)
        return True

    def throttle_callback(self, data):
        self.throttle = max(min(data.data,1.0),-1.0)
        self.last_command_timestamp = self.node.get_clock().now()
        
    def rudder_callback(self, data):
        self.rudder = max(min(data.data,1.0),-1.0)

    def update(self):
        now = self.node.get_clock().now()
        if self.last_command_timestamp is None or now - self.last_command_timestamp > rclpy.time.Duration(seconds = 0.5):
            self.throttle = 0.0
            self.rudder = 0.0
            self.have_commands_pub.publish(Bool(data=False))
        else:
            self.have_commands_pub.publish(Bool(data=True))
        
        diag = self.dynamics.update(self.throttle, self.rudder, now)

        for key,value in diag.items():
            if not key in self.diag_publishers:
                self.diag_publishers[key] = self.node.create_publisher(Float64, '~/'+self.name+'/diagnostics/'+key, 5)
            self.diag_publishers[key].publish(Float64(data=value))
        
    def updateNav(self):
        nsf = NavSatFix()
        nsf.header.stamp = self.dynamics.last_update.to_msg()
        nsf.header.frame_id = self.mru_frame
        nsf.status.status = NavSatStatus.STATUS_FIX
        nsf.latitude = math.degrees(self.dynamics.latitude)
        nsf.longitude = math.degrees(self.dynamics.longitude)
        self.position_publisher.publish(nsf)

        imu = Imu()                  
        imu.header.stamp = self.dynamics.last_update.to_msg()
        imu.header.frame_id = self.mru_frame
        yaw = math.radians(90.0)-self.dynamics.heading
        q = transforms3d.taitbryan.euler2quat(yaw, 0, 0)
        imu.orientation.x = q[1]
        imu.orientation.y = q[2]
        imu.orientation.z = q[3]
        imu.orientation.w = q[0]
        imu.angular_velocity.z = -self.dynamics.yaw_rate
        imu.linear_acceleration.x = self.dynamics.a
        self.orientation_publisher.publish(imu)

        twcs = TwistWithCovarianceStamped()
        twcs.header.stamp = self.dynamics.last_update.to_msg()
        twcs.header.frame_id = self.mru_frame
        

        course_angle = math.radians(90.0)-self.dynamics.cog
        sin_cog = math.sin(course_angle)
        cos_cog = math.cos(course_angle)
        
        twcs.twist.twist.linear.x = cos_cog*self.dynamics.sog
        twcs.twist.twist.linear.y = sin_cog*self.dynamics.sog
        self.velocity_publisher.publish(twcs)

        
