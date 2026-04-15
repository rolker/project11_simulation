#!/usr/bin/env python3

import math
import random
from typing import List

import asv_sim.asv_sim_node
from asv_sim_msgs.srv import SetPose
from geometry_msgs.msg import TwistWithCovarianceStamped
from rcl_interfaces.msg import Parameter
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Float64
import transforms3d


# Approximate meters-to-degrees at mid-latitudes
_METERS_PER_DEG_LAT = 111320.0


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

        node.declare_parameter(self.ns('namespace'), '/' + name)

        node.declare_parameter(self.ns('start_lat'), 0.0)
        node.declare_parameter(self.ns('start_lon'), 0.0)
        node.declare_parameter(self.ns('start_heading'), 0.0)

        node.declare_parameter(self.ns('mru_frame'), 'mru')

        # Sensor noise parameters (measurement uncertainty, Layer 3)
        node.declare_parameter(
            self.ns('sensor_noise.gps.horizontal'), 0.02,
            ParameterDescriptor(
                description='GPS horizontal noise std dev in meters '
                '(0.02 = RTK, 2.0 = standalone)'))
        node.declare_parameter(
            self.ns('sensor_noise.gps.vertical'), 0.04,
            ParameterDescriptor(
                description='GPS vertical noise std dev in meters'))
        node.declare_parameter(
            self.ns('sensor_noise.imu.orientation'), 0.1,
            ParameterDescriptor(
                description='IMU orientation noise std dev in degrees '
                '(applied to roll, pitch, yaw)'))
        node.declare_parameter(
            self.ns('sensor_noise.imu.angular_rate'), 0.01,
            ParameterDescriptor(
                description='IMU angular rate noise std dev in rad/s'))
        node.declare_parameter(
            self.ns('sensor_noise.velocity.speed'), 0.05,
            ParameterDescriptor(
                description='Velocity speed noise std dev in m/s'))
        node.declare_parameter(
            self.ns('sensor_noise.velocity.course'), 1.0,
            ParameterDescriptor(
                description='Velocity course noise std dev in degrees'))

        start = {}
        for k in self.start_params:
            start[k] = self.start_params[k]

        self.dynamics = asv_sim.dynamics.Dynamics(
            self.model, node.environment, start)

        self.diag_publishers = {}
        self.have_commands_pub = node.create_publisher(
            Bool, '~/' + self.name + '/have_commands', 5)

        self.reset_subscriber = node.create_subscription(
            Bool, '~/' + self.name + '/sim_reset',
            self.reset_callback, 5)

        self.set_pose_service = node.create_service(
            SetPose, '~/' + self.name + '/set_pose', self.set_pose)

    def updateParameters(self, parameters: List[Parameter]):
        for param in parameters:
            self.node.get_logger().info(
                'param: {}: {}'.format(param.name, param.value))

            if param.name == self.ns('namespace'):
                self.namespace = param.value
                self.position_publisher = (
                    self.node.create_publisher(
                        NavSatFix,
                        self.namespace + '/position', 5))
                self.orientation_publisher = (
                    self.node.create_publisher(
                        Imu,
                        self.namespace + '/orientation', 5))
                self.velocity_publisher = (
                    self.node.create_publisher(
                        TwistWithCovarianceStamped,
                        self.namespace + '/velocity', 5))

                self.throttle_subscriber = (
                    self.node.create_subscription(
                        Float32,
                        self.namespace + '/throttle',
                        self.throttle_callback, 5))
                self.rudder_subscriber = (
                    self.node.create_subscription(
                        Float32,
                        self.namespace + '/rudder',
                        self.rudder_callback, 5))

            if param.name == self.ns('start_lat'):
                self.start_params['lat'] = param.value
            if param.name == self.ns('start_lon'):
                self.start_params['lon'] = param.value
            if param.name == self.ns('start_heading'):
                self.start_params['heading'] = param.value
            if param.name == self.ns('mru_frame'):
                self.mru_frame = param.value

            # Sensor noise parameters
            if param.name == self.ns('sensor_noise.gps.horizontal'):
                self.gps_horizontal_noise = param.value
            if param.name == self.ns('sensor_noise.gps.vertical'):
                self.gps_vertical_noise = param.value
            if param.name == self.ns('sensor_noise.imu.orientation'):
                self.imu_orientation_noise = param.value
            if param.name == self.ns('sensor_noise.imu.angular_rate'):
                self.imu_angular_rate_noise = param.value
            if param.name == self.ns('sensor_noise.velocity.speed'):
                self.velocity_speed_noise = param.value
            if param.name == self.ns('sensor_noise.velocity.course'):
                self.velocity_course_noise = param.value

    def ns(self, key: str) -> str:
        return 'platforms.' + self.name + '.' + key

    def reset_callback(self, data):
        lat = self.start_params['lat'].value
        lon = self.start_params['lon'].value
        heading = self.start_params['heading'].value
        self.dynamics.set(lat, lon, heading)

    def set_pose(self, req):
        self.dynamics.set(
            req.point.position.latitude,
            req.point.position.longitude,
            req.nav.orientation.heading)
        return True

    def throttle_callback(self, data):
        self.throttle = max(min(data.data, 1.0), -1.0)
        self.last_command_timestamp = self.node.get_clock().now()

    def rudder_callback(self, data):
        self.rudder = max(min(data.data, 1.0), -1.0)

    def update(self):
        now = self.node.get_clock().now()
        if (self.last_command_timestamp is None
                or now - self.last_command_timestamp
                > rclpy.time.Duration(seconds=0.5)):
            self.throttle = 0.0
            self.rudder = 0.0
            self.have_commands_pub.publish(Bool(data=False))
        else:
            self.have_commands_pub.publish(Bool(data=True))

        diag = self.dynamics.update(self.throttle, self.rudder, now)

        for key, value in diag.items():
            if key not in self.diag_publishers:
                self.diag_publishers[key] = (
                    self.node.create_publisher(
                        Float64,
                        '~/' + self.name + '/diagnostics/' + key, 5))
            self.diag_publishers[key].publish(Float64(data=value))

    def updateNav(self):
        nsf = NavSatFix()
        nsf.header.stamp = self.dynamics.last_update.to_msg()
        nsf.header.frame_id = self.mru_frame
        nsf.status.status = NavSatStatus.STATUS_FIX

        # GPS position with sensor noise
        lat_deg = math.degrees(self.dynamics.latitude)
        lon_deg = math.degrees(self.dynamics.longitude)
        lat_noise_deg = (random.gauss(0.0, self.gps_horizontal_noise)
                         / _METERS_PER_DEG_LAT)
        cos_lat = math.cos(self.dynamics.latitude)
        lon_noise_deg = (random.gauss(0.0, self.gps_horizontal_noise)
                         / (_METERS_PER_DEG_LAT * max(cos_lat, 0.01)))
        nsf.latitude = lat_deg + lat_noise_deg
        nsf.longitude = lon_deg + lon_noise_deg
        nsf.altitude = (self.dynamics.altitude
                        + random.gauss(0.0, self.gps_vertical_noise))
        self.position_publisher.publish(nsf)

        # IMU orientation with sensor noise
        imu = Imu()
        imu.header.stamp = self.dynamics.last_update.to_msg()
        imu.header.frame_id = self.mru_frame
        orientation_noise_rad = math.radians(self.imu_orientation_noise)
        yaw = (math.radians(90.0) - self.dynamics.heading
               + random.gauss(0.0, orientation_noise_rad))
        pitch = (self.dynamics.pitch
                 + random.gauss(0.0, orientation_noise_rad))
        roll = (self.dynamics.roll
                + random.gauss(0.0, orientation_noise_rad))
        q = transforms3d.taitbryan.euler2quat(yaw, pitch, roll)
        imu.orientation.x = q[1]
        imu.orientation.y = q[2]
        imu.orientation.z = q[3]
        imu.orientation.w = q[0]
        imu.angular_velocity.z = (-self.dynamics.yaw_rate
                                  + random.gauss(
                                      0.0, self.imu_angular_rate_noise))
        imu.linear_acceleration.x = self.dynamics.a
        self.orientation_publisher.publish(imu)

        # Velocity with sensor noise
        twcs = TwistWithCovarianceStamped()
        twcs.header.stamp = self.dynamics.last_update.to_msg()
        twcs.header.frame_id = self.mru_frame

        noisy_sog = max(
            0.0,
            self.dynamics.sog
            + random.gauss(0.0, self.velocity_speed_noise))
        noisy_cog = (self.dynamics.cog
                     + math.radians(
                         random.gauss(0.0, self.velocity_course_noise)))
        course_angle = math.radians(90.0) - noisy_cog
        twcs.twist.twist.linear.x = math.cos(course_angle) * noisy_sog
        twcs.twist.twist.linear.y = math.sin(course_angle) * noisy_sog
        self.velocity_publisher.publish(twcs)
