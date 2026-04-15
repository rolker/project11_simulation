#!/usr/bin/env python3

from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.executors import SingleThreadedExecutor
from rclpy.lifecycle import Node
from rclpy.lifecycle import LifecyclePublisher
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.timer import Timer

from marine_acoustic_msgs.msg import DetectionFlag
from marine_acoustic_msgs.msg import SonarDetections
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from std_msgs.msg import Header
from sensor_msgs_py import point_cloud2
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField

import math
import marine_autonomy
from ament_index_python import get_package_share_directory
import pathlib

from .bathy_grid import BathyGrid

class SonarSim(Node):
    def __init__(self, node_name='mbes_sim', **kwargs):
        super().__init__(node_name, **kwargs)
        self.get_logger().debug('SonarSim init')
        self.depth_publisher: Optional[LifecyclePublisher] = None
        self.detections_publisher: Optional[LifecyclePublisher] = None
        self.ping_publisher: Optional[LifecyclePublisher] = None
        self.ping_timer: Optional[Timer] = None
        self.sound_speed = 1500.0  # m/s, typical speed of sound in water
        self.frequency = 200000.0  # Hz, typical frequency for multibeam sonar
        self.grid_file = ""
        self.tide_level = 0.0  # meters above MLLW, updated by subscription

    def on_configure(self, state: State):
        default_grid_file = pathlib.Path(get_package_share_directory('mbes_sim'))/'data'/'US5NH02M.tiff'
        self.declare_parameter('grid_file', str(default_grid_file))
        self.declare_parameter('swath_angle', 120.0)
        self.declare_parameter('beam_count', 120)
        self.declare_parameter('ping_interval', 1.0)
        self.declare_parameter('sonar_frame_id', 'mbes')
        self.declare_parameter('tide_level_topic', 'tide_level')
        self.depth_publisher = self.create_lifecycle_publisher(Float32, 'depth', 5) # type: ignore
        self.detections_publisher = self.create_lifecycle_publisher(SonarDetections, 'detections', 5) # type: ignore
        self.ping_publisher = self.create_lifecycle_publisher(PointCloud2, 'soundings', 10) # type: ignore
        tide_topic = self.get_parameter('tide_level_topic').get_parameter_value().string_value
        self.tide_subscription = self.create_subscription(
            Float64, tide_topic, self.tide_callback, 5)
        return super().on_configure(state)


    def tide_callback(self, msg: Float64):
        self.tide_level = msg.data

    def on_activate(self, state):
        grid_file = self.get_parameter('grid_file').get_parameter_value().string_value
        self.get_logger().debug(f'opening grid_file: {grid_file}')
        self.bathy = BathyGrid(grid_file)
        self.get_logger().debug('initializing robot')
        self.robot = marine_autonomy.nav.RobotNavigation(self)
        self.ping_rate = self.get_parameter('ping_interval').get_parameter_value().double_value
        ping_rate = self.ping_rate
        if ping_rate <= 0.0:
            ping_rate = 1.0
            self.ping_rate = 0.0
        self.get_logger().debug(f'initializing timer with ping_rate: {ping_rate}')
        self.ping_timer = self.create_timer(ping_rate, self.ping_callback)
        return super().on_activate(state)
    
    def on_deactivate(self, state):
        if self.ping_timer is not None:
            self.destroy_timer(self.ping_timer)
        self.ping_timer = None
        return super().on_deactivate(state)

    def on_cleanup(self, state):
        if self.ping_timer is not None:
            self.destroy_timer(self.ping_timer)
        self.ping_timer = None
        if self.depth_publisher is not None:
            self.destroy_publisher(self.depth_publisher)
        self.depth_publisher = None
        if self.detections_publisher is not None:
            self.destroy_publisher(self.detections_publisher)
        self.detections_publisher = None
        if self.ping_publisher is not None:
            self.destroy_publisher(self.ping_publisher)
        self.ping_publisher = None
        if hasattr(self, 'tide_subscription') and self.tide_subscription is not None:
            self.destroy_subscription(self.tide_subscription)
            self.tide_subscription = None
        return super().on_cleanup(state)

    def on_shutdown(self, state):
        if self.ping_timer is not None:
            self.destroy_timer(self.ping_timer)
        self.ping_timer = None
        if self.depth_publisher is not None:
            self.destroy_publisher(self.depth_publisher)
        self.depth_publisher = None
        if self.detections_publisher is not None:
            self.destroy_publisher(self.detections_publisher)
        self.detections_publisher = None
        if self.ping_publisher is not None:
            self.destroy_publisher(self.ping_publisher)
        self.ping_publisher = None
        if hasattr(self, 'tide_subscription') and self.tide_subscription is not None:
            self.destroy_subscription(self.tide_subscription)
            self.tide_subscription = None
        return super().on_shutdown(state)

    def ping_callback(self):
        if self.depth_publisher is None or not self.depth_publisher.is_activated:
            return

        grid_file = self.get_parameter('grid_file').get_parameter_value().string_value
        if grid_file != self.grid_file:
            self.grid_file = grid_file
            self.get_logger().debug(f'opening grid_file: {grid_file}')
            self.bathy = BathyGrid(grid_file)
        
        now = self.get_clock().now()
        self.get_logger().debug(f'mbes_sim: ping_callback at {now}')
        position = self.robot.positionLatLon()
        self.get_logger().debug(f'position: {position}')
        if position is None:
            return

        ping_rate = self.get_parameter('ping_interval').get_parameter_value().double_value
        if ping_rate != self.ping_rate:
            if ping_rate <= 0.0:
                ping_rate = 1.0
                self.ping_rate = 0.0
            self.ping_timer = self.create_timer(ping_rate, self.ping_callback)

        if self.ping_rate <= 0.0:
            return

        swath_angle = self.get_parameter('swath_angle').get_parameter_value().double_value
        tan_half_swath_angle = math.tan(math.radians(swath_angle/2.0))
        beam_count = self.get_parameter('beam_count').get_parameter_value().integer_value
        frame_id = self.get_parameter('sonar_frame_id').get_parameter_value().string_value


        lon_rad = position[1]
        lat_rad = position[0]
        lon_deg = math.degrees(lon_rad)
        lat_deg = math.degrees(lat_rad)

        chart_depth = self.bathy.getDepthAtLatLon(lat_deg, lon_deg)
        self.get_logger().debug(f'chart_depth: {chart_depth}')
        if chart_depth is not None and chart_depth >= 0:
            # Actual water depth = chart depth (MLLW) + tide above MLLW
            depth = chart_depth + self.tide_level
            try:
                depth_msg = Float32()
                depth_msg.data = float(depth)
                self.get_logger().debug(f'publishing depth message {depth_msg}')
                self.depth_publisher.publish(depth_msg)
            except Exception as e:
                self.get_logger().warning("Error publishing depth: {e}")
            heading = self.robot.heading()
            self.get_logger().debug(f'heading: {heading}')
            if heading is not None:
                swath_half_width = depth*tan_half_swath_angle
                #print 'swath half width:',swath_half_width
                port_outer_beam_location = marine_autonomy.geodesic.direct(lon_rad, lat_rad, math.radians(heading-90),swath_half_width)
                starboard_outer_beam_location = marine_autonomy.geodesic.direct(lon_rad, lat_rad, math.radians(heading+90),swath_half_width)
                #print 'outer beam locations:',port_outer_beam_location,starboard_outer_beam_location
                port_outer_beam_location_xy = self.bathy.getXY(math.degrees(port_outer_beam_location[1]), math.degrees(port_outer_beam_location[0]))
                starboard_outer_beam_location_xy = self.bathy.getXY(math.degrees(starboard_outer_beam_location[1]), math.degrees(starboard_outer_beam_location[0]))
                dx = (starboard_outer_beam_location_xy[0] - port_outer_beam_location_xy[0])/float(beam_count)
                dy = (starboard_outer_beam_location_xy[1] - port_outer_beam_location_xy[1])/float(beam_count)
                
                sounding_spacing = 2.0*swath_half_width/float(beam_count)
                
                soundings = []
                ranges = []
                angles = []

                for i in range(beam_count):
                    x = port_outer_beam_location_xy[0] + dx*i
                    y = port_outer_beam_location_xy[1] + dy*i
                    z = self.bathy.getDepth(x,y)
                    if z is not None:
                        z = z + self.tide_level
                        soundings.append((0.0, -swath_half_width+i*sounding_spacing, z))
                        y2 = soundings[-1][1]*soundings[-1][1]
                        z2 = z*z
                        ranges.append(math.sqrt(y2 + z2))
                        angles.append(math.atan2(soundings[-1][1], z))

                fields = [
                    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
                ]
                header = Header()
                header.frame_id = frame_id
                header.stamp = now.to_msg()
                pc2 = point_cloud2.create_cloud(header, fields, soundings)

                self.ping_publisher.publish(pc2)

                detections = SonarDetections()
                detections.header = header
                detections.ping_info.frequency = self.frequency
                detections.ping_info.sound_speed = self.sound_speed
                for i in range (len(soundings)):
                    detections.two_way_travel_times.append(2.0*ranges[i] / self.sound_speed)
                    detections.flags.append(DetectionFlag(flag=DetectionFlag.DETECT_OK))
                    detections.rx_angles.append(angles[i])
                    detections.intensities.append(1.0)

                self.detections_publisher.publish(detections)                        

def main(args=None):
    rclpy.init()
    executor = SingleThreadedExecutor()
    sim = SonarSim('mbes_sim')
    executor.add_node(sim)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

if __name__ == '__main__':
    main()