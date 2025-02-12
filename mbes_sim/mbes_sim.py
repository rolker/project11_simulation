#!/usr/bin/env python3

from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.executors import SingleThreadedExecutor
from rclpy.lifecycle import Node
from rclpy.lifecycle import Publisher
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn


from std_msgs.msg import Float32
from std_msgs.msg import Header
from sensor_msgs_py import point_cloud2
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField

import math
import project11
from ament_index_python import get_package_share_directory
import pathlib

from .bathy_grid import BathyGrid

class SonarSim(Node):
    def __init__(self, node_name='mbes_sim', **kwargs):
        super().__init__(node_name, **kwargs)
        self.get_logger().debug('SonarSim init')
        self.depth_publisher: Optional[Publisher] = None
        self.ping_publisher: Optional[Publisher] = None

    def on_configure(self, state):
        default_grid_file = pathlib.Path(get_package_share_directory('mbes_sim'))/'data'/'US5NH02M.tiff'
        self.declare_parameter('grid_file', str(default_grid_file))
        self.declare_parameter('swath_angle', 120.0)
        self.declare_parameter('beam_count', 20)
        self.declare_parameter('ping_interval', 1.0)
        self.declare_parameter('sonar_frame_id', 'mbes')
        self.depth_publisher = self.create_lifecycle_publisher(Float32, 'depth', 5)
        self.ping_publisher = self.create_lifecycle_publisher(PointCloud2, 'soundings', 10)
        return super().on_configure(state)


    def on_activate(self, state):
        grid_file = self.get_parameter('grid_file').get_parameter_value().string_value
        self.get_logger().debug(f'opening grid_file: {grid_file}')
        self.bathy = BathyGrid(grid_file)
        self.get_logger().debug('initializing robot')
        self.robot = project11.nav.RobotNavigation(self)
        self.get_logger().debug('reading parameters')
        self.swath_angle = self.get_parameter('swath_angle').get_parameter_value().double_value
        self.tan_half_swath_angle = math.tan(math.radians(self.swath_angle/2.0))
        self.beam_count = self.get_parameter('beam_count').get_parameter_value().integer_value
        self.ping_rate = self.get_parameter('ping_interval').get_parameter_value().double_value
        self.frame_id = self.get_parameter('sonar_frame_id').get_parameter_value().string_value
        self.get_logger().debug(f'initializing timer with ping_rate: {self.ping_rate}')
        self.ping_timer = self.create_timer(self.ping_rate, self.ping_callback)
        return super().on_activate(state)
    
    def on_deactivate(self, state):
        self.destroy_timer(self.ping_timer)
        self.ping_timer = None
        return super().on_deactivate(state)

    def on_cleanup(self, state):
        self.destroy_timer(self.ping_timer)
        self.ping_timer = None
        self.destroy_publisher(self.depth_publisher)
        self.depth_publisher = None
        self.destroy_publisher(self.ping_publisher)
        self.ping_publisher = None
        return super().on_cleanup(state)
    
    def on_shutdown(self, state):
        self.destroy_timer(self.ping_timer)
        self.ping_timer = None
        self.destroy_publisher(self.depth_publisher)
        self.depth_publisher = None
        self.destroy_publisher(self.ping_publisher)
        self.ping_publisher = None
        return super().on_shutdown(state)

    def ping_callback(self):
        if self.depth_publisher is None or not self.depth_publisher.is_activated:
            return
        now = self.get_clock().now()
        self.get_logger().debug(f'mbes_sim: ping_callback at {now}')
        position = self.robot.positionLatLon()
        self.get_logger().debug(f'position: {position}')
        if position is None:
            return
        lon_rad =position[1]
        lat_rad = position[0]
        lon_deg = math.degrees(lon_rad)
        lat_deg = math.degrees(lat_rad)

        depth = self.bathy.getDepthAtLatLon(lat_deg, lon_deg)
        self.get_logger().debug(f'depth: {depth}')
        if depth is not None and depth >= 0:
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
                swath_half_width = depth*self.tan_half_swath_angle
                #print 'swath half width:',swath_half_width
                port_outer_beam_location = project11.geodesic.direct(lon_rad, lat_rad, math.radians(heading-90),swath_half_width)
                starboard_outer_beam_location = project11.geodesic.direct(lon_rad, lat_rad, math.radians(heading+90),swath_half_width)
                #print 'outer beam locations:',port_outer_beam_location,starboard_outer_beam_location
                port_outer_beam_location_xy = self.bathy.getXY(math.degrees(port_outer_beam_location[1]), math.degrees(port_outer_beam_location[0]))
                starboard_outer_beam_location_xy = self.bathy.getXY(math.degrees(starboard_outer_beam_location[1]), math.degrees(starboard_outer_beam_location[0]))
                dx = (starboard_outer_beam_location_xy[0] - port_outer_beam_location_xy[0])/float(self.beam_count)
                dy = (starboard_outer_beam_location_xy[1] - port_outer_beam_location_xy[1])/float(self.beam_count)
                
                sounding_spacing = 2.0*swath_half_width/float(self.beam_count)
                
                soundings = []

                for i in range(self.beam_count):
                    x = port_outer_beam_location_xy[0] + dx*i
                    y = port_outer_beam_location_xy[1] + dy*i
                    z = self.bathy.getDepth(x,y)
                    #print 'depth:', z
                    if z is not None:
                        soundings.append((0.0, -swath_half_width+i*sounding_spacing, z))

                fields = [
                    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
                ]
                header = Header()
                header.frame_id = self.frame_id
                header.stamp = now.to_msg()
                pc2 = point_cloud2.create_cloud(header, fields, soundings)

                self.ping_publisher.publish(pc2)
                        

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