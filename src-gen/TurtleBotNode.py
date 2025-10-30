import sys, os

import rclpy
import math

from rclpy.node import Node

from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from asyncio import Future

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Imu

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist

class TurtleBot(Node):

    # Init ROS2 variables, subscribers and publishers
    def __init__(self):
        super().__init__('distance_driver')

        self.init_variables()

        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, qos_profile_sensor_data)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, qos_profile_sensor_data)
        self.imu_sub = self.create_subscription(Imu, 'imu', self.imu_callback, qos_profile_sensor_data)

        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

    # Init all variables that are used for storing data
    def init_variables(self):
        # Laser data
        self.s_ranges = [0 for i in range(360)]
        self.s_intensity = [0 for i in range(360)]

        # Odometry data
        self.odom_x = 0
        self.odom_y = 0
        self.odom_z = 0

        # imu data
        self.orientation = Quaternion()

        # Velocity data
        self.vel_msg = Twist()
        self.vel_msg.linear.x = 0.0
        self.vel_msg.linear.y = 0.0
        self.vel_msg.linear.z = 0.0
        self.vel_msg.angular.x = 0.0
        self.vel_msg.angular.y = 0.0
        self.vel_msg.angular.z = 0.0

        # What callback has hapened
        self.last_callback = 0

    # Replace NaN values in lidar data with a neighbouring value.
    # NOTE: not used anymore, data is zeroed instead
    def replace_nan(self, data, i):
        if not math.isnan(data[i - 1]):
            return data[i - 1]
        if not math.isnan(data[i + 1]):
            return data[i + 1]
        # Both neighbours are NaN, try looking ahead
        return replace_nan(data, i + 1)


    # Code to convert the lidar2 data to the lidar1 format
    #
    # src_data contains a variable number of samples (normally ~240) from
    # angle_min to angle_max taken at `angle_increment` radian intervals.
    # Returns an array with 360 samples, where the missing samples are zeroed
    def convert_to_360_samples(self, src_data, angle_min, angle_max, angle_increment):

        for i, x in enumerate(src_data):
            if math.isnan(x):
                # TODO: check this if its ok to zero out missing data
                src_data[i] = 0 # self.replace_nan(src_data, i)

        s_ranges = [0 for i in range(360)]
        degree_incr = 1 / math.degrees(angle_increment)

        src_index = 0
        dest_index = int(math.degrees(angle_min))
        max_src_index = len(src_data) - 1
        dest_index_max = int(math.degrees(angle_max))
        for x in s_ranges:
            if dest_index >= dest_index_max: # no more valid data in src
                break
            s_ranges[dest_index] = src_data[int(min(src_index, max_src_index))]
            src_index += degree_incr
            dest_index += 1

        return s_ranges


    # Receive laser data
    def scan_callback(self, msgs):
        # Debug output
        #print("************** SP: {} {} {}".format(msgs.angle_min, msgs.angle_max, len(msgs.ranges)))

        #print("************** SP: fill angle {}-{} (valid degrees {})".format(int(math.degrees(msgs.angle_min)), math.degrees(msgs.angle_max), math.degrees(msgs.angle_increment) * len(msgs.ranges)))

        if len(msgs.ranges) < 360:  # Lidar2 data, convert to lidar1 format.
            self.s_ranges = self.convert_to_360_samples(msgs.ranges,
                    msgs.angle_min, msgs.angle_max, msgs.angle_increment)
            self.s_intensity = [0 for i in range(360)] # Missing in Lidar2
        else:
            self.s_ranges = msgs.ranges
            self.s_intensity = msgs.intensities

        self.last_callback = 1

    # Receive odometry data
    def odom_callback(self, msgs):
        self.odom_x = msgs.pose.pose.position.x
        self.odom_y = msgs.pose.pose.position.y
        self.odom_z = msgs.pose.pose.position.z
        self.last_callback = 2

    # Receive imu data
    def imu_callback(self, msgs):
        self.orientation = msgs.orientation
        self.last_callback = 3

    # Reset the last_callback value
    def reset_last(self):
        self.last_callback = 0

    # Publish velocity to ROS2
    def vel_publish(self, x=0.0, y=0.0, z=0.0, rx=0.0, ry=0.0, rz=0.0):
        self.vel_msg.linear.x = x
        self.vel_msg.linear.y = y
        self.vel_msg.linear.z = z
        self.vel_msg.angular.x = rx
        self.vel_msg.angular.y = ry
        self.vel_msg.angular.z = rz

        self.vel_pub.publish(self.vel_msg)
