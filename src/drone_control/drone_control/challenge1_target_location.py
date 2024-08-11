#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
import cv2 as cv

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from px4_msgs.msg import VehicleOdometry

class LocationConvertionNode(Node):

    drone_loc_: Point
    global_loc_: Point

    conversion_: Point
    
    def __init__(self):
        super().__init__("location_conversion")

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )

        self.drone_loc_ = None
        self.global_loc_ = None

        # Creates subscription to the current location.
        self.position_sub_ = self.create_subscription(Point, "/drone_sensing/local_position", self.position_callback, qos_profile)
        self.vehicle_sub_ = self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.odom_callback, qos_profile)

        # Creates publisher to the target location.
        self.target_pub_ = self.create_publisher(Point, "/drone_control/ch1_desired_location", qos_profile)
        self.tag_seen_pub_ = self.create_publisher(Bool, "/drone_sensing/tag_seen", qos_profile)


    def odom_callback(self, msg: VehicleOdometry):
        self.drone_loc_.x = msg.velocity[0]
        self.drone_loc_.y = msg.velocity[1]
        self.drone_loc_.z = msg.velocity[2]


    def position_callback(self, msg: Point):
        if self.conversion_ is not None:
            self.target_pub_.publish(self.conversion_)
        
        self.global_loc_.x = msg.x
        self.global_loc_.y = msg.y
        self.global_loc_.z = msg.z

        msg = Point()
        msg.x = 0 + self.drone_loc_.x - self.global_loc_.x
        msg.y = 0 + self.drone_loc_.y - self.global_loc_.y
        msg.z = 0 + self.drone_loc_.z - self.global_loc_.z

        self.conversion_ = msg
        self.target_pub_.publish(self.conversion_)

        msg_bool = Bool()
        msg_bool.data = True
        self.tag_seen_pub_.publish(msg_bool)

            
def main(args=None):
    rclpy.init(args=args)
    node = LocationConvertionNode()
    rclpy.spin(node)
    rclpy.shutdown()
