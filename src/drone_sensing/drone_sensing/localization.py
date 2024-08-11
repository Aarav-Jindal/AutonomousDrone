#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

# For programming.
import numpy as np
import cv2 as cv

import time

# For documentating the code only.
import rclpy.subscription
import rclpy.publisher

# Messages that might be received and published.
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from px4_msgs.msg import VehicleOdometry

from drone_msgs.msg import ARTags, TargetLocation
from std_msgs.msg import Bool
from geometry_msgs.msg import Point

AR_TAG_MAPPING_ = {
    30: [0, 0, 0],
    79: [2, 0, 0],
    85: [3, 1, 0],
    86: [1, 2, 0],
    31: [0, 3, 0],
    83: [2, 3, 0],
    89: [4, 3, 2],
    90: [1, 4, 0],
    20: [3, 5, 0],
    17: [0, 6, 0],
    33: [2, 6, 0],
    23: [4, 7, 0],
    22: [0, 8, 0],
    21: [2, 8, 0],
    10: [4, 9, 0],
    18: [1, 10, 0],
    24: [3, 10, 0],
    26: [2, 11, 0],
    12: [4, 11, 0],
    19: [0, 13, 0],
    9: [3, 13, 0]
}

# Length of a unit of the real world map in meters.
UNIT_LENGTH_ = 0.60

# Side length of the AR Tags in meters.
OFFSET_LENGTH_ = 0.2666

# Camera calibration matrix and distortion matrix.
CALIBRATION_MATRIX_ = np.array([
    [1241.734394, 0.000000, 330.280649],
    [0.000000, 1245.872682, 189.139639],
    [0.000000, 0.000000, 1.000000]
])
DIST_MATRIX_ = np.array([0.008765, 0.386126, 0.003070, 0.004573, 0.000000])

# Retrieve the real-world coordinates of all four corners of an AR tag.
def get_locations(id: int):
    print(id)
    coord = AR_TAG_MAPPING_[id]

    if coord is None:
        return None
    
    cx = coord[0] * UNIT_LENGTH_
    cy = coord[1] * UNIT_LENGTH_
    of = OFFSET_LENGTH_ / 2

    return np.array([
        [cx-of, cy+of, 0],
        [cx+of, cy+of, 0],
        [cx+of, cy-of, 0],
        [cx-of, cy-of, 0]
    ])


class LocalizationNode(Node):
    """
    Obtain the localized position from AR Tags.
    """

    artag_loc_sub_: rclpy.subscription.Subscription
    artag_id_sub_: rclpy.subscription.Subscription

    cur_x: float
    cur_y: float
    cur_z: float
    """
    All stands for the odometry data.
    """

    hasTrans: bool

    trans_x: float
    trans_y: float
    trans_z: float
    """
    All stands for:
    - drone_local + trans_x = drone_global
    """

    DEBUG_TOGGLE_: bool

    def __init__(self):
        super().__init__("localization_node")

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )

        # Subscribe to the AR Tags.
        self.artag_loc_sub_ = self.create_subscription(ARTags, "/drone_sensing/downward_ar_tags", self.on_receive_tag_info, 10)
        self.odometry_sub_ = self.create_subscription(VehicleOdometry, "/fmu/out/vehicle_odometry", self.on_receive_odometry, 10)

        # Boolean 
        self.tag_seen_pub_ = self.create_publisher(Bool, "/drone_sensing/tag_seen", 10)
        self.location_pub_ = self.create_publisher(Point, "/drone_sensing/local_position", 10)
        # Publisher for the destination of challenge 1.
        self.intended_dest_pub_ = self.create_publisher(Point, "/drone_control/ch1_intend_dest", 10)
        self.found_dest_pub_ = self.create_publisher(Bool, "/drone_control/ch1_found_dest", 10)

        # Debugging toggle.
        self.DEBUG_TOGGLE_ = False
        self.hasTrans = False

        self.get_logger().info("Successfully setting up the localization module!")


    def on_receive_odometry(self, msg: VehicleOdometry):
        self.cur_x = msg.position[0]
        self.cur_y = msg.position[1]
        self.cur_z = msg.position[2]

        if self.hasTrans:
            pt = Point()
            pt.x = self.cur_x + self.trans_x
            pt.y = self.cur_y + self.trans_y
            pt.z = self.cur_z + self.trans_z

            pt.x *= -1.0
            pt.y *= -1.0
            pt.z *= -1.0

            self.location_pub_.publish(pt)
        else:
            pt = Point()
            pt.x = pt.y = 0.0
            pt.z = -2.0

            self.location_pub_.publish(pt)


    def on_receive_tag_info(self, msg: ARTags):
        
        msg_bool = Bool()
        msg_bool.data = False

        msg_bool2 = Bool()

        if self.hasTrans:
            msg_tar = Point()

            msg_tar.x = -self.trans_x
            msg_tar.y = -self.trans_y

            self.intended_dest_pub_.publish(msg_tar)

            msg_bool2.data = True
        else:
            msg_bool2.data = False
        
        self.found_dest_pub_.publish(msg_bool2)


        # Check if the information is valid.
        if msg.count == 0:
            self.tag_seen_pub_.publish(msg_bool)
            return
        
        if msg.ids[0] not in AR_TAG_MAPPING_.keys():
            self.tag_seen_pub_.publish(msg_bool)
            return
        
        msg_bool.data = True

        self.get_logger().info("Received tag info.")

        # Process the input data.
        objp = get_locations(msg.ids[0])
        tag_corners = np.array(msg.locs).reshape(-1, 4, 2).astype(dtype=np.float32)
        tag_corners = tag_corners[0]

        ret, rvec, tvec = cv.solvePnP(objp, tag_corners, CALIBRATION_MATRIX_, DIST_MATRIX_)

        rot_matrix = cv.Rodrigues(rvec)[0].T
        translation = -rot_matrix @ tvec
        translation = [round(i[0], 2) for i in translation]

        # x, y, z = objp[0, 0]+translation[0, 0], objp[0, 1]+translation[0, 1], objp[0, 2]+translation[0, 2] 
        x, y, z = AR_TAG_MAPPING_[msg.ids[0]]
        x *= UNIT_LENGTH_
        y *= UNIT_LENGTH_
        z *= UNIT_LENGTH_
        x += translation[0]
        y += translation[1]
        z += translation[2]

        self.get_logger().info(f"Current (x, y, z): {x}, {y}, {z}")

        # self.get_logger().info(f"x: {x}, y: {y}, z: {z}")

        msg_point = Point()
        msg_point.x, msg_point.y, msg_point.z = x, y, z

        self.trans_x = x - self.cur_x
        self.trans_y = y - self.cur_y
        self.trans_z = z - self.cur_z

        self.hasTrans = True

        self.get_logger().info(f"Current trans infromation: {self.trans_x}, {self.trans_y}, {self.trans_z}")

        self.tag_seen_pub_.publish(msg_bool)
        self.location_pub_.publish(msg_point)


def main(args=None):
    rclpy.init(args=args)

    node = LocalizationNode()
    rclpy.spin(node)

    rclpy.shutdown()
        

if __name__ == "__main__":
    main()
