#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import cv2 as cv

from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from drone_msgs.msg import ARTags

from cv_bridge import CvBridge

class ARTagsDetectNode(Node):
    """
    This is the node written to detect the AR Tags from the information given in a node.

    Subscriptions
    ------------------
    /drone_sensing/stereo_camera_raw: The RGB image stored in Image msg.


    Publishers
    -------------------

    """

    DICTIONARY_: cv.aruco.Dictionary
    """
    This is the OpenCV dictionary that stores all the AR Tags that should be detected in this node.

    Default value is cv.aruco.DICT_5X5_100
    """

    PARAMETERS_ = cv.aruco.DetectorParameters
    """
    This is the OpenCV parameter set to be applied to the AR Tags detector.

    Default value is cv.aruco.DetectorParameters()
    """

    DETECTOR_: cv.aruco.ArucoDetector
    """
    This is the OpenCV detector that's invoked every time to detect the AR tags in the image.
    """

    DEBUG_ON_LOG_: bool
    """
    Whether to show debug messages on the logger.

    Default value is False.
    """

    br_: CvBridge
    """
    The bridge used to transform ROS2 image message into the OpenCV image.
    """

    def __init__(self):
        super().__init__("camera_read_ar_tags")

        # qos_profile = QoSProfile(
        #     reliability=QoSReliabilityPolicy.BEST_EFFORT,
        #     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        #     depth=10
        # )

        # Define all the constants used in this node.
        self.DICTIONARY_ = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_5X5_100)
        self.PARAMETERS_ = cv.aruco.DetectorParameters()
        self.DETECTOR_ = cv.aruco.ArucoDetector(self.DICTIONARY_, self.PARAMETERS_)
        self.DEBUG_ON_LOG_ = True

        # Create a subscriber.
        self.create_subscription(Image, "/drone_sensing/stereo_camera_raw", self.receive_stereo_image_data, 10)
        self.create_subscription(Image, "/image_raw", self.receive_downward_image_data, 10)

        # Create two publishers: one for location and another for IDs.
        # self.stereo_id_publisher_ = self.create_publisher(Int32MultiArray, "/drone_sensing/stereo_ar_tag_IDs", 10)
        # self.stereo_loc_publisher_ = self.create_publisher(Int32MultiArray, "/drone_sensing/stereo_ar_tag_locs", 10)
        self.stereo_publisher_ = self.create_publisher(ARTags, "/drone_sensing/stereo_ar_tags", 10)

        # self.downward_id_publisher_ = self.create_publisher(Int32MultiArray, "/drone_sensing/downward_ar_tag_IDs", 10)
        # self.downward_loc_publisher_ = self.create_publisher(Int32MultiArray, "/drone_sensing/downward_ar_tag_locs", 10)
        self.downward_publisher_ = self.create_publisher(ARTags, "/drone_sensing/downward_ar_tags", 10)

        self.debug_publisher_ = self.create_publisher(Image, "/drone_sensing/debug_image_output", 10)

        # Create the bridge for exporting images.
        self.br_ = CvBridge()

        self.get_logger().info("Successfully launched the AR Tag detection module!")


    def publish_marker_id(self, publisher, markerId):
        msg = Int32MultiArray()

        if markerId is None:
            publisher.publish(msg)
        else:
            # markerId: np.ndarray
            for element in markerId.tolist():
                msg.data.append(element[0])
            publisher.publish(msg)


    def publish_marker_loc(self, publisher, markerLoc):
        msg = Int32MultiArray()

        if markerLoc is None or len(markerLoc) == 0:
            publisher.publish(msg)
        else:
            for marker in markerLoc:
                for value in marker.flatten().tolist():
                    msg.data.append(int(value))
            publisher.publish(msg)

    def receive_stereo_image_data(self, msg: Image):
        # cv.imshow("Frame", self.br.imgmsg_to_cv2(msg))
        img = self.br_.imgmsg_to_cv2(msg)

        img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        ret, img = cv.threshold(img, 140, 255, cv.THRESH_BINARY)

        markerCorners, markerIDs, rejectedCandidates = self.DETECTOR_.detectMarkers(img)

        if self.DEBUG_ON_LOG_:
            self.get_logger().info(f"Obtained corners: {markerCorners}")
            self.get_logger().info(f"Detected AR Tag Label: {markerIDs}")

        tagMsg = ARTags()

        if markerIDs is not None:
            tagMsg.count = len(markerIDs)
            for element in markerIDs.tolist():
                tagMsg.ids.append(element[0])
            for marker in markerCorners:
                for value in marker.flatten().tolist():
                    tagMsg.locs.append(int(value))
        else:
            tagMsg.count = 0

        self.stereo_publisher_.publish(tagMsg)

        # self.publish_marker_id(self.stereo_id_publisher_, markerIDs)
        # self.publish_marker_loc(self.stereo_loc_publisher_, markerCorners)

    def draw_circle(self, color_image, center, color=(0, 255, 255), radius=6):
        """
        Draws a circle on the provided image.

        Args:
            color_image: The color image on which to draw the contour.
            center: The pixel (row, column) of the center of the image.
            color: The color to draw the circle in BGR format.
            radius: The radius of the circle in pixels.
        """
        # cv.circle expects the center in (column, row) format
        cv.circle(color_image, (center[1], center[0]), radius, color, -1)

    # TODO: change the downward facing code as well.
    def receive_downward_image_data(self, msg: Image):
        # cv.imshow("Frame", self.br.imgmsg_to_cv2(msg))
        img = self.br_.imgmsg_to_cv2(msg)

        img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        ret, img = cv.threshold(img, 150, 255, cv.THRESH_BINARY)

        # self.debug_publisher_.publish(self.br_.cv2_to_imgmsg(img))

        markerCorners, markerIDs, rejectedCandidates = self.DETECTOR_.detectMarkers(img)

        if self.DEBUG_ON_LOG_:
            self.get_logger().info(f"Obtained corners: {markerCorners}")
            self.get_logger().info(f"Detected AR Tag Label: {markerIDs}")
        
        # if markerIDs is not None:
        #     for marker in markerCorners:
        #         for id, value in zip(range(4), marker.flatten().reshape(-1, 2)):
        #             if id == 2:
        #                 self.draw_circle(img, (int(value[1]), int(value[0])), color=(0, 0, 0))

        self.debug_publisher_.publish(self.br_.cv2_to_imgmsg(img))

        tagMsg = ARTags()

        if markerIDs is not None:
            tagMsg.count = len(markerIDs)
            for element in markerIDs.tolist():
                tagMsg.ids.append(element[0])
            for marker in markerCorners:
                for value in marker.flatten().tolist():
                    tagMsg.locs.append(int(value))
        else:
            tagMsg.count = 0

        self.downward_publisher_.publish(tagMsg)

def main(args=None):
    rclpy.init(args=args)

    # Create a node
    node = ARTagsDetectNode()
    rclpy.spin(node)

    # Shutdown the rclpy
    rclpy.shutdown()


if __name__ == "__main__":
    main()
