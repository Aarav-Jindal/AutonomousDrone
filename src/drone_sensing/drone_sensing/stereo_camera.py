#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import cv2 as cv
import depthai as dai

from sensor_msgs.msg import Image

# OpenCV to ROS2
from cv_bridge import CvBridge

class ReadStereoCameraNode(Node):
    """
    This is the node created to read the RGB image passed by the stereo camera.

    Publishers
    -------------------------
    /drone_sensing/stereo_camera_raw: The RGB image stored in Image msg.
    """

    INTERVAL_: float
    """
    The time interval (in seconds) between sending each frame of the data.

    Default value is 0.0333333 (30 fps).
    """

    FRAME_PIXEL_: int
    """
    The size of the image read and send from the camera (in FRAME_PIXEL_ * FRAME_PIXEL_).

    Default value is 1000.
    """

    br_: CvBridge
    """
    The bridge used to transform OpenCV image into the ROS2 image message.
    """

    def __init__(self) -> None:
        """
        Initialize the node and set up the publishers and cameras.
        """

        # Constants for reading from camera.
        self.INTERVAL_ = 0.033333333
        self.FRAME_PIXEL_ = 1000

        super().__init__("camera_read") # Initialize the node titled "camera_read"

        # Create publishers
        self.image_publisher_ = self.create_publisher(Image, "/drone_sensing/stereo_camera_raw", 10)

        # Publish the information at a certain time interval
        self.create_timer(self.INTERVAL_, self.send_image_data)
        self.get_logger().info("Stereo camera sending node connected!")

        # Creates a bridge between opencv and ros2
        self.br_ = CvBridge()

        # Create pipelines for depthai.
        self.pipeline = dai.Pipeline()

        self.camRgb = self.pipeline.create(dai.node.ColorCamera)
        self.xoutRgb = self.pipeline.create(dai.node.XLinkOut)

        self.xoutRgb.setStreamName("rgb")

        # Setting the properties
        self.camRgb.setPreviewSize(self.FRAME_PIXEL_, self.FRAME_PIXEL_)
        self.camRgb.setInterleaved(False)
        self.camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)

        # Linking
        self.camRgb.preview.link(self.xoutRgb.input)

        self.device = dai.Device(self.pipeline)
        self.qRgb = self.device.getOutputQueue(name="rgb", maxSize=4, blocking=False)

        self.get_logger().info("Successfully connected to the stereo camera!")

    def send_image_data(self) -> None:
        """
        Captures the image received from the stereo camera at the moment, and send it to the corresponding topic.
        """
        # Obtain the data from the RGB camera
        inRgb = self.qRgb.get()

        # Read the frame information from the data
        frame = inRgb.getCvFrame()

        # Publish the frame as Image message to the topic.
        self.image_publisher_.publish(self.br_.cv2_to_imgmsg(frame))


def main(args=None):

    rclpy.init()

    # Create a new node.
    node = ReadStereoCameraNode()
    rclpy.spin(node)

    # Shutdown the rclpy and device.
    rclpy.shutdown()



if __name__ == "__main__":
    main()
