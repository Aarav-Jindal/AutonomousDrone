#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from px4_msgs.msg import SensorOpticalFlow

class OpticalFlowSubscriber(Node):
    def __init__(self):
        super().__init__("optical_flow_subscriber")

        # TODO: Find correct optical flow sensor topic
        self.subscription = self.create_subscription(SensorOpticalFlow, "/image_raw", self.optical_flow_callback, 10)

    def optical_flow_callback(self, msg):
        delta_angle = msg.delta_angle
        self.get_logger().info(str(delta_angle))

def main(args=None):
    rclpy.init(args=args)
    node = OpticalFlowSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()
