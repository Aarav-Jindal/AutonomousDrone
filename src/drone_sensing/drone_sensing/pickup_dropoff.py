import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from px4_msgs.msg import VehicleOdometry
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

class ColorDetectionNode(Node):
    def __init__(self):
        super().__init__('color_detection_node')
        
        # Create a subscription to the downward-facing camera image topic
        self.create_subscription(
            Image,
            '/image_raw',
            self.receive_downward_image_data,
            10
        )
        
        # Create a subscription to the vehicle odometry topic
        self.create_subscription(
            VehicleOdometry,
            'fmu/vehicle_odometry/out',
            self.update_current_position,
            10
        )
        
        # Create a publisher for the target position
        self.target_position_publisher = self.create_publisher(
            Point,
            'target_position',
            10
        )
        
        self.bridge = CvBridge()
        self.landing_position = None
        self.pickup_position = None
        
        self.current_position = Point()  # Current position of the drone
        
        # Define the initial square size and increment
        self.initial_square_size = 5.0  # initial size of the square in meters
        self.square_increment = 2.0     # increment in size after each iteration
        self.current_square_size = self.initial_square_size
        
        # Define the fixed square path corners
        self.update_path_corners()
        
        self.current_target_index = 0  # Index of the current target corner
        
        self.timer = self.create_timer(1.0, self.move_in_square_pattern)
        
    def receive_downward_image_data(self, msg):
        # Convert the ROS Image message to an OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Detect colors and save positions
        self.detect_and_save_position(cv_image)
        
    def detect_and_save_position(self, cv_image):
        # Convert the image to HSV color space
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # Define the HSV range for red color
        red_lower1 = np.array([0, 70, 50])
        red_upper1 = np.array([10, 255, 255])
        red_lower2 = np.array([170, 70, 50])
        red_upper2 = np.array([180, 255, 255])
        
        # Define the HSV range for yellow color
        yellow_lower = np.array([20, 100, 100])
        yellow_upper = np.array([30, 255, 255])
        
        # Create masks for red and yellow colors
        red_mask1 = cv2.inRange(hsv_image, red_lower1, red_upper1)
        red_mask2 = cv2.inRange(hsv_image, red_lower2, red_upper2)
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)
        yellow_mask = cv2.inRange(hsv_image, yellow_lower, yellow_upper)
        
        # Find contours for red and yellow colors
        red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        yellow_contours, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if red_contours:
            self.get_logger().info("Red color detected")
            if not self.landing_position:
                self.landing_position = Point(
                    x=self.current_position.x, 
                    y=self.current_position.y, 
                    z=self.current_position.z
                )
                self.get_logger().info(f"Landing position detected at {self.landing_position}")
        
        if yellow_contours:
            self.get_logger().info("Yellow color detected")
            if not self.pickup_position:
                self.pickup_position = Point(
                    x=self.current_position.x, 
                    y=self.current_position.y, 
                    z=self.current_position.z
                )
                self.get_logger().info(f"Pickup position detected at {self.pickup_position}")
        
        if not red_contours and not yellow_contours:
            self.get_logger().info("No red or yellow color detected")
    
    def update_current_position(self, msg):
        # Update the current position of the drone from the odometry message
        self.current_position.x = msg.position[0]
        self.current_position.y = msg.position[1]
        self.current_position.z = msg.position[2]
        
    def update_path_corners(self):
        # Update the path corners based on the current square size
        self.path_corners = [
            Point(x=0.0, y=0.0, z=0.0),
            Point(x=self.current_square_size, y=0.0, z=0.0),
            Point(x=self.current_square_size, y=self.current_square_size, z=0.0),
            Point(x=0.0, y=self.current_square_size, z=0.0)
        ]
        
    def move_in_square_pattern(self):
        # Check if both positions are already detected
        if self.landing_position and self.pickup_position:
            self.get_logger().info("Both positions detected. Calling movement function")
            self.final_movement()
            self.timer.cancel()
            return
        
        # Move to the next target corner in the square pattern
        target_position = self.path_corners[self.current_target_index]
        self.target_position_publisher.publish(target_position)
        self.get_logger().info(f"Moving to position {target_position}")
        
        # Update the target index for the next move
        self.current_target_index = (self.current_target_index + 1) % len(self.path_corners)
        
        # Check if a full square pattern has been completed
        if self.current_target_index == 0:
            if not self.landing_position or not self.pickup_position:
                # Increase the size of the square if both positions are not detected
                self.current_square_size += self.square_increment
                self.update_path_corners()
                self.get_logger().info(f"Increasing square size to {self.current_square_size} meters")

    def final_movement(self):
        for i in range(5):
            self.target_position_publisher.publish(self.pickup_position)
        time.sleep(1)
        msg = Point()
        msg.x = self.pickup_position.x
        msg.y = self.pickup_position.y
        msg.z = 0.0
        for i in range(5):
            self.target_position_publisher.publish(msg)
        time.sleep(7)
        for i in range(5):
            self.target_position_publisher.publish(self.pickup_position)
        time.sleep(1)
        for i in range(5):
            self.target_position_publisher.publish(self.landing_position)
        time.sleep(3)
        msg = Point()
        msg.x = self.landing_position.x
        msg.y = self.landing_position.y
        msg.z = 0.0
        for i in range(5):
            self.target_position_publisher.publish(msg)
        

def main(args=None):
    rclpy.init(args=args)
    node = ColorDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
