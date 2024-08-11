import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from drone_msgs.msg import LineDetection
from cv_bridge import CvBridge, CvBridgeError

class LineDetectionNode(Node):
    def __init__(self):
        super().__init__("line_detection")

        self.bridge = CvBridge()
        self.subscriber = self.create_subscription(Image, "/image_raw", self.image_callback, 10)
        self.publisher = self.create_publisher(LineDetection, "/line_detect_regression", 10)

    def get_largest_contour(self, contours, min_area=500):
        """
        Finds the largest contour with size greater than min_area.

        Args:
            contours: A list of contours found in an image.
            min_area: The smallest contour to consider (in number of pixels)

        Returns:
            The largest contour from the list, or None if no contour was larger than min_area.
        """
        if len(contours) == 0:
            return None

        greatest_contour = None
        greatest_contour_area = 0
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < min_area:
                continue

            if area > greatest_contour_area:
                greatest_contour_area = area
                greatest_contour = contour


        return greatest_contour

    def linear_regression(self, contour):
        """
        Fits a regression line to a given contour.

        Args:
            contour: The OpenCV contour to fit a regression line to.

        Returns:
            The values (a, b) in the regression equation y = a + bx.
        """
        # y = a + bx

        # Slice by columns to get x and y coordinates
        x = contour[:, 0, 0]
        y = contour[:, 0, 1]

        xy = np.multiply(x, y)
        xsq = x ** 2

        xavg = np.mean(x)
        yavg = np.mean(y)
        xyavg = np.mean(xy)
        xsqavg = np.mean(xsq)

        num = xavg * yavg - xyavg
        den = xavg ** 2 - xsqavg

        b = float("nan") if np.isclose(den, 0) else num / den
        a = yavg - b * xavg

        return a, b

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(str(e))

        kernel = np.ones((5, 5), np.uint8)
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Dilate image to combine LEDs into a single line
        dilated = cv2.dilate(gray_image, kernel, iterations=1)

        # Threshold to get brightest parts of image
        _, thresholded = cv2.threshold(dilated, 240, 255, cv2.THRESH_BINARY)

        # Get contours
        contours, _ = cv2.findContours(thresholded, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        largest_contour = self.get_largest_contour(contours)

        msg = LineDetection()

        if largest_contour is None:
            msg.a = 0.0
            msg.b = 0.0
            msg.line_found = False
        else:
            # Fit regression line to largest contour
            a, b = self.linear_regression(largest_contour)
            msg.a = a
            msg.b = b
            msg.line_found = True

        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = LineDetectionNode()
    rclpy.spin(node)
    rclpy.shutdown()
