#!/usr/bin/env python3
import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import Float32

class AltitudePublisherNode(Node):
    def __init__(self):
        super().__init__("altitude_publisher")

        self.publisher = self.create_publisher(Float32, "/altitude_reading", 10)
        self.timer = self.create_timer(0.001, self.timer_callback)
        port = "/dev/ttyUSB0"
        self.serial = serial.Serial(port, timeout=None, baudrate=115200, bytesize=8)

    def read_byte(self):
        self.serial.write(b"T")
        self.serial.flush()
        return self.serial.read()

    def publish_invalid(self):
        msg = Float32()
        msg.data = -1.0
        self.publisher.publish(msg)

    def timer_callback(self):
        self.serial.write(b"T")
        self.serial.flush()
        self.serial.reset_input_buffer()

        line = b""
        while (data := self.read_byte()) != b"\n":
            line += data

        if not line.endswith(b"\r") or not 4 <= len(line) <= 6:
            # Format error
            self.publish_invalid()
            return

        digits = line[:-1]
        if not digits.isalnum():
            # Format error
            self.publish_invalid()
            return

        distance = int(digits)
        if distance == -1:
            self.get_logger().error("Invalid reading from sensor")

        msg = Float32()
        msg.data = distance / 1000
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = AltitudePublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()
