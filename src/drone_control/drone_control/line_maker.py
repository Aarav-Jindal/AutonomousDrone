import rclpy
from rclpy.node import Node
from drone_msgs.msg import LineDetection

class TestNode(Node):
    def __init__(self):
        super().__init__("test_node")
        self.publisher = self.create_publisher(LineDetection, "/line_detect_regression", 10)
        self.timer = self.create_timer(0.5, self.callback)

        self.c = 0

    def callback(self):
        msg = LineDetection()
        msg.line_found = True
        msg.a = 0.0

        if self.c > 70:
            msg.b = -2.0
        elif self.c > 35:
            msg.b = 0.5
        else:
            msg.b = 1.0
        self.publisher.publish(msg)

        self.c += 1

def main(args=None):
    rclpy.init(args=args)
    node = TestNode()
    rclpy.spin(node)
    rclpy.shutdown()
