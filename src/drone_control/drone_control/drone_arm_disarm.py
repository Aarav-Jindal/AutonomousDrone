#!usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from px4_msgs.msg import VehicleCommand
from drone_msgs.srv import ArmDisarm

class ArmingNode(Node):
    def __init__(self):
        # the below contains the name of the node in "name"
        super().__init__("arm_disarm_node")
        self.publisher = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", 10)
        self.srv = self.create_service(ArmDisarm, 'arm_disarm', self.callback)

    def callback(self, request, response):
        if request.should_arm:
            self.arm()
        else:
            self.disarm()

        response.new_state = request.should_arm
        return response

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0, param7=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.param7 = param7    # altitude value in takeoff command
        msg.command = command  # command ID
        msg.target_system = 1  # system which should execute the command
        msg.target_component = 1  # component which should execute the command, 0 for all components
        msg.source_system = 1  # system sending the command
        msg.source_component = 1  # component sending the command
        msg.from_external = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.publisher.publish(msg)

    def arm(self):
        # self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("Arm command sent")

    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.get_logger().info("Disarm command sent")

def main(args=None):
    rclpy.init(args=args)
    arm_disarm_node = ArmingNode()
    rclpy.spin(arm_disarm_node)
    rclpy.shutdown()