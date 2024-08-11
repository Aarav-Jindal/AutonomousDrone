import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand
from drone_msgs.srv import TakeOff  # Replace with the custom service once created
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy


class takeOff(Node):

    def __init__(self):
        super().__init__('drone_control_node')

        qos_profile_publisher = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history = QoSHistoryPolicy.KEEP_LAST,
            depth=0
        )

        qos_profile_subscriber = QoSProfile(
            durability = QoSDurabilityPolicy.VOLATILE,
            reliability = QoSReliabilityPolicy.BEST_EFFORT,
            history = QoSHistoryPolicy.KEEP_LAST,
            depth = 10,

        )
        
        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, '/VehicleCommand_PubSubTopic', qos_profile_publisher)
        self.create_service(TakeOff, 'set_mode', self.set_mode_callback)
        self.get_logger().info("Drone Control Node Initialized and Service Ready.")

    def set_mode_callback(self, request, response):
        if request.takeoff:  # takeoff
            self.get_logger().info("Received request to takeoff.")
            self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param7=2.0)
            response.success = True
        else:  # land
            self.get_logger().info("Received request to land.")
            self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            response.success = True
        return response

    def send_vehicle_command(self, command, param1=0.0, param2=0.0, param3=0.0, param4=0.0, param5=0.0, param6=0.0, param7=0.0):
        msg = VehicleCommand()
        msg.timestamp = self.get_clock().now().to_msg().sec
        msg.param1 = param1
        msg.param2 = param2
        msg.param3 = param3
        msg.param4 = param4
        msg.param5 = param5
        msg.param6 = param6
        msg.param7 = param7
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_command_publisher.publish(msg)
        self.get_logger().info(f"Sent command {command}")

def main(args=None):
    rclpy.init(args=args)
    node = takeOff()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
