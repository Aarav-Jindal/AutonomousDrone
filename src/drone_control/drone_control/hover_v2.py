import rclpy
from rclpy.node import Node
from px4_msgs.msg import TrajectorySetpoint, VehicleCommand
import time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy


class HoverNode(Node):

    def __init__(self):
        super().__init__('hover_node')

        qos_profile_publisher = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history = QoSHistoryPolicy.KEEP_LAST,
            depth=0
        )

        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            qos_profile_publisher
        )

        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            qos_profile_publisher
        )

        self.timer = self.create_timer(0.4, self.timer_callback)  # Timer callback every 0.4 seconds (2.5 Hz)

        self.set_offboard_mode()
        self.arm_vehicle()

    def arm_vehicle(self):
        vehicle_command = VehicleCommand()
        vehicle_command.timestamp = int(time.time() * 1e6)  # Timestamp in microseconds
        vehicle_command.param1 = 1.0  # Arm
        vehicle_command.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        vehicle_command.target_system = 1
        vehicle_command.target_component = 1
        vehicle_command.source_system = 1
        vehicle_command.source_component = 1
        vehicle_command.from_external = True

        self.vehicle_command_pub.publish(vehicle_command)
        self.get_logger().info('Sent arm command')

    def set_offboard_mode(self):
        vehicle_command = VehicleCommand()
        vehicle_command.timestamp = int(time.time() * 1e6)  # Timestamp in microseconds
        vehicle_command.param1 = 1.0  # Set to offboard
        vehicle_command.param2 = 6.0  # Main mode: PX4_CUSTOM_MAIN_MODE_OFFBOARD
        vehicle_command.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        vehicle_command.target_system = 1
        vehicle_command.target_component = 1
        vehicle_command.source_system = 1
        vehicle_command.source_component = 1
        vehicle_command.from_external = True

        self.vehicle_command_pub.publish(vehicle_command)
        self.get_logger().info('Sent offboard mode command')

    def timer_callback(self):
        # Create and publish a TrajectorySetpoint message
        trajectory_setpoint = TrajectorySetpoint()
        trajectory_setpoint.timestamp = int(time.time() * 1e6)  # Timestamp in microseconds
        trajectory_setpoint.position = [0.0, 0.0, -2.0]
        # trajectory_setpoint.x = 0.0
        # trajectory_setpoint.y = 0.0
        # trajectory_setpoint.z = -2.0  # Hover 2 meters above ground
        trajectory_setpoint.yaw = 0.0

        self.trajectory_setpoint_pub.publish(trajectory_setpoint)

        # Set offboard mode continuously
        self.set_offboard_mode()
        self.arm_vehicle()


        self.get_logger().info('Published hover setpoint and set offboard mode')


def main(args=None):
    rclpy.init(args=args)
    hover_node = HoverNode()
    rclpy.spin(hover_node)
    hover_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
