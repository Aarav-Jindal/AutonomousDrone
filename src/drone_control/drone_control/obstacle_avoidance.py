#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from px4_msgs.msg import VehicleCommand, VehicleStatus, VehicleOdometry
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

# List of states.
IDLE_ = 0
IDLE_TAKEOFF = 1
TAKEOFF = 2
TAKEOFF_WORKING = 3
WORKING = 4
WORKING_LANDING = 5
LANDING = 6

PI = 3.1415926535

# Path that needs to be followed.
path = [
    [0.0, 0.0],
    [0.2, 0.0],
    [0.2, 1.0]
]

class ObstacleAvoidanceControl(Node):
    """
    The essential controller node for the obstacle avoidance task.
    """    
    state: int
    cur: int

    x: float
    y: float
    height: float

    isArmed: bool
    
    isBlocked: bool
    """
    If the drone is blocked by an obstacle. 
    """
    suggested_direction: int
    """
    0 up, 1 right, 2 down, 3 left.
    """

    CRUISE_HEIGHT_: float
    HEIGHT_TOLERANCE_: float

    def __init__(self):
        super().__init__("obstacle_avoidance")

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )

        # Creates a timer and callback.
        self.create_timer(0.05, self.timer_callback)

        # Vehicle status subscribers from the drone.
        self.vehicle_status_sub_ = self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status", self.vehicle_status_callback, qos_profile)
        self.vehicle_odometry_sub_ = self.create_subscription(VehicleOdometry, "/fmu/out/vehicle_odometry", self.odometry_callback, qos_profile)

        # Vehicle command publisher for the drone.
        self.vehicle_command_pub_ = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", qos_profile)
        self.offboard_control_mode_pub_ = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", qos_profile)
        self.motion_cmd_pub_ = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos_profile)

        self.state = 0
        self.cur = -1
        self.isArmed = False

        self.height = 0.0

        self.CRUISE_HEIGHT_ = -10.0
        self.HEIGHT_TOLERANCE_ = 0.5

        self.MAX_HORIZONTAL_DRIFT_ = 0.5


    def timer_callback(self):

        msg = TrajectorySetpoint()
        msg.timestamp = self.get_timestamp()
        msg.yaw = float("nan")
        msg.velocity = [float("nan") for _ in range(3)]
        msg.acceleration = [float("nan") for _ in range(3)]

        self.get_logger().info(f"Current state: {self.state}, height: {-self.height} meters.")

        match self.state:
            # Get ready for takeoff.
            case 0:
                self.set_offboard_mode()
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)

                # First we need to arm our drone.
                if self.isArmed:
                    self.state = TAKEOFF
                else:
                    self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
                    self.state = IDLE_TAKEOFF
            
            # Listen to the callback from the drone, waiting for takeoff.
            case 1:
                self.set_offboard_mode()

                # Wait until the takeoff.
                if self.isArmed:
                    self.state = TAKEOFF

            # Actual takeoff.
            case 2:
                self.set_offboard_mode()
                
                msg.position = [0.0, 0.0, self.CRUISE_HEIGHT_]
                self.motion_cmd_pub_.publish(msg)

                self.state = TAKEOFF_WORKING
            
            # Wait until it reaches the target height.
            case 3:
                self.set_offboard_mode()

                # Keep sending this command to make sure it's working.
                msg.position = [0.0, 0.0, self.CRUISE_HEIGHT_]
                self.motion_cmd_pub_.publish(msg)

                if abs(self.height - self.CRUISE_HEIGHT_) <= self.HEIGHT_TOLERANCE_:
                    self.state = WORKING

            # Working.
            case 4:
                self.set_offboard_mode()

                # If nothing's blocking: move forward directly.
                
                # msg.yaw = 0.0
                # msg.position = [0.0, 0.0, self.CRUISE_HEIGHT_]

                # self.motion_cmd_pub_.publish(msg)

            case _:
                pass
    

    def vehicle_status_callback(self, msg: VehicleStatus):
        """
        Callback for the vehicle status.
        """
        self.isArmed = msg.arming_state


    def odometry_callback(self, msg: VehicleOdometry):
        """
        Callback for the vehicle odometry.
        """
        self.x = msg.position[0]
        self.y = msg.position[1]
        self.height = msg.position[2]



    
    def get_timestamp(self) -> int:
        """
        Getting the timestamp in microseconds from the controller.
        """
        return self.get_clock().now().nanoseconds // 1000


    def publish_vehicle_command(self, command, param1=0.0, param2=0.0, param7=0.0):
        """
        Publishes command to /fmu/in/vehicle_command.
        """
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
        msg.timestamp = self.get_timestamp() # time in microseconds
        self.vehicle_command_pub_.publish(msg)

    
    def set_offboard_mode(self):
        """
        Set the drone in the offboard mode.
        """
        msg = OffboardControlMode()
        msg.timestamp = self.get_timestamp()

        # Currently is controlling the position instead of using PID controller.
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False

        self.offboard_control_mode_pub_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceControl()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
