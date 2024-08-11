#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from px4_msgs.msg import VehicleCommand, VehicleOdometry
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

class ControllerInputManager(Node):
    """
    This node is receiving all the controller inputs, and determine the best way to perform the actions.
    
    Main controller: the core path and mission.
    Obstacle controller: the controller to react when see the obstacle.
    Emergency reacter: when emergency, land.
    """
    
    STATE_IDLE: int = 0
    
    main_active: bool
    STATE_MAIN: int = 1

    obstacle_active: bool
    STATE_OBSTACLE: int = 2

    emergency_reaction: bool
    STATE_EMERGENCY: int = -1

    HEIGHT_BOUND: float = -5.0
    
    state: int
    mode_state: int
    
    MODE_POSITION: int = 1
    MODE_SPEED: int = 2
    
    target_position: TrajectorySetpoint
    avoidance_action: TrajectorySetpoint

    x: float
    y: float
    z: float

    def __init__(self):
        """
        Initialize the node.
        """
        super().__init__("main_controller")

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        
        self.main_active = False
        self.obstacle_active = False
        self.emergency_reaction = False
        self.state = self.STATE_IDLE

        # Sending the information to the drone.
        self.create_timer(0.1, self.timer_callback)
        
        # Creates a subscriber to the command.
        self.create_subscription(TrajectorySetpoint, "/cmd/target_position", self.on_receive_main, 10)
        self.create_subscription(VehicleOdometry, "/fmu/out/vehicle_odometry", self.on_receive_odometry, qos_profile)
        
        # For publishing the information.
        self.trajectory_pub_ = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", 10)
        self.offboard_pub_ = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", 10)
        
        self.get_logger().info("Finish setting up the controller!")


    def timer_callback(self):
        self.set_offboard_mode()
        
        if self.state == self.STATE_IDLE:
            return
        
        if self.state == self.STATE_MAIN:
            self.trajectory_pub_.publish(self.target_position)

        if self.state == self.STATE_EMERGENCY:
            msg_traj = TrajectorySetpoint()
            msg_traj.timestamp = self.get_timestamp()
            msg_traj.yaw = float("nan")
            msg_traj.velocity = msg_traj.acceleration = [float("nan") for _ in range(3)]

            msg_traj.position = [float("nan"), float("nan"), 0.0]
            self.trajectory_pub_.publish(msg_traj)
    
    
    def on_receive_main(self, msg: TrajectorySetpoint):
        self.target_position = msg
        
        if self.state == self.STATE_IDLE:
            self.state = self.STATE_MAIN

    
    def on_receive_odometry(self, msg: VehicleOdometry):
        self.x = msg.position[0]
        self.y = msg.position[1]
        self.z = msg.position[2]

        # Check if height exceeded.
        if self.z < self.HEIGHT_BOUND:
            self.state = self.STATE_EMERGENCY
            self.get_logger().warn("Height limit exceeded - forcing to land.")






    def get_timestamp(self):
        return self.get_clock().now().nanoseconds // 1000


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

        self.offboard_pub_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ControllerInputManager()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
