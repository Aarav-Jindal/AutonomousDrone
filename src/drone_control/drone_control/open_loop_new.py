#!usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from px4_msgs.msg import OffboardControlMode, VehicleCommand, TrajectorySetpoint

import time

from geometry_msgs.msg import Point

# Note: This is just a simple implementation of a node that arms the simulation drone (no timers, no switching between arming/disarming, etc)

class OpenLoop(Node):
    def __init__(self):
        super().__init__("OpenLoop")
        self.offboard_publisher_ = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", 10)
        self.arm_publisher_ = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.trajectory_publisher_ = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)

        self.create_subscription(Point, 'target_position', self.set_movement_point, 10)
        self.counter = 0
        self.target_position = [0.0, 0.0, -10.0]
        self.get_logger().info(f"{self.target_position}")
        
        # Publisher to the central commander.
        self.controller_cmd_pub_ = self.create_publisher(TrajectorySetpoint, "/drone_control/position_command", 10)

        # time.sleep(1)
        self.timer_ = self.create_timer(0.01, self.main_func)


    def set_movement_point(self, msg):
        self.get_logger().info(f"getting target position from topic \n")
        self.target_position = [msg.x, msg.y, msg.z]
        self.get_logger().info(f"getting target position from topic \n")


    def main_func(self):
        # if self.counter == 150:
        if self.counter == 101:
            self.send_mode_command()
            self.send_arm_command()
            # self.send_takeoff_command()
        self.ocm()
        # self.send_arm_command() #Code to call send_arm_command every few seconds
            
        
        if self.counter < 101:
            self.counter += 1
        print(self.counter)
        

    def ocm(self):
        # offboard_msg = OffboardControlMode()
        # offboard_msg.position = True
        # offboard_msg.velocity = False
        # offboard_msg.acceleration = False
        # offboard_msg.attitude = False
        # offboard_msg.body_rate = False
        # offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)

        trajectory_msg = TrajectorySetpoint()
        # trajectory_msg.position = [0.0, 0.0, -2.0]
        trajectory_msg.position = self.target_position
        self.get_logger().info(f"{self.target_position}")
        self.get_logger().info(f"setting position to x:{self.target_position[0]}, y:{self.target_position[1]}, z:{self.target_position[2]}")
        # trajectory_msg.velocity = [0.0, 0.0, float("nan")]
        trajectory_msg.yaw = 0.0
        trajectory_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        # self.trajectory_publisher_.publish(trajectory_msg)
        # self.offboard_publisher_.publish(offboard_msg)
        
        self.controller_cmd_pub_.publish(trajectory_msg)

        print("publishing offboard control mode")
        

    def send_mode_command(self):
        mode_msg = VehicleCommand()
        mode_msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        mode_msg.param1 = 1.0
        mode_msg.param2 = 6.0
        #1,1 is manual, 1,2 is altitude, 1,3 is position
        # 1,4 is also mission, 1,5 is acro, 1,6 is offboard
        #2,2 is armed,2,4 is armed
        # Idk if we need the rest of this stuff

        mode_msg.target_system = 1
        mode_msg.target_component = 1
        mode_msg.source_system = 1
        mode_msg.source_component = 1
        mode_msg.from_external = True
        mode_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.arm_publisher_.publish(mode_msg)
        self.get_logger().info("hh")
        

    def send_arm_command(self):
        # self.arm_toggle_ = not self.arm_toggle_ # Code to switch between arming/disarming nodes 
    
        arm_msg = VehicleCommand()
        arm_msg.param1 = 1.0
        arm_msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        arm_msg.target_system  = 1
        arm_msg.target_component = 1
        arm_msg.source_system = 1
        arm_msg.source_component = 1
        arm_msg.from_external = True
        arm_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.arm_publisher_.publish(arm_msg)

        print("Arm toggle message sent")
        
    
    def send_takeoff_command(self):
        # self.arm_toggle_ = not self.arm_toggle_ # Code to switch between arming/disarming nodes 
    
        msg = VehicleCommand()
        msg.param1 = 0.0
        msg.param7 = 2.0
        msg.command = VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF
        msg.target_system  = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        # msg.from_external = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.arm_publisher_.publish(msg)

        print("Arm toggle message sent")


def main(args=None):
    rclpy.init(args=args)
    node = OpenLoop()
    # node.send_arm_command() # Sends the command once
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
