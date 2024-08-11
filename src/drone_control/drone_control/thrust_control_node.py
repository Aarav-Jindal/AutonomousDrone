#Importing ros2 functionalities
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

#Importing Topics
from px4_msgs.msg import VehicleOdometry
from px4_msgs.msg import ActuatorMotors
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import TrajectorySetpoint
from geometry_msgs.msg import Point
from std_msgs.msg import Bool

#IMporting miscelanious
import time

#from PID_controller import PIDController

class PIDController:
    def __init__(self, kp, ki, kd, dt, integral_limit=float('inf')):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.integral_limit = integral_limit
        self.prev_error = 0
        self.integral = 0

    def compute(self, setpoint, measurement):
        error = setpoint - measurement
        self.integral += error * self.dt
        # Prevent integral windup
        self.integral = max(min(self.integral, self.integral_limit), -self.integral_limit)
        derivative = (error - self.prev_error) / self.dt
        self.prev_error = error
        cmd = self.kp * error + self.ki * self.integral + self.kd * derivative
        return cmd


class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        
        # Subscribers
        self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.odom_callback, qos_profile)
        self.create_subscription(VehicleStatus, 'fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)

        self.create_subscription(Point, '/target_position', self.get_target_location, qos_profile)
        self.create_subscription(Bool, "precision_mode", self.set_precise, qos_profile)
        
        # Publisher
        self.control_publisher = self.create_publisher(ActuatorMotors, '/fmu/in/actuator_motors', qos_profile)
        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", qos_profile)
        self.trajectory_setpoint_acceleration_publisher = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos_profile)
        self.target_reached_publisher = self.create_publisher(Bool, "target_reached", qos_profile)


        # Timer
        self.timer = self.create_timer(0.05, self.control_loop)

        #setting target position
        self.target_x = None
        self.target_y = None
        self.target_z = None
        # self.get_logger().info(f"target location is x: {self.target_x} y: {self.target_y} z: {self.target_z}\n")

        #Setting other variables
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.offboardMode = False
        self.is_armed = False
        self.prev_time = self.get_clock().now()
        self.offboard_setpoint_cnt = 0
        self.nav_status = 1
        self.precise_mode = False

        #arming drone
        self.state_offboard()
        self.arm()

        # PID Controllers
        # Optimal gain values and dt value according to Particle Swarm Optimization (dt = 0.11643779420722095): 
        # dt = (current_time-self.prev_time).nanoseconds // 10004
        dt = 0.05
        #self.get_logger().info(dt)
        current_time = self.get_clock().now()
        self.prev_time = current_time
        self.get_logger().info("Initializing pid controller for x: \n")
        self.y_pid = PIDController(kp=1.8962470407900696, ki=0.10687890725576543, kd=1.6502218555506412, dt=dt, integral_limit=1.0)
        self.get_logger().info("Initializing pid controller for y: \n")
        self.x_pid = PIDController(kp=1.8962470407900696, ki=0.10687890725576543, kd=1.6502218555506412, dt=dt, integral_limit=1.0)
        self.get_logger().info("Initializing pid controller for z: \n")
        self.z_pid = PIDController(kp=2.0, ki=2.0, kd=2.0, dt=dt, integral_limit=1.0)

    def get_target_location(self, msg):
        self.target_x = msg.x
        self.target_y = msg.y
        self.target_z = msg.z

    def set_precise(self, msg):
        self.precise_mode = msg.data

    #publishes command to /fmu/in/vehicle_command
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
        self.vehicle_command_publisher_.publish(msg)

    #Sets drone to offboard mode
    def state_offboard(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)
        self.get_logger().info("Setting to OFFBOARD mode!")
        self.offboardMode = True
    
    #sets drone to armed
    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("Arm command send")
        self.is_armed = True
    
    #recieves vehicle status values
    def vehicle_status_callback(self, msg):
        self.get_logger().info(f'Vehicle Armed: {msg.arming_state}') #1 = diarmed, 2 = armed
        self.get_logger().info(f'Vehical Status: {msg.nav_state}') #14 = offbaord

        if msg.arming_state == 1:
            self.is_armed = False
        if not msg.nav_state == 14:
            self.offboardMode = False
        
        self.nav_status = msg.nav_state

        # time.sleep(1)

    def odom_callback(self, msg):
        # Update current position
        self.current_x = msg.position[0]
        self.current_y = msg.position[1]
        self.current_z = msg.position[2]
        self.get_logger().info(f"Current position is x: {self.current_x} y: {self.current_y} z: {self.current_z}\n")

    def control_loop(self):
        self.get_logger().warn("Starting new iteration of control loop")
        self.get_logger().info(f"Drone is armed: {self.is_armed}")
        self.get_logger().info(f"Drone is in offboard: {self.offboardMode}")
        

        # if not self.is_armed:
        #     for i in range(10):
        #         self.arm()

        # if not self.offboardMode or not self.nav_status==14:
        #     for i in range(10): 
        #         self.state_offboard()
        if not self.target_x==None and not self.target_y==None and not self.target_z==None:
            self.get_logger().info(f"target location is x: {self.target_x} y: {self.target_y} z: {self.target_z}\n")
            if self.offboard_setpoint_cnt == 10:
                self.state_offboard()
            

            # Publish offboard control modes
            offboard_msg = OffboardControlMode()
            offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
            offboard_msg.position = False
            offboard_msg.velocity = False
            offboard_msg.acceleration = True
            # offboard_msg.direct_actuator = True
            # offboard_msg.thrust_and_torque = True
            offboard_msg.body_rate = False
            offboard_msg.attitude = False
            self.publisher_offboard_mode.publish(offboard_msg)  

            #Creating PID controllers for x, y and z direction
            z_acc = self.z_pid.compute(self.target_z, self.current_z)
            self.get_logger().info(f"computer z_acc to {z_acc}\n")

            x_acc = self.x_pid.compute(self.target_x, self.current_x)
            self.get_logger().info(f"computer x_Acc to {x_acc}\n")

            y_acc = self.y_pid.compute(self.target_y, self.current_y)
            self.get_logger().info(f"computer y_Acc to {y_acc}\n")
            

            # Create actuator motors message
            # control_msg = ActuatorMotors()
            
            # control_msg.timestamp = self.get_clock().now().nanoseconds // 1000 
            # control_msg.control[3] = 1 #temporary for testing for thrust.
            # control_msg.control[0] = roll  
            # control_msg.control[2] = pitch 
            # control_msg.control[1] = 1
            
            # self.get_logger().info(f"Message being published = {control_msg}\n\n")
            
            # # Publish control commands
            # self.control_publisher.publish(control_msg)

            # Create Trajectory Setpoint message
            control_msg = TrajectorySetpoint()

            control_msg.timestamp = self.get_clock().now().nanoseconds // 1000
            control_msg.acceleration = [x_acc, y_acc, z_acc]
            # control_msg.acceleration = [0.0, 0.0, -10.0] #For testing
            control_msg.yaw = float('nan')  # NaN to ignore yaw
            control_msg.position = [float("nan") for _ in range(3)]
            control_msg.velocity = [float("nan") for _ in range(3)]
            # control_msg.acceleration[0] = x_acc
            # control_msg.acceleration[1] = y_acc
            # control_msg.acceleration[2] = z_acc
            self.get_logger().info(f"Accerelation is set to: {control_msg.acceleration}")
            self.trajectory_setpoint_acceleration_publisher.publish(control_msg)
            self.get_logger().warn("MEssage has been published")

            self.offboard_setpoint_cnt +=1

            distance = ((self.current_x - self.target_x)**2 + (self.target_y - self.current_y)**2 + (self.current_z - self.target_z)**2)**0.5

            if self.precise_mode:
                if distance < 0.1:
                    msg = Bool()
                    msg.data = True
                    self.target_reached_publisher.publish(msg)
            else:
                if distance < 0.5:
                    msg = Bool()
                    msg.data = True
                    self.target_reached_publisher.publish(msg) 

def main(args=None):
    rclpy.init(args=args)
    print("initializing node")
    node = DroneController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
