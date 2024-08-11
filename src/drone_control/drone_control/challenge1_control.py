#importing ros2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

#importing ros2 topics
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleOdometry
from px4_msgs.msg import VehicleStatus
from geometry_msgs.msg import Point
from std_msgs.msg import Bool

#immporting services
from drone_msgs.srv import ArmDisarm 

#importing miscelanious
import time

class Challenge1MainController(Node):
    def __init__(self):
        super().__init__('chal1_main_control_node')

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

        self.location_publisher = self.create_publisher(Point, 'target_position', 10)
        

        self.landing_publisher = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile_publisher)
        
        self.create_subscription(Bool, '/tag_seen', self.detecting_callback, qos_profile_subscriber)
        self.create_subscription(Point, '/target_location', self.get_destination, qos_profile_subscriber)
        self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.get_current_position, qos_profile_subscriber)
        self.create_subscription(VehicleStatus, 'fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile_subscriber)


        #Client Setup
        # self.client = self.create_client(ArmDisarm, "arm_disarm")

        # while not self.client.wait_for_service(timeout_sec=5.0):
        #     self.get_logger().info('ArmDisarm service not available, waiting for service...')
        # self.request = ArmDisarm.Request()

        
        self.timer = self.create_timer(0.01, self.main_controller)

        self.status = [
            "takeoff",
            "detected",
            "going_to_location",
            "landing"
        ]
        self.status_index = 0

        self.is_armed = False
        self.offboardMode = False
        self.nav_status = 1
        self.target_location = [None, None, None]
        self.current_position = [0.0, 0.0, 0.0]
        self.height = -2.0 #should always be negative
        self.hover_height = [0.0, 0.0, self.height]

    #Checking whether drone is armed and in offboard mode
    def vehicle_status_callback(self, msg):
        self.get_logger().info(f'Vehicle Armed: {msg.arming_state}') #1 = diarmed, 2 = armed
        self.get_logger().info(f'Vehical Status: {msg.nav_state}') #14 = offbaord

        if msg.arming_state == 1:
            self.is_armed = False
        if not msg.nav_state == 14:
            self.offboardMode = False
        
        self.nav_status = msg.nav_state

        # time.sleep(1)
    
    #Getting the target destination from the drone_sensing nodes
    def get_destination(self, msg):
        self.target_location = [msg.x, msg.y, msg.z]

    #Getting the drones current position
    def get_current_position(self, msg):
        self.current_position = [msg.position[0], msg.position[1], msg.position[2]]
    
    #Function to make the drone hover
    def hover(self):
        msg = Point()
        msg.x = self.hover_height[0]
        msg.y = self.hover_height[1]
        msg.z = self.hover_height[2]
        self.location_publisher.publish(msg)

    #Dunction to make drone go to target location
    def gotolocation(self):
        msg = Point()
        msg.x = self.target_location[0]
        msg.y = self.target_location[1]
        msg.z = self.target_location[2]
        self.location_publisher.publish(msg)
        time.sleep(0.5)
        self.status_index = 3

    #callback to see if the drone has detected the AR tag
    def detecting_callback(self, msg):
        if msg.data:
            self.status_index = 2
        self.get_logger.info("tag detecting... calculating position to travel to")

    #method to arm the drone
    # def arm(self):
    #     self.request.should_arm = True
    #     self.future = self.client.call_async(self.request)

    #method to takeoff
    def takeoff(self):
        msg = Point()
        msg.x = 0.0
        msg.y = 0.0
        msg.z = self.height
        self.location_publisher.publish(msg)
        self.status_index = 1

    #method for landing the drone
    def land(self):
        msg = TrajectorySetpoint()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000  
        msg.position = [self.target_location[0], self.target_location[1], 0]  
        msg.yaw = float('nan')  # NaN to ignore yaw
        self.landing_publisher.publish(msg)
        
    #Main callback handling all the states of the drone during the challenge
    def main_controller(self):
        match self.status_index:
            case 0:
                self.get_logger().info("case 0")
                # while not self.is_armed and not self.offboardMode:
                #     self.get_logger().info("arming")
                #     self.arm()
                self.takeoff()
                self.get_logger().info("taking off")
            case 1:
                self.hover()
            
            case 2:
                self.gotolocation()
            case 3:
                self.land()
                

def main(args=None):
    rclpy.init(args=args)
    node = Challenge1MainController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
