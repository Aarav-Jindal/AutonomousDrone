import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy

from px4_msgs.msg import VehicleOdometry, OffboardControlMode, TrajectorySetpoint, VehicleCommand

def now_us():
    return int(time.time_ns() // 1000)

class Hover(Node):
    def __init__(self):
        super().__init__('height_control_node')

        # NED frame: up is negative z → hover 1 m above takeoff
        self.target_height = 1.0  # meters UP
        self.height_tolerance = 0.1
        self.current_height = None

        # State flags/counters
        self.setpoint_count = 0
        self.armed = False
        self.offboard_mode_set = False

        # QoS: keep it simple & compatible with PX4
        pub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        sub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # I/O
        self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.odometry_callback, sub_qos)
        self.offboard_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', pub_qos)
        self.traj_pub     = self.create_publisher(TrajectorySetpoint,    '/fmu/in/trajectory_setpoint',   pub_qos)
        self.cmd_pub      = self.create_publisher(VehicleCommand,        '/fmu/in/vehicle_command',       pub_qos)

        # Run at 20 Hz (PX4 requires >2 Hz)
        self.timer = self.create_timer(0.05, self.control_loop)

    def odometry_callback(self, msg: VehicleOdometry):
        # NED: position[2] = z (down is +, up is -)
        self.current_height = -float(msg.position[2])

    # ---- Helpers -------------------------------------------------------------

    def send_vehicle_command(self, command, *, p1=0.0, p2=0.0, p3=0.0, p4=0.0, p5=0.0, p6=0.0, p7=0.0):
        vc = VehicleCommand()
        vc.timestamp = now_us()
        vc.param1 = float(p1); vc.param2 = float(p2); vc.param3 = float(p3); vc.param4 = float(p4)
        vc.param5 = float(p5); vc.param6 = float(p6); vc.param7 = float(p7)
        vc.command = int(command)
        vc.target_system = 1
        vc.target_component = 1
        vc.source_system = 1
        vc.source_component = 1
        vc.from_external = True
        self.cmd_pub.publish(vc)

    def arm(self):
        # VEHICLE_CMD_COMPONENT_ARM_DISARM (400), param1=1 → arm
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, p1=1.0)
        self.get_logger().info('ARM command sent')
        self.armed = True

    def set_offboard(self):
        # MAV_CMD_DO_SET_MODE (176): base_mode = 1 (CUSTOM), custom_main_mode = 6 (OFFBOARD)
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, p1=1.0, p2=6.0)
        self.get_logger().info('OFFBOARD mode requested')
        self.offboard_mode_set = True

    # ---- Main loop -----------------------------------------------------------

    def control_loop(self):
        # 1) Always stream OffboardControlMode
        ocm = OffboardControlMode()
        ocm.timestamp = now_us()
        ocm.position = True
        ocm.velocity = False
        ocm.acceleration = False
        ocm.attitude = False
        ocm.body_rate = False
        self.offboard_pub.publish(ocm)

        # 2) Always stream a position setpoint (NED: z = -height to go up)
        ts = TrajectorySetpoint()
        ts.timestamp = now_us()
        ts.position = (0.0, 0.0, -float(self.target_height))
        ts.yaw = 0.0  # set a real float; avoid NaN
        self.traj_pub.publish(ts)

        self.setpoint_count += 1

        # 3) After a few setpoints, ARM
        if not self.armed and self.setpoint_count > 20:
            self.arm()

        # 4) After arming, request OFFBOARD
        if self.armed and not self.offboard_mode_set and self.setpoint_count > 30:
            self.set_offboard()

        # 5) Optional: log height convergence (when odom is available)
        if self.current_height is not None:
            err = self.target_height - self.current_height
            if abs(err) > self.height_tolerance:
                self.get_logger().info(f'[height_control_node]: Adjusting height to {self.target_height:.1f} m.')
            else:
                self.get_logger().info('[height_control_node]: Height within tolerance.')

def main(args=None):
    rclpy.init(args=args)
    node = Hover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
