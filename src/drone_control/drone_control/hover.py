# hover.py
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from px4_msgs.msg import VehicleOdometry, OffboardControlMode, TrajectorySetpoint, VehicleCommand

def now_us() -> int:
    return int(time.time_ns() // 1000)

class Hover(Node):
    def __init__(self):
        super().__init__('height_control_node')

        # Targets (NED: up is negative z)
        self.target_height = 1.0          # meters up
        self.takeoff_vel   = -0.8         # m/s in NED (negative = up)
        self.takeoff_ticks = 60           # ~3s at 20 Hz of velocity climb before position hold

        # State
        self.current_height_up = None     # meters up (derived from odom z)
        self.setpoint_count = 0
        self.armed = False
        self.offboard_set = False
        self.have_odom = False

        # QoS
        pub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        sub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # I/O
        self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self._odom_cb, sub_qos)
        self.offboard_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', pub_qos)
        self.traj_pub     = self.create_publisher(TrajectorySetpoint,    '/fmu/in/trajectory_setpoint',   pub_qos)
        self.cmd_pub      = self.create_publisher(VehicleCommand,        '/fmu/in/vehicle_command',       pub_qos)

        # 20 Hz loop
        self.timer = self.create_timer(0.05, self._loop)

        self.get_logger().info('Hover node: velocity takeoff -> position hold (OFFBOARD).')

    def _odom_cb(self, msg: VehicleOdometry):
        # NED z is "down" (+); convert to "up"
        self.current_height_up = -float(msg.position[2])
        self.have_odom = True

    def _loop(self):
        # 1) Always stream OffboardControlMode
        ocm = OffboardControlMode()
        ocm.timestamp = now_us()
        ocm.position = True
        ocm.velocity = True   # we use velocity during takeoff, then position
        ocm.acceleration = False
        ocm.attitude = False
        ocm.body_rate = False
        self.offboard_pub.publish(ocm)

        # 2) Choose setpoint: initial velocity up, then position hold
        ts = TrajectorySetpoint()
        ts.timestamp = now_us()
        if self.setpoint_count < self.takeoff_ticks:
            # Velocity climb for a short phase
            ts.velocity = (0.0, 0.0, float(self.takeoff_vel))  # NED: negative is up
        else:
            # Position hover at target height
            ts.position = (0.0, 0.0, -float(self.target_height))  # NED: up is negative
        ts.yaw = 0.0
        self.traj_pub.publish(ts)

        self.setpoint_count += 1

        # 3) Only arm once we have odometry and after a short warmup
        if self.have_odom and not self.armed and self.setpoint_count > 20:
            self._arm()
            self.armed = True

        # 4) Request OFFBOARD a moment after arming
        if self.armed and not self.offboard_set and self.setpoint_count > 30:
            self._set_offboard()
            self.offboard_set = True

        # (Optional) log
        if self.current_height_up is not None and self.setpoint_count % 20 == 0:
            self.get_logger().info(f'h={self.current_height_up:.2f} m, phase={"VEL" if self.setpoint_count<self.takeoff_ticks else "POS"}')

    # -------- commands ----------
    def _send_cmd(self, command: int, *, p1=0.0, p2=0.0, p3=0.0, p4=0.0, p5=0.0, p6=0.0, p7=0.0):
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

    def _arm(self):
        self._send_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, p1=1.0)
        self.get_logger().info('ARM sent')

    def _set_offboard(self):
        # base_mode = 1 (custom), custom_main_mode = 6 (OFFBOARD)
        self._send_cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, p1=1.0, p2=6.0)
        self.get_logger().info('OFFBOARD requested')

def main(args=None):
    rclpy.init(args=args)
    node = Hover()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
