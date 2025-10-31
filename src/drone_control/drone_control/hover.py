# hover.py
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy

from px4_msgs.msg import (
    VehicleOdometry,
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
)

# ----------------------------------------------------------------------
# Helpers
# ----------------------------------------------------------------------

def now_us() -> int:
    """Wall-clock time in microseconds (stable for PX4 uXRCE timestamps)."""
    return int(time.time_ns() // 1000)

# ----------------------------------------------------------------------
# Node
# ----------------------------------------------------------------------

class Hover(Node):
    """
    Minimal OFFBOARD hover:
    - Streams OffboardControlMode + TrajectorySetpoint at 20 Hz
    - Arms after ~1 s of streaming
    - Requests OFFBOARD after arming
    - Holds (x,y,z) = (0, 0, -target_height) in NED (up is negative z)
    """

    def __init__(self):
        super().__init__('height_control_node')

        # ----- User params -----
        self.target_height = 1.0          # meters UP (NED => z = -1.0)
        self.height_tolerance = 0.1       # not strictly needed; here for future PID logic

        # ----- State -----
        self.current_height = None         # meters UP (converted from NED z)
        self.setpoint_count = 0
        self.armed = False
        self.offboard_mode_set = False

        # ----- QoS -----
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

        # ----- I/O -----
        self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self._odom_cb,
            sub_qos,
        )
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', pub_qos
        )
        self.traj_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', pub_qos
        )
        self.cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', pub_qos
        )

        # 20 Hz (PX4 requires > 2 Hz)
        self.timer = self.create_timer(0.05, self._loop)

        self.get_logger().info('Hover node started (target: 1.0 m up; OFFBOARD via uXRCE-DDS).')

    # ------------------------------------------------------------------
    # Callbacks / loop
    # ------------------------------------------------------------------

    def _odom_cb(self, msg: VehicleOdometry):
        # NED: position[2] is "down" (+). Convert to "up" meters.
        self.current_height = -float(msg.position[2])

    def _loop(self):
        # 1) Stream OffboardControlMode (position control)
        ocm = OffboardControlMode()
        ocm.timestamp = now_us()
        ocm.position = True
        ocm.velocity = False
        ocm.acceleration = False
        ocm.attitude = False
        ocm.body_rate = False
        self.offboard_pub.publish(ocm)

        # 2) Stream TrajectorySetpoint: hover at (0,0,-target_height) in NED
        ts = TrajectorySetpoint()
        ts.timestamp = now_us()
        ts.position = (0.0, 0.0, -float(self.target_height))
        ts.yaw = 0.0  # real float; avoid NaN
        self.traj_pub.publish(ts)

        # 3) After some streaming, ARM
        self.setpoint_count += 1
        if not self.armed and self.setpoint_count > 20:
            self._arm()
            self.armed = True

        # 4) After arming, request OFFBOARD
        if self.armed and not self.offboard_mode_set and self.setpoint_count > 30:
            self._set_offboard()
            self.offboard_mode_set = True

        # (Optional) simple status log when odom is available
        if self.current_height is not None:
            err = self.target_height - self.current_height
            if abs(err) > self.height_tolerance:
                self.get_logger().info(
                    f'[hover] current={self.current_height:.2f} m, target={self.target_height:.2f} m'
                )

    # ------------------------------------------------------------------
    # Commands
    # ------------------------------------------------------------------

    def _send_vehicle_command(
        self,
        command: int,
        *,
        p1: float = 0.0,
        p2: float = 0.0,
        p3: float = 0.0,
        p4: float = 0.0,
        p5: float = 0.0,
        p6: float = 0.0,
        p7: float = 0.0,
    ):
        vc = VehicleCommand()
        vc.timestamp = now_us()
        vc.param1 = float(p1)
        vc.param2 = float(p2)
        vc.param3 = float(p3)
        vc.param4 = float(p4)
        vc.param5 = float(p5)
        vc.param6 = float(p6)
        vc.param7 = float(p7)
        vc.command = int(command)
        vc.target_system = 1
        vc.target_component = 1
        vc.source_system = 1
        vc.source_component = 1
        vc.from_external = True
        self.cmd_pub.publish(vc)

    def _arm(self):
        # VEHICLE_CMD_COMPONENT_ARM_DISARM (400), param1=1 → arm
        self._send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, p1=1.0)
        self.get_logger().info('ARM sent')

    def _set_offboard(self):
        # MAV_CMD_DO_SET_MODE (176)
        # base_mode = 1 (MAV_MODE_FLAG_CUSTOM_MODE_ENABLED), custom_main_mode = 6 (OFFBOARD)
        self._send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, p1=1.0, p2=6.0
        )
        self.get_logger().info('OFFBOARD requested')

# ----------------------------------------------------------------------
# Main
# ----------------------------------------------------------------------

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
