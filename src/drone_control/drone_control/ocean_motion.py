import rclpy
import numpy as np
import yaml
from dataclasses import dataclass
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from px4_msgs.msg import TrajectorySetpoint, OffboardControlMode, VehicleCommand, VehicleOdometry

@dataclass
class Spline:
    c: list[float]
    s: list[float]
    d: list[float]

# Tridiagonal matrix solver
def solve(a, b, c, d, unknowns):
    n = unknowns - 1
    c_temp = c[:]
    res = d[:]

    c_temp[0] /= b[0]
    res[0] /= b[0]

    for i in range(1, n):
        c_temp[i] /= b[i] - a[i] * c_temp[i - 1]
        res[i] = (res[i] - a[i] * res[i - 1]) / (b[i] - a[i] * c_temp[i - 1])

    res[n] = (res[n] - a[n] * res[n - 1]) / (b[n] - a[n] * c_temp[n - 1])

    for i in range(n, 0, -1):
        res[i - 1] -= c_temp[i - 1] * res[i]

    return res

# Express the list of points as a piecewise parametric function
def get_cubic_splines(points):
    # Cubic spline interpolation
    # https://www.math.ntnu.no/emner/TMA4215/2008h/cubicsplines.pdf

    n = len(points) - 1
    h = [points[i + 1][0] - points[i][0] for i in range(n)]
    b = [(points[i + 1][1] - points[i][1]) / h[i] for i in range(n)]
    v = [2 * (h[i - 1] + h[i]) for i in range(1, n)]
    u = [6 * (b[i] - b[i - 1]) for i in range(1, n)]

    z = solve(h, v, h, u, len(points) - 2)

    splines = []
    for i in range(n):
        zi = 0 if i == 0 else z[i - 1]
        zi1 = 0 if i == n - 1 else z[i]
        hi = h[i]
        xi, yi = points[i]
        xi1, yi1 = points[i + 1]

        coeffs = [zi1 / (6 * hi), zi / (6 * hi), (yi1 / hi) - hi * (zi1 / 6), (yi / hi) - zi * (hi / 6)]
        shifts = [xi, xi1, xi, xi1]

        splines.append(Spline(coeffs, shifts, [xi, xi1]))

    return splines

def interpolate_points(points, dt):
    x_points = [[i, points[i][0]] for i in range(len(points))]
    y_points = [[i, points[i][1]] for i in range(len(points))]
    z_points = [[i, points[i][2]] for i in range(len(points))]

    x_splines = get_cubic_splines(x_points)
    y_splines = get_cubic_splines(y_points)
    z_splines = get_cubic_splines(z_points)

    xs = []
    ys = []
    zs = []
    mult = 1 / dt
    for xsp, ysp, zsp in zip(x_splines, y_splines, z_splines):
        for i in range(int(xsp.d[0] * mult), int(xsp.d[1] * mult)):
            t = i * dt
            fn = lambda x, sp: sp.c[0] * (x - sp.s[0]) ** 3 \
                    + sp.c[1] * (sp.s[1] - x) ** 3 \
                    + sp.c[2] * (x - sp.s[2]) \
                    + sp.c[3] * (sp.s[3] - x)

            xs.append(fn(t, xsp))
            ys.append(fn(t, ysp))
            zs.append(fn(t, zsp))

    xs.append(points[-1][0])
    ys.append(points[-1][1])
    zs.append(points[-1][2])

    return xs, ys, zs

class ocean_motion(Node):
    def __init__(self):
        super().__init__("ocean_motion")

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )

        self.timer = self.create_timer(0.2, self.line_callback)
        self.odometry_subscriber = self.create_subscription(VehicleOdometry, "/fmu/out/vehicle_odometry", self.odometry_callback, qos_profile)

        self.control_publisher = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos_profile)
        self.offboard_control_mode_publisher = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", qos_profile)
        self.publisher = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", qos_profile)

        self.armed = False
        self.up = False

        self.current_pos = None
        self.initial_pos = None

        self.declare_parameter("file", "path.yaml")
        self.declare_parameter("path_num", "1")
        file = self.get_parameter("file").get_parameter_value().string_value
        path_num = self.get_parameter("path_num").get_parameter_value().string_value
        with open(file) as f:
            y = yaml.load(f, Loader=yaml.FullLoader)
            points = [[float(d["x"]), float(d["y"]), -float(d["z"])] for d in y["paths"][path_num]]

        # Interpolate path to get a smoother trajectory
        self.xs, self.ys, self.zs = interpolate_points(points, 0.2)
        self.i = 0

    def set_offboard_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000  # PX4 timestamp in microseconds
        msg.position = True
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.offboard_control_mode_publisher.publish(msg)

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
        self.publisher.publish(msg)

    def set_position(self, pos, vel):
        msg = TrajectorySetpoint()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.yaw = float("nan")
        msg.velocity = vel
        msg.acceleration = [float("nan") for _ in range(3)]
        msg.position = np.array(pos, dtype=np.float32) + self.initial_pos
        msg.position[2] = msg.position[2] + np.sin(msg.timestamp)
        self.get_logger().info("SINE WAVING...")

        self.set_offboard_mode()
        self.control_publisher.publish(msg)

    def line_callback(self):
        if self.i == len(self.xs) or self.current_pos is None:
            self.get_logger().info("Path finished")
            return

        self.get_logger().info(f"{len(self.xs) - self.i} points left")

        # Arm and takeoff
        self.get_logger().info(f"Current position: {self.current_pos}")
        if self.current_pos[2] > self.zs[0] and not self.armed:
            self.set_position([0.0, 0.0, self.zs[0]], [float("nan") for _ in range(3)])
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            self.armed = True

            return

        next_pt = [self.xs[self.i], self.ys[self.i], self.zs[self.i]]
        d = np.linalg.norm(self.current_pos - next_pt)
        self.get_logger().info(f"Distance to next point: {d}")
        if d < 0.4:
            self.i += 1

        v = [0.0 if np.isclose(self.current_pos[i], next_pt[i]) else 0.1 for i in range(3)]
        self.set_position(next_pt, v)

    def odometry_callback(self, msg):
        if self.initial_pos is None:
            self.initial_pos = np.array(msg.position, dtype=np.float32)

        new_pos = np.array(msg.position, dtype=np.float32)
        self.current_pos = new_pos - self.initial_pos

def main(args=None):
    rclpy.init(args=args)
    node = ocean_motion()
    rclpy.spin(node)
    rclpy.shutdown()