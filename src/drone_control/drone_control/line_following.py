import rclpy
import numpy as np
import yaml
from dataclasses import dataclass
from rclpy.node import Node
from rclpy.clock import Clock
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
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

class LineFollowingNode(Node):
    def __init__(self):
        super().__init__("line_following_node")

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )

        self.timer = self.create_timer(0.2, self.line_callback)

        self.publisher = self.create_publisher(Point, "/target_position", qos_profile)
        self.target_reached_subscriber = self.create_subscription(Bool, self.target_reached_callback, qos_profile)

        self.up = False
        self.target_reached = False

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

    def target_reached_callback(self):
        self.target_reached = True

    def set_position(self, pos):
        msg = Point()
        msg.x, msg.y, msg.z = pos
        self.publisher.publish(msg)

    def at_position(self):
        if self.target_reached:
            self.target_reached = False
            return True

        return False

    def line_callback(self):
        if self.i == len(self.xs) or self.current_pos is None:
            self.get_logger().info("Path finished")
            return

        self.get_logger().info(f"{len(self.xs) - self.i} points left")

        # Arm and takeoff
        self.get_logger().info(f"Current position: {self.current_pos}")
        if not self.up:
            self.set_position([0.0, 0.0, self.zs[0]])
            if self.at_position():
                self.up = True

        if self.at_position():
            self.i += 1

        next_pt = [self.xs[self.i], self.ys[self.i], self.zs[self.i]]
        self.set_position(next_pt)

def main(args=None):
    rclpy.init(args=args)
    node = LineFollowingNode()
    rclpy.spin(node)
    rclpy.shutdown()
