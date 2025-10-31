# ROS 2 Drone Workspace

**Tested on:** Ubuntu 22.04 + ROS 2 Humble + PX4 v1.14

This repository integrates PX4 with ROS 2 for autonomous drone control in both SITL (Gazebo) and hardware setups.

**All things under AutonomousDrone/src/drone_control are my work**


## 1. Prerequisites

```shell
sudo apt update
sudo apt install -y git python3-colcon-common-extensions python3-vcstool \
  build-essential cmake python3-pip

# Optional (recommended tools)
sudo apt install -y ros-humble-rqt* ros-humble-rviz2 ros-humble-gazebo-ros-pkgs
```

---

## 2. Clone this Repository

```shell
git clone --recursive https://github.com/Aarav-Jindal/AutonomousDrone.git
cd AutonomousDrone
```

---

## 3. Bring in PX4 ROS 2 Interfaces (If they are missing)

```shell
git submodule add https://github.com/PX4/px4_msgs.git src/px4_msgs
git submodule add https://github.com/PX4/px4_ros_com.git src/px4_ros_com
git submodule update --init --recursive
```

---

## 4. Resolve Dependencies and Build

```shell
# rosdep (first time only)
sudo rosdep init 2>/dev/null || true
rosdep update

# Install dependencies for all packages
rosdep install --from-paths src --ignore-src -y --rosdistro humble

# Build workspace
colcon build --symlink-install

# Source setup (add to ~/.bashrc for convenience)
source install/setup.bash
echo "source $(pwd)/install/setup.bash" >> ~/.bashrc
```

---

## 5. Connect PX4 ↔ ROS 2

### A) Real Drone (over UDP)

On companion computer or laptop:

```shell
source install/setup.bash
ros2 run px4_ros_com micrortps_agent -t UDP
```

---

### B) SITL (Gazebo)

1. Launch PX4 SITL with RTPS enabled (per PX4 documentation).  
2. Run the agent in a new terminal:

```shell
source install/setup.bash
ros2 run px4_ros_com micrortps_agent -t UDP
```

The `micrortps_agent` exposes:

- `/fmu/in/*` → PX4 input topics  
- `/fmu/out/*` → PX4 output topics  

These are used by your control and sensing nodes.

---

## 6. Nodes Overview

### drone_control

| Node | Publishes | Subscribes | Description |
|------|------------|-------------|--------------|
| **arm_disarm_node** | `/fmu/in/vehicle_command` (`px4_msgs/msg/VehicleCommand`) | `/arm_disarm` (`drone_msgs/srv/ArmDisarm`) | Arms/disarms the drone via service request |
| **hover_node** | `/fmu/in/vehicle_command` (`px4_msgs/msg/VehicleCommand`), `/fmu/in/trajectory_setpoint` (`px4_msgs/msg/TrajectorySetpoint`) | `/fmu/out/vehicle_local_position` (`px4_msgs/msg/VehicleLocalPosition`) | Hovers at a target altitude using offboard control |
| **ocean_motion** | `/fmu/in/trajectory_setpoint` | `/fmu/out/vehicle_odometry` | Follows a smooth spline trajectory from `path.yaml` |
| **spin_2_win** | `/fmu/in/trajectory_setpoint` | `/fmu/out/vehicle_odometry` | Rotational yaw demo |
| **line_following** | `/fmu/in/trajectory_setpoint` | `/image_raw`, `/fmu/out/vehicle_odometry` | Uses camera input for line tracking |
| **obstacle_avoidance** | `/fmu/in/trajectory_setpoint` | `/scan`, `/fmu/out/vehicle_odometry` | Avoids obstacles using sensor data |

---

## 7. Example Runs

```shell
# Hover demo
ros2 run drone_control hover

# Ocean motion (requires path.yaml)
ros2 run drone_control ocean_motion --ros-args -p file:=path.yaml -p path_num:="1"

# Spin-in-place demo
ros2 run drone_control spin_2_win
```

---

## 8. Project Structure

```
src/
├── drone_control/         # Flight control nodes
├── drone_sensing/         # Vision, LiDAR, and localization nodes
├── px4_msgs/              # PX4 ROS 2 message definitions
└── px4_ros_com/           # PX4 communication bridge (RTPS)
```

---

## 9. Highlights

- Native ROS 2 ↔ PX4 offboard control  
- Works with Gazebo (gz-sim) or real hardware  
- Demonstrates autonomy, control, and perception integration  

---

## 11. Acknowledgments

- **PX4 Autopilot:** [https://github.com/PX4/PX4-Autopilot](https://github.com/PX4/PX4-Autopilot)  
- **PX4 ROS 2 Interface:** [https://github.com/PX4/px4_ros_com](https://github.com/PX4/px4_ros_com)  
- **Gazebo Simulation Environment:** [https://gazebosim.org](https://gazebosim.org)  

## Other software

### V4L2 ROS 2 node

```shell
# Install:
sudo apt install ros-humble-v4l2-camera

# Run camera node on camera with id [ID]:
# (Run `ls /dev/v4l/by-id` to find IDs of connected cameras)
ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:="/dev/v4l/by-id/[ID]"
```

### OAK D Lite Camera

Refer to the [official documentations](https://docs.luxonis.com/hardware/platform/deploy/usb-deployment-guide/) for detailed instructions.

To install the depthai and its dependencies:

```shell
sudo wget -qO- https://docs.luxonis.com/install_depthai.sh | bash
```

To install the Depthai Viewer to test the connection:

```shell
python3 -m pip install depthai-viewer
```

Run this application to make sure the stereo camera is connected correctly.

```shell
python3 -m depthai_viewer
```

## Collaborators
Katrina (https://github.com/Katrina-1)
Victor Chen (https://github.com/Goldenglow1427)
Aidan Sun (https://github.com/AidanSun05)
William Frabizio (https://github.com/memebot50000)

_Saved Copy of Original Repository, which was archived._
