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

PX4 communicates with ROS 2 through a **DDS bridge**.  
For PX4 v1.14+ (recommended), use the **Micro XRCE-DDS Agent**; older versions use the legacy `micrortps_agent`.

---

### A) Install PX4-Autopilot and Micro XRCE-DDS Agent

```shell
# Clone PX4-Autopilot
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot

# Install PX4 dependencies
bash ./Tools/setup/ubuntu.sh

# Build PX4 SITL with Gazebo
make px4_sitl gz_x500
```

Then install the **Micro XRCE-DDS Agent** (needed for PX4 ↔ ROS 2 communication):

```shell
sudo apt install libasio-dev libtinyxml2-dev
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build && cd build
cmake ..
make
sudo make install
sudo ldconfig
```

Verify installation:

```shell
MicroXRCEAgent --version
```

---

### B) Run PX4 SITL + ROS 2 Bridge

Open three terminals (in this order):

#### 1️⃣ Start the Micro XRCE-DDS Agent
```shell
pkill -f MicroXRCEAgent || true
MicroXRCEAgent udp4 -p 8888 -v 6
```

#### 2️⃣ Launch PX4 SITL (Gazebo)
```shell
cd ~/PX4-Autopilot
make px4_sitl gz_x500
```

Expected log:
```
INFO  [uxrce_dds_client] init UDP agent IP:127.0.0.1, port:8888
INFO  [uxrce_dds_client] time sync converged
```

#### 3️⃣ Run a ROS 2 node
```shell
cd ~/Desktop/AutonomousDrone
source install/setup.bash
ros2 run drone_control hover
```

---

### C) Install QGroundControl

**QGroundControl (QGC)** is used for monitoring, telemetry, and parameter tuning.

```shell
cd ~
wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage
chmod +x QGroundControl.AppImage
./QGroundControl.AppImage
```

It auto-connects to PX4 SITL on UDP port `14550`.

---

### D) Verify PX4 ↔ ROS 2 Connection

```shell
ros2 topic list | grep fmu
```

Expected topics:

```
/fmu/in/offboard_control_mode
/fmu/in/trajectory_setpoint
/fmu/in/vehicle_command
/fmu/out/vehicle_odometry
/fmu/out/vehicle_status
```

If you see these, PX4 and ROS 2 are successfully connected.

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

## 9. Acknowledgments

- **PX4 Autopilot** — Open-source flight control stack for drones  
  [https://github.com/PX4/PX4-Autopilot](https://github.com/PX4/PX4-Autopilot)
- **ROS 2 Humble Hawksbill** — Middleware for robotic systems  
  [https://docs.ros.org/en/humble](https://docs.ros.org/en/humble)
- **eProsima Micro XRCE-DDS Agent** — Lightweight DDS communication layer  
  [https://github.com/eProsima/Micro-XRCE-DDS-Agent](https://github.com/eProsima/Micro-XRCE-DDS-Agent)
- **Gazebo (gz-sim)** — Open robotics simulator  
  [https://gazebosim.org](https://gazebosim.org)
- **QGroundControl** — MAVLink-based ground control software  
  [https://qgroundcontrol.com](https://qgroundcontrol.com)

  
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
