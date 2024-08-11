# ROS 2 Drone Workspace

## Clone repository

```shell
git clone --recursive https://github.com/AidanSun05/drone
```

`--recursive` is needed to pull in submodules (third-party ROS 2 packages).

## Nodes

### drone_control

- arm_disarm_node
  - Publishes to:
    - `/fmu/in/vehicle_command` (publishes commands to either arm or disarm the drone; type `px4_msgs.msg.VehicleCommand`)
  - Serves to:
    - `/arm_disarm` (receives information to determine the new state of the drone; type `drone_msgs.srv.ArmDisarm`)
- hover_node
  -Publishes to
    - `/fmu/in/vehicle_command` (publishes commands to either arm or disarm the drone; type `px4_msgs.msg.VehicleCommand`)
    - `/fmu/in/trajectory_setpoint` (publishes certain position/acceleration/velocity/yaw for the drone to recieve and apply; type `px4_msgs.msg.   TrajectorySetpoint`)
  - Subscribes to
    - `/fmu/out/vehicle_local_position` (reads to current position of the drone in the LNED frame); type `px4_msgs.msg.VehicleLocalPosition`

### drone_sensing

- line_detection
  - Subscribes to:
    - `/image_raw` (type `sensor_msgs.msg.Image`)
  - Publishes to:
    - `/line_detect_regression` (publishes [a, b] coefficients in the equation of the regressed line `y = a + bx` and if there is a detection; type `drone_msgs.msg.LineDetection`)
- optical_flow_subscriber
  - Subscribes to:
    - TODO: Find topic the Pixhawk publishes to (type px4_msgs.msg.SensorOpticalFlow)
  - Publishes to:
    - TODO: Figure out what to publish
- stereo_camera
  This node is implemented in `drone_sensing/drone_sensing/stereo_camera.py`;
  - Publishes to:
    - `/drone_sensing/stereo_camera_raw`:
      The node publishes the RGB image data received from the stereo camera to this topic in `sensor_msgs.msg.Image`;

      The resolution of the image is set as 1000*1000, and the frame rate is 30 images per second.
- ar_tag_detecter
  This node is implemented in `drone_sensing/drone_sensing/ar_tag_detection.py`;
  - Subscribes to:
    - `/drone_sensing/stereo_camera_raw`:
      The node receives the RGB image data from the stereo camera to identify the AR tags in the image.
  - Publishes to:
    - `/drone_sensing/ar_tag_IDs`
      The node publishes the ID of the AR tags detected (in particular order) to the topic using `std_msgs.msg.Int32MultiArray`;
    - `/drone_sensing/ar_tag_locs`
      The node publishes the locations of the AR tags detected (in the same order as the IDs) to the topic using `std_msgs.msg.Int32MultiArray`;

      Each AR tag is stored using consecutive 8 integers in the array, which shows $(x_1,y_1),(x_2,y_2),(x_3,y_3),(x_4,y_4)$, respectively.
- altitude_publisher
  - Publishes to:
    - `/altitude_reading` (publishes the distance in meters detected by the range finder; type `std_msgs.msg.Float32`)

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
