# Roomba ROS2 Package

This package provides ROS2 nodes to interface with a Roomba robot and a Kinect sensor. It includes functionalities for controlling the Roomba and processing depth and RGB data from the Kinect.

## Nodes

- `kinect_node`: Captures and processes depth and RGB data from the Kinect sensor.

## Requirements

- `ROS2 Humble or later`
- `libfreenect`
- `rclpy`
- `camera_info_manager_py`
- `cv_bridge`

## Install Dependencies

```bash
sudo apt update
sudo apt install -y ros-$ROS_DISTRO-cv-bridge ros-$ROS_DISTRO-camera-info-manager-py
```

### Install libfreenect and Python Wrappers

Follow the instructions in the [kinect_node/README.md](./kinect_node/README.md) file to install `libfreenect` and its Python wrappers.

## Build

After installing the dependencies, build the package as follows:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/samuelhrqe/roomba_ros2.git
cd ~/ros2_ws
rosdep install --from-paths src --rosdistro $ROS_DISTRO --ignore-src -r -y
colcon build --package-select roomba_ros2
source install/setup.bash
```

> After `rosdep install` command:
>
> `roomba_ros2: Cannot locate rosdep definition for [libfreenect]` will appear. You can ignore this message.

## Run

To run the Kinect node, use the following command:

```bash
ros2 run roomba_ros2 kinect_node
```
