# Roomba ROS2 Package

This package provides ROS2 nodes to interface with a Roomba robot and a Kinect sensor. It includes functionalities for controlling the Roomba and processing depth and RGB data from the Kinect.

## Nodes

- `kinect_node`: Captures and processes depth and RGB data from the Kinect sensor.
- `yolo_node`: Captures and processes object detection with YOLO11

## Launch Files

- `roomba_camera_launch.py`: Execute the `rs_launch.py` file from `realsense_camera` node from `realsense-ros` package and the `yolo_node`.

## Requirements

- `ROS2 Humble or later`
- `libfreenect`
- `rclpy`
- `camera_info_manager_py`
- `cv_bridge`

## Install Dependencies

```bash
sudo apt update
sudo apt install -y \
    ros-$ROS_DISTRO-cv-bridge \
    ros-$ROS_DISTRO-camera-info-manager-py \
    ros-$ROS_DISTRO-rmw-cyclonedds-cpp
```

### Install libfreenect and Python Wrappers (If you use Kinect)

Follow the instructions in the [kinect_node/README.md](./kinect_node/README.md) file to install `libfreenect` and its Python wrappers.

### Use Intel® RealSense™ SDK 2.0 (If you use Intel® RealSense™ L515) with Docker

#### Setup the udev rules in your host

```sh
git clone --depth 1 https://github.com/IntelRealSense/librealsense.git -b v2.54.2
cd librealsense
./scripts/setup_udev_rules.sh
```

#### Clone this repository

```sh
git clone https://github.com/samuelhrqe/roomba_ros2.git
```

#### Using Docker

In the directory `./Docker/` are the `Dockerfile`, `docker-compose.yaml` and `docker_build.sh`

You can build the image with:

```sh
cd Docker/
./docker_build.sh
```

Or, you can pull the image (Recommended: faster):

```sh
docker pull samuelhrqe/ros2_realsense_x86_64:humble
```

Using Docker Compose, start the container:

```sh
docker compose up
```

> Use `docker compose up -d` to run in background and `docker compose logs -f` to view the logs

To close and remove the container:

```sh
docker compose down
```

The package use the Eclipse Cyclone DDS, so in your host do:

```sh
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

You can see the published topics with:

```sh
ros2 topic list
```

### Build Intel® RealSense™ SDK 2.0 manually


## Build

After installing the dependencies, build the package as follows:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/samuelhrqe/roomba_ros2.git
cd ~/ros2_ws
rosdep install --from-paths src --rosdistro $ROS_DISTRO --ignore-src --skip-keys=libfreenect --skip-keys=librealsense2 -r -y
colcon build --package-select roomba_ros2
source install/setup.bash
```

## Run

To run the Kinect node, use the following command:

```bash
ros2 run roomba_ros2 kinect_node
```
