# Kinect Node

## Requirements

- libusb >= 1.0.18 (Windows needs >= 1.0.22)
- CMake >= 3.12.4
- python >= 2.7 or >= 3.3 (only if BUILD_PYTHON=ON or BUILD_PYTHON2=ON or BUILD_PYTHON3=ON or BUILD_REDIST_PACKAGE=OFF)
- numpy (for python bindings)
- setuptools (for python bindings)
- cython <= 0.29.37 (for python bindings)

## Installation

### Linux (I use Ubuntu 22.04)

```bash
sudo apt-get install git cmake build-essential libusb-1.0-0-dev

# only if you are building the examples:
sudo apt-get install freeglut3-dev libxmu-dev libxi-dev
```

## Fetch & Build

```bash
git clone https://github.com/OpenKinect/libfreenect
cd libfreenect
mkdir build && cd build
cmake .. -DBUILD_REDIST_PACKAGE=OFF -DBUILD_PYTHON3=ON
make
sudo make install
sudo ldconfig
```

If you want to use the kinect as a non-root user then do the following

```bash
sudo adduser $USER video
sudo adduser $USER plugdev
```

Create a file named `/etc/udev/rules.d/51-kinect.rules` with the following content:

```bash
# ATTR{product}=="Xbox NUI Motor"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02b0", MODE="0666"
# ATTR{product}=="Xbox NUI Audio"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02ad", MODE="0666"
# ATTR{product}=="Xbox NUI Camera"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02ae", MODE="0666"

# Kinect for Windows
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02c2", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02be", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02bf", MODE="0666"
```

Then reload the udev rules:

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Unplug and replug your Kinect device.

## Build Python Wrappers

### Clean previous build artifacts (if any)

```bash
cd ../wrappers/python
rm -rf build freenect.c freenect.cpp freenect.*.so *.egg-info
```

### Install dependencies and build

```bash
sudo apt update
sudo apt install -y python3-dev python3-numpy python3-setuptools
python3 -m pip install --upgrade pip
python3 -m pip install "cython<3"

export CYTHONIZE=1
python3 setup.py build_ext --inplace
python3 -m pip install . --no-build-isolation
```

### Test the installation

```bash
python3 -c "import freenect, sys; print('OK', freenect.__file__, sys.version)"
```

Should print something like:

```bash
OK /home/$USER/.local/lib/python3.10/site-packages/freenect.cpython-310-x86_64-linux-gnu.so 3.10.12 (main, Aug 15 2025, 14:32:43) [GCC 11.4.0]
```

Now you can run the kinect_node ROS2 node to interface with the Kinect sensor.
