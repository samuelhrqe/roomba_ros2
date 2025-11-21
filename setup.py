from setuptools import find_packages, setup

package_name = 'roomba_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config/kinect_calibration', ['config/kinect_calibration/calibration_depth.yaml', 'config/kinect_calibration/calibration_rgb.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Samuel Henrique',
    maintainer_email='samuelhenriq12@gmail.com',
    description='Final project for ROS2 and Roomba integration',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'kinect_node = kinect_node.kinect:main',
            'yolo_node = yolo_node.yolo_node:main',
        ],
    },
)
