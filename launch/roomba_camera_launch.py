from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    rs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py'
            ])
        ),
        launch_arguments={
            'camera_namespace': 'roomba',
            'camera_name': 'L515',
            'pointcloud.enable': 'true',
            'enable_gyro': 'true',
            'enable_accel': 'true',
            'enable_rgbd': 'true',
            'enable_sync': 'true',
            'enable_depth': 'true',
            'colorizer.enable': 'false',            
            'rgb_camera.profile': '640x480x30',
        }.items(),
    )

    yolo_node = Node(
        package='roomba_ros2',
        namespace='roomba',
        executable='yolo_node',
        name='yolo_node',
        output='screen'
    )

    return LaunchDescription([
        rs_launch,
        #yolo_node
    ])
