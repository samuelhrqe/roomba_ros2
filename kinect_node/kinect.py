import atexit
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
import freenect

from ament_index_python.packages import get_package_share_directory
from camera_info_manager import CameraInfoManager

def _safe_sync_stop():
    # Avoid exceptions on exit
    try:
        freenect.sync_stop()
    except Exception:
        pass

class KinectNode(Node):
    def __init__(self):
        super().__init__('kinect_node')

        # Publishers
        self.depth_pub = self.create_publisher(Image, '/depth/image_raw', 10)
        self.rgb_pub = self.create_publisher(Image, '/rgb/image_raw', 10)
        self.depth_info_pub = self.create_publisher(CameraInfo, '/depth/camera_info', 10)
        self.rgb_info_pub = self.create_publisher(CameraInfo, '/rgb/camera_info', 10)

        _safe_sync_stop()
        self._stopping = False
        self._timer = None
        atexit.register(_safe_sync_stop)

        self.cv_bridge = CvBridge()

        # CameraInfo (Calibration files)
        self.pkg_share = get_package_share_directory('roomba_ros2')
        self.depth_info_url = 'file://' + self.pkg_share + '/kinect_node/config/kinect_calibration/calibration_depth.yaml'
        self.rgb_info_url = 'file://' + self.pkg_share + '/kinect_node/config/kinect_calibration/calibration_rgb.yaml'

        self.depth_cinfo_mgr = CameraInfoManager(self, 'kinect', self.depth_info_url)
        self.rgb_cinfo_mgr = CameraInfoManager(self, 'kinect', self.rgb_info_url)

        self.depth_cinfo_mgr.loadCameraInfo()
        self.rgb_cinfo_mgr.loadCameraInfo()

        self.depth_info = self.depth_cinfo_mgr.getCameraInfo()
        self.rgb_info = self.rgb_cinfo_mgr.getCameraInfo()

        self.depth_info.header.frame_id = 'kinect_depth'
        self.rgb_info.header.frame_id = 'kinect_rgb'

        try:
            freenect.set_log_level(freenect.LOG_FATAL)
        except Exception:
            pass

        self.timer = self.create_timer(0.01, self.timer_cb)

        self.get_logger().info('Kinect ROS2 Python started.')

    def shutdown(self):
        """Idempotent: can be called multiple times without breaking."""
        if self._stopping:
            return
        self._stopping = True
        try:
            if self.timer is not None:
                self.timer.cancel()
        except Exception:
            pass

        _safe_sync_stop()

        print("Shutting down KinectNode...")

    def timer_cb(self):

        if self._stopping or not rclpy.ok():
            return

        depth_mm_tup = freenect.sync_get_depth(0, freenect.DEPTH_MM)
        rgb_tup = freenect.sync_get_video()

        if not depth_mm_tup or not rgb_tup:
            return

        depth_mm, _ = depth_mm_tup
        rgb, _ = rgb_tup

        # Conversions
        # 1) Depth: ensure uint16 and contiguity for cv_bridge (like CV_16UC1 in C++)
        depth_mm = np.ascontiguousarray(depth_mm, dtype=np.uint16)

        # 2) RGB: ensure uint8 and contiguity for cv_bridge (like rgb8 in C++)
        rgb = np.ascontiguousarray(rgb, dtype=np.uint8)

        # --- HEADERS ---
        stamp = self.get_clock().now().to_msg()

        # Publish depth (16UC1)
        depth_msg = self.cv_bridge.cv2_to_imgmsg(depth_mm, encoding='16UC1')
        depth_msg.header.stamp = stamp
        depth_msg.header.frame_id = 'kinect_depth'
        self.depth_info.header.stamp = stamp

        self.depth_pub.publish(depth_msg)
        self.depth_info_pub.publish(self.depth_info)

        # Publish RGB (rgb8)
        rgb_msg = self.cv_bridge.cv2_to_imgmsg(rgb, encoding='rgb8')
        rgb_msg.header.stamp = stamp
        rgb_msg.header.frame_id = 'kinect_rgb'
        self.rgb_info.header.stamp = stamp

        self.rgb_pub.publish(rgb_msg)
        self.rgb_info_pub.publish(self.rgb_info)

def main():
    rclpy.init()
    node = KinectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
