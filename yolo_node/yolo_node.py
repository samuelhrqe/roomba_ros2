import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
from geometry_msgs.msg import TransformStamped
import cv_bridge
import pyrealsense2 as rs

import cv2
from ultralytics import YOLO
import tf2_ros
import math

class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')

        self._stopping = 0

        # Initialize YOLO model
        self.model = YOLO('yolo11n.pt')
        self.get_logger().info("YOLO model loaded successfully.")

        self.cv_bridge = cv_bridge.CvBridge()

        # Subscribers
        self.rgb_sub = self.create_subscription(
            Image,
            '/L515/color/image_raw',
            self.image_callback,
            10)
        
        self.depth_sub = self.create_subscription(
            Image,
            '/L515/depth/image_rect_raw',
            self.depth_callback,
            10)
        
        self.depth_image = None
        
        # Publishers
        self.image_pub = self.create_publisher(Image, '/yolo_node/annoted_image', 10)
        self.marker_pub = self.create_publisher(Marker, '/yolo_node/marker', 10)

    def depth_callback(self, msg):
        self.depth_image = self.cv_bridge.imgmsg_to_cv2(msg, msg.encoding)

    def image_callback(self, msg):
        if msg is None:
            self.get_logger().warning("Received None message.")
            return
        
        # Convert ROS Image to OpenCV Image
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, msg.encoding)

        results = self.model(cv_image, verbose=False)
        if results is None or len(results) == 0:
            self.get_logger().warning("No detections made.")
            return
        
        height, width, channels = cv_image.shape
        
        annotaded_image = cv_image.copy()
        annotaded_image = results[0].plot()
        detected_image_msg = self.cv_bridge.cv2_to_imgmsg(annotaded_image, encoding='rgb8') 
        self.image_pub.publish(detected_image_msg)

        for result in results:
            boxes = result.boxes
            for box in boxes:
                cls_id = int(box.cls[0])
                cls_name = self.model.names[cls_id]
                conf = box.conf[0]
                if conf < 0.5:
                    continue
                
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cx = (x1 + x2) // 2
                cy = (y1 + y2) // 2

                # if self.depth_image is not None:
                #     depth = float(self.depth_image[cy, cx]) / 255.0
                # else:
                #     depth = float('nan')


                # Publish Marker
                marker = Marker()
                marker.header.frame_id = "L515_link"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.text = f"Class: [{cls_name}: {cls_id}], Conf: {conf:.2f}"
                self.marker_pub.publish(marker)
                # self.get_logger().info(f"Published marker for {cls_name} with confidence {conf:.2f}.")

    def shutdown(self):
        
        if self._stopping:
            return
        
        self._stopping = True

        try:
            self.destroy_subscription(self.rgb_sub)
            self.destroy_subscription(self.depth_sub)
            self.destroy_publisher(self.image_pub)
            self.destroy_publisher(self.marker_pub)
        except Exception:
            pass

        print('Shutting down YoloNode...')


def main(args=None):
    rclpy.init(args=args)
    yolo_node = YoloNode()
    try:
        rclpy.spin(yolo_node)
    except KeyboardInterrupt:
        pass
    finally:
        yolo_node.shutdown()
        yolo_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

                


