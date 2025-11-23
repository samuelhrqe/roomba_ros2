import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from visualization_msgs.msg import Marker
import cv_bridge
import tf2_ros
from ultralytics import YOLO
import numpy as np

class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')

        self._stopping = 0

        # Initialize YOLO model
        self.model = YOLO('yolo11n.pt')
        self.get_logger().info("YOLO model loaded successfully.")

        self.cv_bridge = cv_bridge.CvBridge()

        # Synchronized subscribers
        self.rgb_sub = Subscriber(self, Image, '/L515/color/image_raw')
        self.depth_sub = Subscriber(self, Image, '/L515/depth/image_rect_raw')
        self.info_sub = Subscriber(self, CameraInfo, '/L515/depth/camera_info')
        
        self.depth_image = None
        self.camera_info = None

        # Synchronized callback
        self.ts = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub, self.info_sub],
            queue_size=10,
            slop=0.1
        )
        self.ts.registerCallback(self.synced_callback)
        
        # Publishers
        self.image_pub = self.create_publisher(Image, '/yolo_node/annoted_image', 10)
        self.marker_pub = self.create_publisher(Marker, '/yolo_node/object_3d_point', 10)
        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

    def synced_callback(self, rgb_msg, depth_msg, info_msg):
        if rgb_msg is None or depth_msg is None or info_msg is None:
            self.get_logger().warning("Received None message.")
            return
        
        self.camera_info = info_msg
        
        # Convert ROS Image to OpenCV Image
        cv_image = self.cv_bridge.imgmsg_to_cv2(rgb_msg, rgb_msg.encoding)
        self.depth_image = self.cv_bridge.imgmsg_to_cv2(depth_msg)

        results = self.model(cv_image, verbose=False)
        if results is None or len(results) == 0:
            self.get_logger().warning("No detections made.")
            return
               
        annotaded_image = results[0].plot()
        detected_image_msg = self.cv_bridge.cv2_to_imgmsg(annotaded_image, encoding='rgb8')
        detected_image_msg.header = rgb_msg.header  # keep timestamp
        self.image_pub.publish(detected_image_msg)
        self.get_logger().info("Publish!")

        # Matriz intrínseca K da câmera de profundidade
        K = info_msg.k  # [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        fx = K[0]
        fy = K[4]
        cx0 = K[2]
        cy0 = K[5]

        for result in results:
            boxes = result.boxes
            for box in boxes:
                cls_id = int(box.cls[0])
                cls_name = self.model.names[cls_id]
                conf = float(box.conf[0])
                if conf < 0.5:
                    continue
                
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                center_x = (x1 + x2) // 2
                center_y = (y1 + y2) // 2

                if center_x < 0 or center_y < 0 or center_x >= self.depth_image.shape[1] or center_y >= self.depth_image.shape[0]:
                    continue

                depth_pixel = self.depth_image[center_y, center_x]
                # Caso depth_pixel seja um array (ex: tem canal extra), achata e pega o primeiro valor
                if isinstance(depth_pixel, np.ndarray):
                    if depth_pixel.size == 0:
                        return  # nada para usar
                    depth_raw = float(depth_pixel.reshape(-1)[0])
                else:
                    depth_raw = float(depth_pixel)

                # Ignore invalid depth
                if np.isnan(depth_raw):
                    return

                Z = depth_raw / 1000.0
                X = (center_x - cx0) * Z / fx
                Y = (center_y - cy0) * Z / fy

                # Marker 3D
                marker = Marker()
                marker.header.frame_id = "L515_link"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.id = cls_id  # ou outro id único se quiser vários ao mesmo tempo

                marker.pose.position.x = X
                marker.pose.position.y = Y
                marker.pose.position.z = Z
                marker.pose.orientation.w = 1.0

                marker.scale.x = marker.scale.y = marker.scale.z = 0.05

                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 1.0

                marker.text = f"{cls_name} ({conf:.2f})"

                self.marker_pub.publish(marker)

                t = TransformStamped()
                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = "L515_color_optical_frame"
                t.child_frame_id = f"{cls_name}_{int(cls_id)}"

                t.transform.translation.x = X
                t.transform.translation.y = Y
                t.transform.translation.z = Z

                # No orientation info → identity quaternion
                t.transform.rotation.x = 0.0
                t.transform.rotation.y = 0.0
                t.transform.rotation.z = 0.0
                t.transform.rotation.w = 1.0

                self.tf_broadcaster.sendTransform(t)

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

                


