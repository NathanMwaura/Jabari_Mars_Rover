#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from rclpy.exceptions import ParameterException

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.bridge = CvBridge()

        # ✅ Declare parameters with in-code defaults
        self.declare_parameter('camera_type', 'webcam')                # 'webcam', 'droidcam', 'picam'
        self.declare_parameter('camera_source', 'auto')                # 'auto' scans /dev/video*
        self.declare_parameter('frame_rate', 30.0)
        self.declare_parameter('frame_id', 'webcam_link')

        # 🧠 Load parameter values
        self.camera_type = self.get_parameter('camera_type').get_parameter_value().string_value
        self.camera_source = self.get_parameter('camera_source').get_parameter_value().string_value
        self.frame_rate = self.get_parameter('frame_rate').get_parameter_value().double_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        # 🎥 Initialize video capture
        self.cap = None
        if self.camera_type == 'droidcam':
            self.cap = cv2.VideoCapture(self.camera_source)
        elif self.camera_type in ['webcam', 'picam']:
            if self.camera_source == 'auto':
                self.cap = self.find_working_camera()
            else:
                try:
                    source = int(self.camera_source)
                except ValueError:
                    source = self.camera_source
                self.cap = cv2.VideoCapture(source)
        else:
            self.get_logger().error(f"Unsupported camera type: {self.camera_type}")
            raise ParameterException(f"Invalid camera_type: {self.camera_type}")

        if not self.cap or not self.cap.isOpened():
            self.get_logger().error(f"Failed to open camera: {self.camera_source} (type: {self.camera_type})")
            raise RuntimeError("Could not open video stream")

        self.get_logger().info(f"Connected to {self.camera_type} at source {self.camera_source}")
        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        self.timer = self.create_timer(1.0 / self.frame_rate, self.publish_frame)

    def find_working_camera(self):
        for i in range(6):  # Scan /dev/video0 to /dev/video5
            self.get_logger().info(f"Trying /dev/video{i}...")
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                ret, _ = cap.read()
                if ret:
                    self.get_logger().info(f"Auto-selected /dev/video{i} as working camera")
                    self.camera_source = f"/dev/video{i}"
                    return cap
                cap.release()
        raise RuntimeError("No working camera found on /dev/video*")

    def publish_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to read frame from camera")
            return

        try:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            self.publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish frame: {str(e)}")

    def destroy_node(self):
        self.get_logger().info("Releasing camera resources")
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CameraPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {str(e)}")
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

















