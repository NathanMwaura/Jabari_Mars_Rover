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

        # Declare parameters
        self.declare_parameter('camera_type', 'webcam')  # Options: 'webcam', 'droidcam', 'picam'
        self.declare_parameter('camera_source', '0')      # Webcam index, DroidCam URL, or PiCam device
        self.declare_parameter('frame_rate', 30.0)
        self.declare_parameter('frame_id', 'camera_link')

        # Get parameters
        self.camera_type = self.get_parameter('camera_type').get_parameter_value().string_value
        self.camera_source = self.get_parameter('camera_source').get_parameter_value().string_value
        self.frame_rate = self.get_parameter('frame_rate').get_parameter_value().double_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        # Initialize publisher
        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)

        # Initialize video capture based on camera type
        self.cap = None
        if self.camera_type == 'droidcam':
            self.cap = cv2.VideoCapture(self.camera_source)
        elif self.camera_type in ['webcam', 'picam']:
            # For webcam or PiCam, try to interpret camera_source as an index or device path
            try:
                source = int(self.camera_source)  # Try as integer (webcam index)
            except ValueError:
                source = self.camera_source  # Use as device path (e.g., /dev/video0 for PiCam)
            self.cap = cv2.VideoCapture(source)
        else:
            self.get_logger().error(f"Unsupported camera type: {self.camera_type}")
            raise ParameterException(f"Invalid camera_type: {self.camera_type}", parameters=[])

        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open camera: {self.camera_source} (type: {self.camera_type})")
            raise RuntimeError("Could not open video stream")

        self.get_logger().info(f"Connected to {self.camera_type} at {self.camera_source}")
        self.timer = self.create_timer(1.0 / self.frame_rate, self.publish_frame)

    def publish_frame(self):
        if self.cap is None:
            self.get_logger().warn(f"Camera capture is not initialized for {self.camera_type}")
            return
        
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn(f"Failed to read frame from {self.camera_type}")
            return

        try:
            # Convert OpenCV frame to ROS Image message
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            self.publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish frame: {str(e)}")

    def destroy_node(self):
        self.get_logger().info(f"Releasing {self.camera_type} resources")
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    try:
        node = CameraPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {str(e)}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()