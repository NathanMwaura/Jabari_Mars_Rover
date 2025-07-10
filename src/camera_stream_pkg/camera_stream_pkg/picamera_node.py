#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np
from picamera2 import Picamera2
import threading
import time
import os
from datetime import datetime

class PiCameraNode(Node):
    def __init__(self):
        super().__init__('picamera_node')
        
        # Initialize parameters
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('fps', 30)
        self.declare_parameter('quality', 80)
        self.declare_parameter('format', 'MJPEG')
        
        # Initialize camera
        self.picam2 = Picamera2()
        self.bridge = CvBridge()
        self.is_recording = False
        self.video_writer = None
        self.frame_count = 0
        self.last_fps_time = time.time()
        
        # ROS2 Publishers
        self.image_pub = self.create_publisher(
            Image, 
            '/camera/image_raw', 
            10
        )
        
        self.compressed_image_pub = self.create_publisher(
            CompressedImage, 
            '/camera/image_raw/compressed', 
            10
        )
        
        self.camera_info_pub = self.create_publisher(
            CameraInfo, 
            '/camera/camera_info', 
            10
        )
        
        # ROS2 Subscribers
        self.recording_sub = self.create_subscription(
            Bool,
            '/camera/recording_command',
            self.recording_callback,
            10
        )
        
        # Initialize and start camera
        self.setup_camera()
        self.start_camera()
        
        # Timer for publishing camera info
        self.timer = self.create_timer(1.0, self.publish_camera_info)
        
        self.get_logger().info('PiCamera node started')
    
    def setup_camera(self):
        """Configure the camera with current parameters"""
        width = self.get_parameter('width').value
        height = self.get_parameter('height').value
        fps = self.get_parameter('fps').value
        
        # Configure camera
        config = self.picam2.create_video_configuration(
            main={"size": (width, height), "format": "RGB888"},
            controls={"FrameRate": fps}
        )
        
        self.picam2.configure(config)
        
        self.get_logger().info(f'Camera configured: {width}x{height} @ {fps}fps')
    
    def start_camera(self):
        """Start the camera and begin capture thread"""
        try:
            self.picam2.start()
            
            # Start capture thread
            self.capture_thread = threading.Thread(target=self.capture_loop)
            self.capture_thread.daemon = True
            self.capture_thread.start()
            
            self.get_logger().info('Camera started successfully')
            
        except Exception as e:
            self.get_logger().error(f'Failed to start camera: {str(e)}')
    
    def capture_loop(self):
        """Main capture loop running in separate thread"""
        while rclpy.ok():
            try:
                # Capture frame
                frame = self.picam2.capture_array()
                
                if frame is not None:
                    # Convert to OpenCV format (BGR)
                    cv_image = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                    
                    # Publish raw image
                    self.publish_raw_image(cv_image)
                    
                    # Publish compressed image
                    self.publish_compressed_image(cv_image)
                    
                    # Handle recording
                    if self.is_recording:
                        self.write_video_frame(cv_image)
                    
                    # Update frame count for FPS calculation
                    self.update_fps()
                    
                # Small delay to prevent overwhelming the system
                time.sleep(0.01)
                
            except Exception as e:
                self.get_logger().error(f'Capture error: {str(e)}')
                time.sleep(0.1)
    
    def publish_raw_image(self, cv_image):
        """Publish raw image message"""
        try:
            msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_frame"
            self.image_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f'Failed to publish raw image: {str(e)}')
    
    def publish_compressed_image(self, cv_image):
        """Publish compressed image message"""
        try:
            # Compress image to JPEG
            quality = self.get_parameter('quality').value
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), quality]
            _, compressed_data = cv2.imencode('.jpg', cv_image, encode_param)
            
            # Create compressed image message
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_frame"
            msg.format = "jpeg"
            msg.data = compressed_data.tobytes()
            
            self.compressed_image_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f'Failed to publish compressed image: {str(e)}')
    
    def publish_camera_info(self):
        """Publish camera info message"""
        try:
            width = self.get_parameter('width').value
            height = self.get_parameter('height').value
            
            msg = CameraInfo()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_frame"
            msg.width = width
            msg.height = height
            
            # Camera matrix (you should calibrate your camera for accurate values)
            msg.k = [
                800.0, 0.0, width/2.0,
                0.0, 800.0, height/2.0,
                0.0, 0.0, 1.0
            ]
            
            # Distortion coefficients (set to zero if not calibrated)
            msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
            
            # Rectification matrix
            msg.r = [
                1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0
            ]
            
            # Projection matrix
            msg.p = [
                800.0, 0.0, width/2.0, 0.0,
                0.0, 800.0, height/2.0, 0.0,
                0.0, 0.0, 1.0, 0.0
            ]
            
            self.camera_info_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f'Failed to publish camera info: {str(e)}')
    
    def recording_callback(self, msg):
        """Handle recording start/stop commands"""
        if msg.data and not self.is_recording:
            self.start_recording()
        elif not msg.data and self.is_recording:
            self.stop_recording()
    
    def start_recording(self):
        """Start video recording"""
        try:
            # Create recordings directory if it doesn't exist
            os.makedirs('/home/pi/recordings', exist_ok=True)
            
            # Generate filename with timestamp
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = f'/home/pi/recordings/video_{timestamp}.mp4'
            
            # Initialize video writer
            width = self.get_parameter('width').value
            height = self.get_parameter('height').value
            fps = self.get_parameter('fps').value
            
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.video_writer = cv2.VideoWriter(filename, fourcc, fps, (width, height))
            
            self.is_recording = True
            self.get_logger().info(f'Started recording to: {filename}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to start recording: {str(e)}')
    
    def stop_recording(self):
        """Stop video recording"""
        try:
            if self.video_writer:
                self.video_writer.release()
                self.video_writer = None
            
            self.is_recording = False
            self.get_logger().info('Recording stopped')
            
        except Exception as e:
            self.get_logger().error(f'Failed to stop recording: {str(e)}')
    
    def write_video_frame(self, frame):
        """Write frame to video file"""
        try:
            if self.video_writer:
                self.video_writer.write(frame)
                
        except Exception as e:
            self.get_logger().error(f'Failed to write video frame: {str(e)}')
    
    def update_fps(self):
        """Update FPS calculation"""
        self.frame_count += 1
        current_time = time.time()
        
        if current_time - self.last_fps_time >= 1.0:
            fps = self.frame_count / (current_time - self.last_fps_time)
            self.get_logger().debug(f'Current FPS: {fps:.2f}')
            self.frame_count = 0
            self.last_fps_time = current_time
    
    def destroy_node(self):
        """Cleanup when node is destroyed"""
        self.get_logger().info('Shutting down camera node...')
        
        # Stop recording if active
        if self.is_recording:
            self.stop_recording()
        
        # Stop camera
        if hasattr(self, 'picam2'):
            self.picam2.stop()
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = PiCameraNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()