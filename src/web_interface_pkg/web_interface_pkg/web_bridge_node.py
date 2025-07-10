#!/usr/bin/env python3
"""
ROS2 Web Bridge Node for Dell Latitude E7440 Camera Streaming.
This node captures camera feed and streams it via WebSocket to web clients.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
import cv2
import numpy as np
from cv_bridge import CvBridge
import threading
import time
from .websocket_handler import WebSocketHandler

class WebBridgeNode(Node):
    def __init__(self):
        super().__init__('web_bridge_node')
        
        # Declare parameters
        self.declare_parameter('camera_topic', 'camera/image_raw')
        self.declare_parameter('websocket_host', 'localhost')
        self.declare_parameter('websocket_port', 8080)
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('publish_compressed', True)
        self.declare_parameter('jpeg_quality', 80)
        
        # Get parameters
        self.camera_topic = self.get_parameter('camera_topic').value
        self.websocket_host = self.get_parameter('websocket_host').value
        self.websocket_port = self.get_parameter('websocket_port').value
        self.frame_width = self.get_parameter('frame_width').value
        self.frame_height = self.get_parameter('frame_height').value
        self.fps = self.get_parameter('fps').value
        self.publish_compressed = self.get_parameter('publish_compressed').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Initialize camera
        
        # Initialize WebSocket handler
        self.websocket_handler = WebSocketHandler(
            host=self.websocket_host,
            port=self.websocket_port
        )
        
        # ROS2 publishers
        self.image_pub = self.create_publisher(Image, 'camera/image_raw', 10)
        if self.publish_compressed:
            self.compressed_image_pub = self.create_publisher(
                CompressedImage, 'camera/image_raw/compressed', 10
            )
        
        # Status publisher
        self.status_pub = self.create_publisher(String, 'camera/status', 10)
        
        # Statistics
        self.frame_count = 0
        self.last_stats_time = time.time()
        
        # ROS2 subscribers
        self.image_sub = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            10
        )

        self.start_websocket_server()
        
        self.get_logger().info(
            f"Web Bridge Node initialized - WebSocket: ws://{self.websocket_host}:{self.websocket_port}"
        )
        
    def image_callback(self, msg):
        """Callback for incoming camera images."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.process_frame(frame)
        except Exception as e:
            self.get_logger().error(f"Error in image_callback: {e}")
            
    def start_websocket_server(self):
        """Start the WebSocket server for web interface."""
        try:
            self.websocket_thread = self.websocket_handler.start_server_thread()
            self.get_logger().info(
                f"WebSocket server started on ws://{self.websocket_host}:{self.websocket_port}"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to start WebSocket server: {e}")
            
                
    def process_frame(self, frame):
        """Process captured frame and distribute to subscribers."""
        try:
            self.frame_count += 1
            current_time = self.get_clock().now()
            
            # Create ROS Image message
            ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            ros_image.header.stamp = current_time.to_msg()
            ros_image.header.frame_id = "camera_frame"
            
            # Publish raw image
            self.image_pub.publish(ros_image)
            
            # Publish compressed image if enabled
            if self.publish_compressed:
                compressed_msg = CompressedImage()
                compressed_msg.header = ros_image.header
                compressed_msg.format = "jpeg"
                
                # Compress image
                encode_params = [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
                _, compressed_data = cv2.imencode('.jpg', frame, encode_params)
                compressed_msg.data = compressed_data.tobytes()
                
                self.compressed_image_pub.publish(compressed_msg)
                
                # Send to WebSocket clients (use compressed data)
                self.websocket_handler.add_image(
                    compressed_data.tobytes(),
                    frame.shape[1],  # width
                    frame.shape[0],  # height
                    "jpeg"
                )
            else:
                # Send raw frame to WebSocket clients
                _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                self.websocket_handler.add_image(
                    buffer.tobytes(),
                    frame.shape[1],  # width
                    frame.shape[0],  # height
                    "jpeg"
                )
            
            # Log statistics every 100 frames
            if self.frame_count % 100 == 0:
                current_stats_time = time.time()
                fps = 100 / (current_stats_time - self.last_stats_time)
                client_count = self.websocket_handler.get_connected_clients_count()
                
                self.get_logger().info(
                    f"Processed {self.frame_count} frames, Current FPS: {fps:.2f}, "
                    f"WebSocket clients: {client_count}"
                )
                
                # Update status
                status_msg = String()
                status_msg.data = f"Streaming: {fps:.1f}fps, Clients: {client_count}"
                self.status_pub.publish(status_msg)
                
                # Send status to web clients
                self.websocket_handler.send_system_message(
                    f"Camera FPS: {fps:.1f}, Connected clients: {client_count}",
                    "status"
                )
                
                self.last_stats_time = current_stats_time
                
        except Exception as e:
            self.get_logger().error(f"Error processing frame: {e}")
            
    def publish_error_status(self, error_message: str):
        """Publish error status message."""
        status_msg = String()
        status_msg.data = f"ERROR: {error_message}"
        self.status_pub.publish(status_msg)
        
        # Also send to web clients
        self.websocket_handler.send_system_message(error_message, "error")
        
    def cleanup(self):
        """Cleanup resources."""
        self.get_logger().info("Cleaning up Web Bridge Node...")
        
        # Stop WebSocket server
        try:
            self.websocket_handler.stop_server()
        except Exception as e:
            self.get_logger().error(f"Error stopping WebSocket server: {e}")
            
        # Release camera
                
        self.get_logger().info("Web Bridge Node cleanup complete")
        
    def __del__(self):
        """Destructor."""
        self.cleanup()

def main(args=None):
    """Main function."""
    rclpy.init(args=args)
    
    try:
        node = WebBridgeNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Keyboard interrupt detected")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'node' in locals():
            node.cleanup()
        rclpy.shutdown()

if __name__ == '__main__':
    main()