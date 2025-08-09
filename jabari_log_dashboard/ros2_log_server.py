#!/usr/bin/env python3
"""
ROS2 Log Web Dashboard Server

This Flask application provides a web interface to monitor ROS2 logs in real-time.
It captures logs from the manual control node and streams them to a web dashboard.

Features:
- Real-time log streaming via WebSockets
- Log filtering by level and source
- Interactive log viewer with search
- System status monitoring
- Export logs functionality

Requirements:
pip install flask flask-socketio rclpy std-msgs geometry-msgs sensor-msgs

Usage:
python3 ros2_log_server.py
Then open http://localhost:5000 in your browser
"""

import json
import time
import threading
import logging
from datetime import datetime
from collections import deque
from typing import Dict, List, Optional
import re

import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image

from flask import Flask, render_template_string, request, jsonify
from flask_socketio import SocketIO, emit


class LogEntry:
    """Structured log entry for web display"""
    def __init__(self, timestamp: float, level: str, source: str, message: str, data: Optional[Dict] = None):
        self.timestamp = timestamp
        self.level = level
        self.source = source
        self.message = message
        self.data = data or {}
        
    def to_dict(self):
        return {
            'timestamp': self.timestamp,
            'time_str': datetime.fromtimestamp(self.timestamp).strftime('%H:%M:%S.%f')[:-3],
            'level': self.level,
            'source': self.source,
            'message': self.message,
            'data': self.data
        }


class ROS2LogMonitor(Node):
    """ROS2 node that captures logs and system status"""
    
    def __init__(self, socketio):
        super().__init__('log_monitor_node')
        self.socketio = socketio
        self.log_buffer = deque(maxlen=1000)  # Keep last 1000 log entries
        self.system_status = {}
        
        # Import additional message types
        from sensor_msgs.msg import Image
        from std_msgs.msg import Float32
        
        # Subscribers for monitoring the manual control node
        self.status_sub = self.create_subscription(
            String, '/manual_control_status', self.status_callback, 10)
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.vector3_sub = self.create_subscription(
            Vector3, '/vector3_cmd', self.vector3_callback, 10)
            
        # Additional subscribers for new nodes
        # Motor controller monitoring
        self.left_motor_sub = self.create_subscription(
            Float32, '/motor/left_speed', self.left_motor_callback, 10)
        self.right_motor_sub = self.create_subscription(
            Float32, '/motor/right_speed', self.right_motor_callback, 10)
            
        # Object detection monitoring
        self.object_info_sub = self.create_subscription(
            String, '/object_info', self.object_detection_callback, 10)
        self.camera_sub = self.create_subscription(
            Image, '/camera/image_raw', self.camera_callback, 10)
        self.annotated_image_sub = self.create_subscription(
            Image, '/image_annotated', self.annotated_image_callback, 10)
            
        # Custom log handler to capture our node's logs
        self.setup_log_capture()
        
        # Initialize motor status tracking
        self.motor_status = {'left_speed': 0.0, 'right_speed': 0.0}
        self.last_object_detection = None
        self.camera_frame_count = 0
        
        self.get_logger().info("ROS2 Log Monitor initialized with additional node monitoring")
        
    def setup_log_capture(self):
        """Setup custom log handler to capture ROS2 logs"""
        class WebLogHandler(logging.Handler):
            def __init__(self, monitor):
                super().__init__()
                self.monitor = monitor
                
            def emit(self, record):
                try:
                    # Parse ROS2 log format
                    level = record.levelname
                    source = getattr(record, 'name', 'unknown')
                    message = record.getMessage()
                    
                    log_entry = LogEntry(
                        timestamp=time.time(),
                        level=level,
                        source=source,
                        message=message
                    )
                    
                    self.monitor.add_log_entry(log_entry)
                except Exception:
                    pass  # Don't let logging errors break the system
        
        # Note: This captures Python logs, but ROS2 C++ logs need different approach
        # For full ROS2 log capture, you'd need to parse /rosout topic
        handler = WebLogHandler(self)
        logging.getLogger().addHandler(handler)
    
    def status_callback(self, msg):
        """Handle manual control status updates"""
        try:
            status_data = json.loads(msg.data)
            self.system_status.update(status_data)
            
            # Create log entry for status update
            log_entry = LogEntry(
                timestamp=time.time(),
                level='INFO',
                source='manual_control',
                message='Status update received',
                data=status_data
            )
            self.add_log_entry(log_entry)
            
            # Emit status to web clients (broadcast to all clients)
            self.socketio.emit('status_update', status_data)
            
        except Exception as e:
            self.get_logger().error(f"Error processing status: {e}")
    
    def cmd_vel_callback(self, msg):
        """Handle movement commands"""
        if abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01:
            log_entry = LogEntry(
                timestamp=time.time(),
                level='INFO',
                source='movement',
                message=f'Movement: linear={msg.linear.x:.2f}, angular={msg.angular.z:.2f}',
                data={
                    'linear_x': msg.linear.x,
                    'angular_z': msg.angular.z
                }
            )
            self.add_log_entry(log_entry)
    
    def vector3_callback(self, msg):
        """Handle servo commands"""
        log_entry = LogEntry(
            timestamp=time.time(),
            level='INFO',
            source='servos',
            message=f'Servo: pan={msg.x:.1f}°, tilt={msg.y:.1f}°, third={msg.z:.1f}°',
            data={
                'pan': msg.x,
                'tilt': msg.y,
                'third': msg.z
            }
        )
        self.add_log_entry(log_entry)
    
    def left_motor_callback(self, msg):
        """Handle left motor speed updates"""
        self.motor_status['left_speed'] = msg.data
        log_entry = LogEntry(
            timestamp=time.time(),
            level='INFO',
            source='motor_controller',
            message=f'Left motor speed: {msg.data:.2f}',
            data={'left_speed': msg.data}
        )
        self.add_log_entry(log_entry)
        
        # Update system status for web display
        self.system_status['motor_left'] = msg.data
        self.socketio.emit('status_update', {'motor_left': msg.data})
    
    def right_motor_callback(self, msg):
        """Handle right motor speed updates"""
        self.motor_status['right_speed'] = msg.data
        log_entry = LogEntry(
            timestamp=time.time(),
            level='INFO',
            source='motor_controller',
            message=f'Right motor speed: {msg.data:.2f}',
            data={'right_speed': msg.data}
        )
        self.add_log_entry(log_entry)
        
        # Update system status for web display
        self.system_status['motor_right'] = msg.data
        self.socketio.emit('status_update', {'motor_right': msg.data})
    
    def object_detection_callback(self, msg):
        """Handle object detection results"""
        if msg.data and msg.data != self.last_object_detection:
            self.last_object_detection = msg.data
            
            # Parse detection info for logging
            detection_count = msg.data.count('Object:')
            
            log_entry = LogEntry(
                timestamp=time.time(),
                level='INFO',
                source='object_detection',
                message=f'Detected {detection_count} objects: {msg.data[:100]}...' if len(msg.data) > 100 else msg.data,
                data={
                    'detection_count': detection_count,
                    'full_detection': msg.data
                }
            )
            self.add_log_entry(log_entry)
            
            # Update system status
            self.system_status['last_detection'] = {
                'count': detection_count,
                'timestamp': time.time(),
                'data': msg.data
            }
            self.socketio.emit('status_update', {'object_detection': self.system_status['last_detection']})
    
    def camera_callback(self, msg):
        """Handle camera frame updates (minimal logging to avoid spam)"""
        self.camera_frame_count += 1
        
        # Log every 30 frames to avoid spam
        if self.camera_frame_count % 30 == 0:
            log_entry = LogEntry(
                timestamp=time.time(),
                level='DEBUG',
                source='camera',
                message=f'Camera frame #{self.camera_frame_count} - {msg.width}x{msg.height}',
                data={
                    'frame_count': self.camera_frame_count,
                    'width': msg.width,
                    'height': msg.height,
                    'encoding': msg.encoding
                }
            )
            self.add_log_entry(log_entry)
            
            # Update system status
            self.system_status['camera'] = {
                'frame_count': self.camera_frame_count,
                'resolution': f"{msg.width}x{msg.height}",
                'encoding': msg.encoding,
                'last_frame_time': time.time()
            }
    
    def annotated_image_callback(self, msg):
        """Handle annotated image updates"""
        log_entry = LogEntry(
            timestamp=time.time(),
            level='DEBUG',
            source='yolo_processor',
            message=f'Annotated image published - {msg.width}x{msg.height}',
            data={
                'width': msg.width,
                'height': msg.height,
                'encoding': msg.encoding
            }
        )
        self.add_log_entry(log_entry)
    
    def add_log_entry(self, log_entry: LogEntry):
        """Add log entry to buffer and emit to web clients"""
        self.log_buffer.append(log_entry)
        
        # Emit to all connected clients (broadcast)
        self.socketio.emit('new_log', log_entry.to_dict())
    
    def get_recent_logs(self, count: int = 100) -> List[Dict]:
        """Get recent log entries"""
        return [entry.to_dict() for entry in list(self.log_buffer)[-count:]]


# Flask Application
app = Flask(__name__)
app.config['SECRET_KEY'] = 'ros2_log_dashboard_secret'
socketio = SocketIO(app, cors_allowed_origins="*")

# Global variables
log_monitor = None
ros_thread = None


def run_ros_node():
    """Run ROS2 node in separate thread"""
    global log_monitor
    rclpy.init()
    try:
        log_monitor = ROS2LogMonitor(socketio)
        rclpy.spin(log_monitor)
    except Exception as e:
        print(f"ROS2 node error: {e}")
    finally:
        if log_monitor:
            log_monitor.destroy_node()
        rclpy.shutdown()


# Web Routes
@app.route('/')
def dashboard():
    """Main dashboard page"""
    return render_template_string(HTML_TEMPLATE)


@app.route('/api/logs')
def get_logs():
    """API endpoint to get recent logs"""
    count = request.args.get('count', 100, type=int)
    level_filter = request.args.get('level', '')
    source_filter = request.args.get('source', '')
    
    if log_monitor:
        logs = log_monitor.get_recent_logs(count)
        
        # Apply filters
        if level_filter:
            logs = [log for log in logs if log['level'].upper() == level_filter.upper()]
        if source_filter:
            logs = [log for log in logs if source_filter.lower() in log['source'].lower()]
            
        return jsonify(logs)
    
    return jsonify([])


@app.route('/api/status')
def get_status():
    """API endpoint to get system status"""
    if log_monitor:
        return jsonify(log_monitor.system_status)
    return jsonify({})


# WebSocket Events
@socketio.on('connect')
def handle_connect():
    """Handle client connection"""
    print(f"Client connected: {request.sid}")
    
    # Send recent logs to new client
    if log_monitor:
        recent_logs = log_monitor.get_recent_logs(50)
        for log_entry in recent_logs:
            emit('new_log', log_entry)
        
        # Send current status
        if log_monitor.system_status:
            emit('status_update', log_monitor.system_status)


@socketio.on('disconnect')
def handle_disconnect():
    """Handle client disconnection"""
    print(f"Client disconnected: {request.sid}")


@socketio.on('clear_logs')
def handle_clear_logs():
    """Handle request to clear logs"""
    if log_monitor:
        log_monitor.log_buffer.clear()
        # Broadcast to all clients (correct Flask-SocketIO syntax)
        socketio.emit('logs_cleared')


# HTML Template (will be in separate artifact)
HTML_TEMPLATE = """
<!-- This will reference the separate HTML artifact -->
<h1>Loading ROS2 Log Dashboard...</h1>
<script>
window.location.href = '/static/dashboard.html';
</script>
"""


if __name__ == '__main__':
    print("Starting ROS2 Log Web Dashboard...")
    print("Dashboard will be available at: http://localhost:5000")
    
    # Start ROS2 node in background thread
    ros_thread = threading.Thread(target=run_ros_node, daemon=True)
    ros_thread.start()
    
    # Give ROS2 node time to initialize
    time.sleep(2)
    
    try:
        # Start Flask-SocketIO server
        socketio.run(app, host='0.0.0.0', port=5000, debug=False)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        if log_monitor:
            rclpy.shutdown()