#!/usr/bin/env python3
"""
ROS2 Web Bridge Server
Connects web interface to ROS2 system
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
import asyncio
import websockets
import json
import threading
from flask import Flask, request, jsonify
from flask_cors import CORS

class ROS2WebBridge(Node):
    def __init__(self):
        super().__init__('ros2_web_bridge')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/manual_cmd_vel', 10)
        self.emergency_stop_pub = self.create_publisher(Bool, '/emergency_stop', 10)
        
        # Subscribers for status feedback
        self.status_sub = self.create_subscription(
            String, '/manual_control_status', self.status_callback, 10
        )
        
        self.latest_status = {}
        
        # Flask app for HTTP API
        self.app = Flask(__name__)
        CORS(self.app)
        
        self.setup_routes()
        
    def setup_routes(self):
        @self.app.route('/api/move', methods=['POST'])
        def move_robot():
            try:
                data = request.json
                linear = data.get('linear', 0.0)
                angular = data.get('angular', 0.0)
                
                twist = Twist()
                twist.linear.x = float(linear)
                twist.angular.z = float(angular)
                
                self.cmd_vel_pub.publish(twist)
                
                return jsonify({'status': 'success'})
            except Exception as e:
                return jsonify({'status': 'error', 'message': str(e)})
        
        @self.app.route('/api/stop', methods=['POST'])
        def stop_robot():
            try:
                twist = Twist()
                self.cmd_vel_pub.publish(twist)
                return jsonify({'status': 'success'})
            except Exception as e:
                return jsonify({'status': 'error', 'message': str(e)})
        
        @self.app.route('/api/emergency_stop', methods=['POST'])
        def emergency_stop():
            try:
                data = request.json
                active = data.get('active', True)
                
                msg = Bool()
                msg.data = active
                self.emergency_stop_pub.publish(msg)
                
                return jsonify({'status': 'success'})
            except Exception as e:
                return jsonify({'status': 'error', 'message': str(e)})
        
        @self.app.route('/api/status', methods=['GET'])
        def get_status():
            return jsonify(self.latest_status)
    
    def status_callback(self, msg):
        try:
            self.latest_status = json.loads(msg.data)
        except:
            self.latest_status = {'raw': msg.data}
    
    def run_flask(self):
        self.app.run(host='0.0.0.0', port=5000, debug=False)

def main():
    rclpy.init()
    bridge = ROS2WebBridge()
    
    # Run Flask in separate thread
    flask_thread = threading.Thread(target=bridge.run_flask)
    flask_thread.daemon = True
    flask_thread.start()
    
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()