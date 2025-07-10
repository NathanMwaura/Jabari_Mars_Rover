code#!/usr/bin/env python3
"""
JABARI MARS ROVER - Manual Control Node
========================================

This node handles manual control commands for the rover, providing a bridge
between user input (web interface, joystick, or direct commands) and the
rover's command velocity system.

Author: Nathan
Node Purpose: Process manual control inputs and publish velocity commands
Communication: Publishes to /manual_cmd_vel topic for mode switching system

Dependencies:
- rclpy: ROS2 Python client library
- geometry_msgs: For Twist messages (velocity commands)
- std_msgs: For basic message types

Hardware Requirements:
- Raspberry Pi 4 (or compatible ROS2 system)
- Network connection for web interface (optional)

Topics:
- Publishers:
  * /manual_cmd_vel (geometry_msgs/Twist): Manual velocity commands
  
- Services: None (can be extended)
- Parameters: velocity_limits, deadband_threshold (can be added)

Usage:
This node acts as a communication gateway between manual control interfaces
and the rover's motion control system. It can receive commands from:
1. Web interface via method calls
2. Joystick input (extendable)
3. Direct ROS2 topic commands
4. Service calls (extendable)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool, Float32MultiArray
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray  # For pan-tilt commands
import time
import json
import threading
from typing import Optional, Tuple, Dict, Any


class ManualControlNode(Node):
    """
    Manual Control Node for JABARI Mars Rover
    
    This node processes manual control commands and publishes them as velocity
    commands that can be consumed by the mode switching system and ultimately
    sent to the motor controller.
    
    Features:
    - Velocity command validation and limiting
    - Command logging and monitoring
    - Emergency stop functionality
    - Configurable velocity limits
    - Status reporting
    - Thread-safe command processing
    """
    
    def __init__(self):
        """
        Initialize the Manual Control Node
        
        Sets up publishers, subscribers, parameters, and internal state
        for processing manual control commands.
        """
        super().__init__('manual_control_node')
        
        # ===== NODE CONFIGURATION =====
        self.setup_manual_control_parameters()
        self.load_parameters()
        
        # ===== QoS PROFILES =====
        # Reliable QoS for critical command messages
        reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
         # Joystick state
        self.joy_enabled = False  # Press to enable/disable joystick control
        self.last_button_states = []

        # Button mapping (customized for your controller)
        self.BUTTON_ENABLE = 9    # Enable/disable joystick control
        self.BUTTON_RESET_PAN_TILT = 8  # Reset pan-tilt to default

        # Axes mapping (customized for your controller)
        self.AXIS_LINEAR = 1      # Left stick vertical (forward/backward)
        self.AXIS_TURN = 3        # Right stick horizontal (turn left/right)
        self.AXIS_PAN = 5         # Right stick horizontal (pan left/right)
        self.AXIS_TILT = 6        # Right stick vertical (tilt up/down)

        # Camera pan-tilt state (0-180 degrees, start at center)
        self.pan_angle = 90
        self.tilt_angle = 90
        self.pan_tilt_step = 5  # Degrees to move per joystick input
        self.pan_tilt_min = 0    # Minimum pan/tilt angle
        self.pan_tilt_max = 180  # Maximum pan/tilt angle
       

        # QoS profile for joystick messages
        joystick_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # QoS profile for pan-tilt commands
        pan_tilt_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        # QoS profile for status messages
        status_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )     

        # ===== PUBLISHERS/SUBSCRIBERS =====
        
        # Subscribe to /joy topic for direct USB controller input
        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            joystick_qos
        )

        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/manual_cmd_vel',
            reliable_qos
        )

        # Publisher for camera pan-tilt (using Int32MultiArray: [pan, tilt])
        self.pan_tilt_pub = self.create_publisher(
            Int32MultiArray,
            '/camera_pan_tilt',
            pan_tilt_qos
        )
        

        # ===== PUBLISHERS =====
        # Main command velocity publisher for manual control
        self.cmd_vel_publisher = self.create_publisher(
            Twist, 
            '/manual_cmd_vel', 
            reliable_qos
        )
        
        # Status publisher for monitoring and debugging
        self.status_publisher = self.create_publisher(
            String,
            '/manual_control_status',
            status_qos
        )
        
        # ===== SUBSCRIBERS =====
        # Emergency stop subscriber (can be triggered by any node)
        self.emergency_stop_sub = self.create_subscription(
            Bool,
            '/emergency_stop',
            self.emergency_stop_callback,
            reliable_qos
        )
        
        # External command subscriber (for joystick, remote control, etc.)
        self.external_cmd_sub = self.create_subscription(
            Twist,
            '/external_manual_cmd',
            self.external_command_callback,
            reliable_qos
        )
        
        # ===== INTERNAL STATE =====
        self.current_linear_vel = 0.0
        self.current_angular_vel = 0.0
        self.emergency_stop_active = False
        self.last_command_time = time.time()
        self.command_count = 0
        self.is_active = True
        
        # Thread lock for thread-safe operations
        self.command_lock = threading.Lock()
        
        # ===== TIMERS =====
        # Status publishing timer (every 1 second)
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        # Command timeout timer (safety feature)
        self.timeout_timer = self.create_timer(0.1, self.check_command_timeout)
        
        # ===== INITIALIZATION COMPLETE =====
        self.get_logger().info(
            f'Manual Control Node initialized successfully\n'
            f'  - Max linear velocity: {self.max_linear_vel} m/s\n'
            f'  - Max angular velocity: {self.max_angular_vel} rad/s\n'
            f'  - Command timeout: {self.command_timeout} seconds\n'
            f'  - Publishing on: /manual_cmd_vel\n'
            f'  - Listening for emergency stop on: /emergency_stop'
        )
        
    def setup_manual_control_parameters(self):
        """
        Declare ROS2 parameters for the node
        
        Parameters allow dynamic configuration without code changes
        """
        # Velocity limits
        self.setup_manual_control_parameters('max_linear_velocity', 1.0)  # m/s
        self.setup_manual_control_parameters('max_angular_velocity', 2.0)  # rad/s

        # Safety parameters
        self.setup_manual_control_parameters('command_timeout', 2.0)  # seconds
        self.setup_manual_control_parameters('deadband_threshold', 0.01)  # minimum command value
        
        # Logging parameters
        self.setup_manual_control_parameters('enable_debug_logging', False)
        self.setup_manual_control_parameters('log_all_commands', False)
        
    def load_parameters(self):
        """
        Load parameters from ROS2 parameter server
        """
        self.max_linear_vel = self.get_parameter('max_linear_velocity').value
        self.max_angular_vel = self.get_parameter('max_angular_velocity').value
        self.command_timeout = self.get_parameter('command_timeout').value
        self.deadband_threshold = self.get_parameter('deadband_threshold').value
        self.debug_logging = self.get_parameter('enable_debug_logging').value
        self.log_all_commands = self.get_parameter('log_all_commands').value

    def joy_callback(self, msg: Joy):
        """
        Handle joystick input:
        - Press button 9 to toggle joystick control (press once to enable, again to disable)
        - Left stick (axes 1): negative = forward, positive = backward
        - Right stick (axes 3): positive = turn right, negative = turn left
        - Pan/tilt: axes 6 (positive = up, negative = down), axes 5 (positive = right, negative = left)
        - Press button 8 to reset pan/tilt to default (90, 90)
        """
        # Initialize last_button_states if first message
        if not self.last_button_states:
            self.last_button_states = [0] * len(msg.buttons)

        # --- Button press/release detection ---
        for idx, (prev, curr) in enumerate(zip(self.last_button_states, msg.buttons)):
            if curr == 1 and prev == 0:
                # Button pressed
                if idx == self.BUTTON_ENABLE:
                    self.joy_enabled = not self.joy_enabled
                    state = "ENABLED" if self.joy_enabled else "DISABLED"
                    self.get_logger().info(f"Joystick control {state} (Button 9 pressed)")
                    if not self.joy_enabled:
                        self.send_stop_command()
                elif idx == self.BUTTON_RESET_PAN_TILT:
                    self.pan_angle = 90
                    self.tilt_angle = 90
                    pan_tilt_msg = Int32MultiArray()
                    pan_tilt_msg.data = [self.pan_angle, self.tilt_angle]
                    self.pan_tilt_pub.publish(pan_tilt_msg)
                    self.get_logger().info("Pan-tilt reset to default (90, 90)")

        self.last_button_states = list(msg.buttons)

        # --- Robot movement (left and right sticks) ---
        if self.joy_enabled:
            # axes 1: negative = forward, positive = backward
            linear = -msg.axes[self.AXIS_LINEAR]
            # axes 3: positive = turn right, negative = turn left
            angular = msg.axes[self.AXIS_TURN]
            twist = Twist()
            twist.linear.x = linear
            twist.angular.z = angular
            self.cmd_vel_pub.publish(twist)

        # --- Camera pan-tilt control (right stick) ---
        # axes 6: positive = up, negative = down
        # axes 5: positive = right, negative = left
        pan_delta = msg.axes[self.AXIS_PAN] * 2    # Adjust sensitivity as needed
        tilt_delta = msg.axes[self.AXIS_TILT] * 2  # Positive = up, negative = down

        # Only update if stick is moved significantly
        if abs(pan_delta) > 0.1 or abs(tilt_delta) > 0.1:
            self.pan_angle = int(max(0, min(180, self.pan_angle + pan_delta)))
            self.tilt_angle = int(max(0, min(180, self.tilt_angle + tilt_delta)))
            pan_tilt_msg = Int32MultiArray()
            pan_tilt_msg.data = [self.pan_angle, self.tilt_angle]
            self.pan_tilt_pub.publish(pan_tilt_msg)
            self.get_logger().info(f"Camera pan/tilt: {self.pan_angle}, {self.tilt_angle}")

    def send_stop_command(self):
        """Send a zero velocity command to stop the robot."""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

    def toggle_mode(self):
        """User-defined mode switching logic."""
        pass
        
    def send_velocity_command(self, linear: float, angular: float) -> bool:
        """
        Send velocity command from external interface (web, API, etc.)
        
        Args:
            linear (float): Linear velocity in m/s (forward/backward)
            angular (float): Angular velocity in rad/s (rotation)
            
        Returns:
            bool: True if command was sent successfully, False otherwise
            
        This is the main interface method for external systems to send
        velocity commands to the rover.
        """
        try:
            with self.command_lock:
                # Validate and limit the command
                limited_linear, limited_angular = self._validate_and_limit_command(
                    linear, angular
                )
                
                # Check if emergency stop is active
                if self.emergency_stop_active:
                    self.get_logger().warn(
                        'Command rejected: Emergency stop is active'
                    )
                    return False
                
                # Apply deadband filter for stability
                if abs(limited_linear) < self.deadband_threshold:
                    limited_linear = 0.0
                if abs(limited_angular) < self.deadband_threshold:
                    limited_angular = 0.0
                
                # Create and publish the message
                success = self._publish_velocity_command(limited_linear, limited_angular)
                
                if success:
                    # Update internal state
                    self.current_linear_vel = limited_linear
                    self.current_angular_vel = limited_angular
                    self.last_command_time = time.time()
                    self.command_count += 1
                    
                    # Log command if enabled
                    if self.log_all_commands or self.debug_logging:
                        self.get_logger().info(
                            f'Velocity command sent: linear={limited_linear:.3f}, '
                            f'angular={limited_angular:.3f} '
                            f'(original: {linear:.3f}, {angular:.3f})'
                        )
                
                return success
                
        except Exception as e:
            self.get_logger().error(f'Error sending velocity command: {e}')
            return False
    
    def _validate_and_limit_command(self, linear: float, angular: float) -> Tuple[float, float]:
        """
        Validate and limit velocity commands to safe ranges
        
        Args:
            linear (float): Requested linear velocity
            angular (float): Requested angular velocity
            
        Returns:
            Tuple[float, float]: Limited linear and angular velocities
        """
        # Limit linear velocity
        limited_linear = max(-self.max_linear_vel, 
                           min(self.max_linear_vel, float(linear)))
        
        # Limit angular velocity
        limited_angular = max(-self.max_angular_vel, 
                            min(self.max_angular_vel, float(angular)))
        
        # Log if limiting occurred
        if (abs(limited_linear - linear) > 0.001 or 
            abs(limited_angular - angular) > 0.001):
            self.get_logger().warn(
                f'Velocity command limited: '
                f'Original: ({linear:.3f}, {angular:.3f}), '
                f'Limited: ({limited_linear:.3f}, {limited_angular:.3f})'
            )
        
        return limited_linear, limited_angular
    
    def _publish_velocity_command(self, linear: float, angular: float) -> bool:
        """
        Publish velocity command as ROS2 Twist message
        
        Args:
            linear (float): Linear velocity in m/s
            angular (float): Angular velocity in rad/s
            
        Returns:
            bool: True if published successfully
        """
        try:
            msg = Twist()
            msg.linear.x = linear
            msg.linear.y = 0.0  # No lateral movement for differential drive
            msg.linear.z = 0.0  # No vertical movement
            
            msg.angular.x = 0.0  # No roll
            msg.angular.y = 0.0  # No pitch
            msg.angular.z = angular  # Yaw rotation
            
            # Add timestamp information (custom)
            # Note: Twist messages don't have headers, but we can log timing
            current_time = self.get_clock().now()
            
            self.cmd_vel_publisher.publish(msg)
            
            if self.debug_logging:
                self.get_logger().debug(
                    f'Published Twist message: linear.x={linear:.3f}, '
                    f'angular.z={angular:.3f} at time {current_time}'
                )
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'Failed to publish velocity command: {e}')
            return False
    
    def emergency_stop_callback(self, msg: Bool):
        """
        Handle emergency stop commands
        
        Args:
            msg (Bool): True to activate emergency stop, False to deactivate
        """
        self.emergency_stop_active = msg.data
        
        if self.emergency_stop_active:
            self.get_logger().warn('EMERGENCY STOP ACTIVATED - All commands stopped')
            # Send zero velocity command immediately
            self._publish_velocity_command(0.0, 0.0)
            self.current_linear_vel = 0.0
            self.current_angular_vel = 0.0
        else:
            self.get_logger().info('Emergency stop deactivated - Manual control resumed')
    
    def external_command_callback(self, msg: Twist):
        """
        Handle velocity commands from external sources (joystick, remote control)
        
        Args:
            msg (Twist): Velocity command from external source
        """
        # Process external command through the same validation system
        self.send_velocity_command(msg.linear.x, msg.angular.z)
    
    def stop_rover(self) -> bool:
        """
        Immediately stop the rover (send zero velocity)
        
        Returns:
            bool: True if stop command was sent successfully
        """
        self.get_logger().info('Stop command received - stopping rover')
        return self.send_velocity_command(0.0, 0.0)
    
    def check_command_timeout(self):
        """
        Check if commands have timed out and stop rover if necessary
        
        This safety feature ensures the rover stops if no commands are
        received within the timeout period.
        """
        current_time = time.time()
        time_since_last_command = current_time - self.last_command_time
        
        if (time_since_last_command > self.command_timeout and 
            (self.current_linear_vel != 0.0 or self.current_angular_vel != 0.0)):
            
            self.get_logger().warn(
                f'Command timeout after {time_since_last_command:.1f}s - stopping rover'
            )
            self._publish_velocity_command(0.0, 0.0)
            self.current_linear_vel = 0.0
            self.current_angular_vel = 0.0
    
    def publish_status(self):
        """
        Publish current status information for monitoring
        """
        try:
            status_data = {
                'node_name': 'manual_control_node',
                'timestamp': time.time(),
                'is_active': self.is_active,
                'emergency_stop_active': self.emergency_stop_active,
                'current_linear_velocity': self.current_linear_vel,
                'current_angular_velocity': self.current_angular_vel,
                'command_count': self.command_count,
                'time_since_last_command': time.time() - self.last_command_time,
                'parameters': {
                    'max_linear_velocity': self.max_linear_vel,
                    'max_angular_velocity': self.max_angular_vel,
                    'command_timeout': self.command_timeout
                }
            }
            
            msg = String()
            msg.data = json.dumps(status_data)
            self.status_publisher.publish(msg)
            
            if self.debug_logging:
                self.get_logger().debug(f'Status published: {json.dumps(status_data, indent=2)}')
                
        except Exception as e:
            self.get_logger().error(f'Error publishing status: {e}')
    
    def get_current_velocity(self) -> Tuple[float, float]:
        """
        Get current velocity command values
        
        Returns:
            Tuple[float, float]: Current linear and angular velocities
        """
        with self.command_lock:
            return self.current_linear_vel, self.current_angular_vel
    
    def set_velocity_limits(self, max_linear: float, max_angular: float) -> bool:
        """
        Dynamically update velocity limits
        
        Args:
            max_linear (float): New maximum linear velocity
            max_angular (float): New maximum angular velocity
            
        Returns:
            bool: True if limits were updated successfully
        """
        try:
            self.max_linear_vel = max(0.0, float(max_linear))
            self.max_angular_vel = max(0.0, float(max_angular))
            
            self.get_logger().info(
                f'Velocity limits updated: linear={self.max_linear_vel}, '
                f'angular={self.max_angular_vel}'
            )
            return True
            
        except Exception as e:
            self.get_logger().error(f'Error updating velocity limits: {e}')
            return False
    
    def shutdown_gracefully(self):
        """
        Perform graceful shutdown of the node
        """
        self.get_logger().info('Manual Control Node shutting down...')
        
        # Stop the rover
        self.stop_rover()
        
        # Set inactive state
        self.is_active = False
        
        # Publish final status
        self.publish_status()
        
        self.get_logger().info('Manual Control Node shutdown complete')


def main(args=None):
    """
    Main entry point for the manual control node
    
    Args:
        args: Command line arguments (optional)
    """
    # Initialize ROS2
    rclpy.init(args=args)
    
    try:
        # Create and run the node
        manual_control_node = ManualControlNode()
        
        # Use MultiThreadedExecutor for better performance with timers
        from rclpy.executors import MultiThreadedExecutor
        executor = MultiThreadedExecutor()
        
        # Add node to executor
        executor.add_node(manual_control_node)
        
        # Spin the node
        try:
            manual_control_node.get_logger().info('Manual Control Node is running...')
            executor.spin()
        except KeyboardInterrupt:
            manual_control_node.get_logger().info('Keyboard interrupt received')
        finally:
            # Graceful shutdown
            manual_control_node.shutdown_gracefully()
            
    except Exception as e:
        print(f'Error starting manual control node: {e}')
    finally:
        # Cleanup
        try:
            manual_control_node.destroy_node()
        except:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()