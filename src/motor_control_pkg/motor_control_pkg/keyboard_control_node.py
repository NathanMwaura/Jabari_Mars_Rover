#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty


class KeyboardControl(Node):
    def __init__(self):
        super().__init__('keyboard_control')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Initialize speed parameters
        self.speed = 0.5  # Default linear speed (m/s)
        self.turn = 0.5   # Default angular speed (rad/s)
        
        # Display control instructions
        self.get_logger().info(
            'Keyboard Controls:\n'
            '  W/S: Move forward/backward\n'
            '  A/D: Turn left/right\n'
            '  U/J: Increase/decrease linear speed\n'
            '  I/K: Increase/decrease angular speed\n'
            '  Q: Quit\n'
            f'Current speeds - Linear: {self.speed:.1f} m/s, Angular: {self.turn:.1f} rad/s'
        )

        self.run_control_loop()

    def run_control_loop(self):
        """Main control loop processing keyboard input"""
        while True:
            key = self.get_key()
            twist = Twist()

            if key == 'w':       # Forward
                twist.linear.x = self.speed
            elif key == 's':     # Backward
                twist.linear.x = -self.speed
            elif key == 'a':     # Turn left
                twist.angular.z = self.turn
            elif key == 'd':     # Turn right
                twist.angular.z = -self.turn
            elif key == 'u':    # Increase linear speed
                self.speed = min(1.0, self.speed + 0.1)
                self.get_logger().info(f'Linear speed: {self.speed:.1f} m/s')
            elif key == 'j':     # Decrease linear speed
                self.speed = max(0.0, self.speed - 0.1)
                self.get_logger().info(f'Linear speed: {self.speed:.1f} m/s')
            elif key == 'i':     # Increase angular speed
                self.turn = min(1.0, self.turn + 0.1)
                self.get_logger().info(f'Angular speed: {self.turn:.1f} rad/s')
            elif key == 'k':     # Decrease angular speed
                self.turn = max(0.0, self.turn - 0.1)
                self.get_logger().info(f'Angular speed: {self.turn:.1f} rad/s')
            elif key == 'q':     # Quit
                self.get_logger().info('Shutting down...')
                break
            else:                # Stop for any other key
                pass

            self.pub.publish(twist)

    def get_key(self):
        """Get single key press without requiring Enter"""
        tty.setraw(sys.stdin.fileno())
        try:
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(
                sys.stdin, 
                termios.TCSADRAIN, 
                termios.tcgetattr(sys.stdin)
            )
        return key


def main():
    rclpy.init()
    try:
        keyboard_control = KeyboardControl()
        rclpy.spin(keyboard_control)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
