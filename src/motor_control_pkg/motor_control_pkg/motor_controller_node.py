#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32


class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        self.sub = self.create_subscription(
            Twist, '/cmd_vel', self.listener_callback, 10
        )
        self.pub_left = self.create_publisher(Float32, '/motor/left_speed', 10)
        self.pub_right = self.create_publisher(Float32, '/motor/right_speed', 10)

        # Distance between the left and right wheels (in meters)
        self.wheel_base = 0.3  # Adjust this value according to your robot

    def listener_callback(self, msg):
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z

        # Differential drive kinematics calculations
        left_speed = linear_vel - (angular_vel * self.wheel_base / 2.0)
        right_speed = linear_vel + (angular_vel * self.wheel_base / 2.0)

        # Create and publish left wheel speed message
        left_msg = Float32()
        left_msg.data = float(left_speed)
        self.pub_left.publish(left_msg)

        # Create and publish right wheel speed message
        right_msg = Float32()
        right_msg.data = float(right_speed)
        self.pub_right.publish(right_msg)


def main():
    rclpy.init()
    controller = MotorController()
    rclpy.spin(controller)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
