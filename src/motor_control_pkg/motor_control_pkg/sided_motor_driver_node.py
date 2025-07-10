#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)

class SidedMotorDriver(Node):
    def __init__(self):
        super().__init__('sided_motor_driver')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('speed_topic', '/motor/unconfigured'),
                ('en_a', -1), ('in1', -1), ('in2', -1),
                ('en_b', -1), ('in3', -1), ('in4', -1)
            ]
        )

        self.en_a = self.get_parameter('en_a').value
        self.in1 = self.get_parameter('in1').value
        self.in2 = self.get_parameter('in2').value
        self.en_b = self.get_parameter('en_b').value
        self.in3 = self.get_parameter('in3').value
        self.in4 = self.get_parameter('in4').value

        self._validate_pins()

        GPIO.setup([self.en_a, self.in1, self.in2, self.en_b, self.in3, self.in4], GPIO.OUT)

        self.pwm_a = GPIO.PWM(self.en_a, 1000)
        self.pwm_b = GPIO.PWM(self.en_b, 1000)
        self.pwm_a.start(0)
        self.pwm_b.start(0)

        topic = self.get_parameter('speed_topic').value
        self.subscription = self.create_subscription(Float32, topic, self.listener_callback, 10)

    def _validate_pins(self):
        pins = [self.en_a, self.in1, self.in2, self.en_b, self.in3, self.in4]
        for pin in pins:
            if pin < 0:
                raise ValueError(f"Invalid GPIO pin: {pin}")

    def listener_callback(self, msg):
        speed = max(min(msg.data, 1.0), -1.0)
        duty = abs(speed) * 100
        self.pwm_a.ChangeDutyCycle(duty)
        self.pwm_b.ChangeDutyCycle(duty)

        if speed > 0:
            GPIO.output(self.in1, True)
            GPIO.output(self.in2, False)
            GPIO.output(self.in3, True)
            GPIO.output(self.in4, False)
        elif speed < 0:
            GPIO.output(self.in1, False)
            GPIO.output(self.in2, True)
            GPIO.output(self.in3, False)
            GPIO.output(self.in4, True)
        else:
            GPIO.output([self.in1, self.in2, self.in3, self.in4], False)

    def destroy_node(self):
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SidedMotorDriver()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

