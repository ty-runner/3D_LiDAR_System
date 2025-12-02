#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from adafruit_servokit import ServoKit

class ServoControlNode(Node):
    def __init__(self):
        super().__init__('servo_control_node')
        self.kit = ServoKit(channels=16)
        #self.kit.servo[15].actuation_range = 360
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/servo_angles',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        pan_angle, tilt_angle = msg.data
        pan_angle = max(0, min(180, pan_angle))
        tilt_angle = max(125, min(180, tilt_angle))

        self.kit.servo[14].angle = pan_angle
        self.kit.servo[15].angle = tilt_angle

        self.get_logger().info(f"Set Pan: {pan_angle}, Tilt: {tilt_angle}")

def main(args=None):
    rclpy.init(args=args)
    servo_node = ServoControlNode()
    rclpy.spin(servo_node)
    servo_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

