#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class CarTeleopNode(Node):
    def __init__(self):
        super().__init__('car_teleop_node')

        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )

        self.get_logger().info("✅ Joystick-only Car Teleop (FIXED)")

        # ✅ CORRECT LEFT STICK AXES
        self.axis_angular = 0   # Left stick LEFT / RIGHT
        self.axis_linear  = 1   # Left stick UP / DOWN

        # Speed limits
        self.max_linear_speed = 0.6     # m/s
        self.max_angular_speed = 1.2    # rad/s

        self.deadzone = 0.15

    def joy_callback(self, msg):
        twist = Twist()

        # Read LEFT STICK
        raw_angular = msg.axes[self.axis_angular]
        raw_linear = msg.axes[self.axis_linear]

        # Deadzone
        if abs(raw_linear) < self.deadzone:
            raw_linear = 0.0
        if abs(raw_angular) < self.deadzone:
            raw_angular = 0.0

        # Correct direction (UP = forward)
        twist.linear.x = -raw_linear * self.max_linear_speed
        twist.angular.z = (raw_angular**3) * self.max_angular_speed

        self.publisher_.publish(twist)

        self.get_logger().info(
            f"cmd_vel → linear={twist.linear.x:.2f}, angular={twist.angular.z:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = CarTeleopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
