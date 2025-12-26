#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import pygame
import sys
import threading


class JoystickNode(Node):
    def __init__(self):
        super().__init__('joystick_node')

        # Publisher
        self.publisher_ = self.create_publisher(Joy, 'joy', 10)

        # Initialize pygame joystick
        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            self.get_logger().error("No joystick detected!")
            sys.exit(1)

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()

        self.get_logger().info(
            f"Joystick connected: {self.joystick.get_name()}"
        )

        # Thread for pygame loop
        self.thread = threading.Thread(target=self.joystick_loop)
        self.thread.daemon = True
        self.thread.start()

    def joystick_loop(self):
        clock = pygame.time.Clock()
        pygame.event.set_allowed([pygame.JOYAXISMOTION, pygame.JOYBUTTONDOWN, pygame.JOYBUTTONUP])

        while rclpy.ok():
            pygame.event.pump()

            joy_msg = Joy()
            joy_msg.header.stamp = self.get_clock().now().to_msg()

            # Read axes
            joy_msg.axes = [
                self.joystick.get_axis(i)
                for i in range(self.joystick.get_numaxes())
            ]

            # Read buttons
            joy_msg.buttons = [
                self.joystick.get_button(i)
                for i in range(self.joystick.get_numbuttons())
            ]

            self.publisher_.publish(joy_msg)

            clock.tick(50)  # 50 Hz


def main(args=None):
    rclpy.init(args=args)
    node = JoystickNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    pygame.quit()


if __name__ == '__main__':
    main()
