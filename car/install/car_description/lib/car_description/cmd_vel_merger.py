#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class CmdVelMerger(Node):
    def __init__(self):
        super().__init__('cmd_vel_merger')
        
        # QoS profile - use RELIABLE for compatibility with Gazebo
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribers
        self.nav_subscription = self.create_subscription(
            Twist,
            'cmd_vel_nav',
            self.nav_callback,
            qos
        )
        
        self.teleop_subscription = self.create_subscription(
            Twist,
            'cmd_vel_teleop',
            self.teleop_callback,
            qos
        )
        
        # Publisher
        self.publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            qos
        )
        
        # Store latest commands
        self.nav_cmd = None
        self.teleop_cmd = None
        self.last_teleop_time = None
        
        # Timer to publish merged command
        self.timer = self.create_timer(0.1, self.publish_merged_cmd)  # 10 Hz
        
        self.get_logger().info('CmdVel merger started. Merging cmd_vel_nav and cmd_vel_teleop')
    
    def nav_callback(self, msg):
        self.nav_cmd = msg
    
    def teleop_callback(self, msg):
        self.teleop_cmd = msg
        self.last_teleop_time = self.get_clock().now()
    
    def publish_merged_cmd(self):
        merged = Twist()
        
        # If teleop command received recently (within 0.5 seconds), prioritize it
        if self.teleop_cmd is not None and self.last_teleop_time is not None:
            time_since_teleop = (self.get_clock().now() - self.last_teleop_time).nanoseconds / 1e9
            if time_since_teleop < 0.5:
                merged = self.teleop_cmd
                self.get_logger().debug('Using teleop command')
            elif self.nav_cmd is not None:
                merged = self.nav_cmd
                self.get_logger().debug('Using nav command')
        elif self.nav_cmd is not None:
            merged = self.nav_cmd
            self.get_logger().debug('Using nav command (no teleop)')
        
        self.publisher.publish(merged)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelMerger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

