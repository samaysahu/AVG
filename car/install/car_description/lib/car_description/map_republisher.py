#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy

class MapRepublisher(Node):
    def __init__(self):
        super().__init__('map_republisher')
        
        # Create QoS profile for subscribing (TRANSIENT_LOCAL to receive from map_server)
        subscribe_qos = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Create QoS profile for publishing (VOLATILE for RViz compatibility)
        publish_qos = QoSProfile(
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribe to the map from map_server
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            subscribe_qos
        )
        
        # Publish the map with compatible QoS
        self.publisher = self.create_publisher(
            OccupancyGrid,
            '/map_republished',
            publish_qos
        )
        
        self.get_logger().info('Map republisher started. Republishing /map to /map_republished')
    
    def map_callback(self, msg):
        # Republish the map with new QoS
        self.publisher.publish(msg)
        self.get_logger().info('Map republished', throttle_duration_sec=5.0)

def main(args=None):
    rclpy.init(args=args)
    node = MapRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

