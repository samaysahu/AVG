#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import time

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')
        self.get_logger().info('Waypoint Follower Node has been started.')

        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('Action server found. Waiting a moment for full readiness...')
        time.sleep(1) # Give the action server a moment to fully initialize

        # Define waypoints (x, y, yaw)
        # These are placeholder coordinates. You will need to adjust them based on your map.
        # The last waypoint should ideally be the starting point to return home.
        self.waypoints = [
            # Go to the specified point
            (4.47208, 6.44691, 0.520654),
            # Return to home (assuming home is 0,0,0)
            (0.0, 0.0, 0.0)
        ]
        self.current_waypoint_index = 0

    def send_goal(self, x, y, yaw):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = yaw # Simplified for 2D navigation, assuming yaw is directly z
        goal_msg.pose.pose.orientation.w = 1.0 # Assuming no pitch/roll

        self.get_logger().info(f'Sending goal: x={x}, y={y}, yaw={yaw}')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            self.current_waypoint_index += 1
            self.follow_next_waypoint()
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Goal reached: {result}')
        self.current_waypoint_index += 1
        self.follow_next_waypoint()

    def follow_next_waypoint(self):
        if self.current_waypoint_index < len(self.waypoints):
            x, y, yaw = self.waypoints[self.current_waypoint_index]
            self.send_goal(x, y, yaw)
        else:
            self.get_logger().info('All waypoints followed. Returning home.')
            # Optionally, you can add logic here to explicitly send the car back to a known home position
            # if the last waypoint wasn't exactly the start.
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    node.follow_next_waypoint() # Start following waypoints
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()