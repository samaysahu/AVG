#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from std_msgs.msg import String # Import String message type
from action_msgs.msg import GoalStatus # Import GoalStatus
import threading

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')
        self.get_logger().info('Waypoint Follower Node has been started.')

        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('Action server ready!')

        # Create a subscriber for room navigation commands
        self.room_command_subscriber = self.create_subscription(
            String,
            'navigate_room',
            self.room_command_callback,
            10
        )
        self.get_logger().info('Subscribing to /navigate_room topic.')

        # Define room positions (x, y, orientation_z, orientation_w)
        self.rooms = {
            '1': {
                'name': 'Room 1',
                'position': (-4.60574, -6.06122, 0.720155, 0.693813)
            },
            '2': {
                'name': 'Room 2',
                'position': (-4.57513, 5.89309, 0.134427, 0.990924)
            },
            '3': {
                'name': 'Room 3',
                'position': (3.35956, 6.51037, 0.089318, 0.996003)
            },
            '4': {
                'name': 'Room 4',
                'position': (5.31544, -6.15778, 0.617646, 0.786456)
            },
            '0': {
                'name': 'Home',
                'position': (0.0, 0.0, 0.0, 1.0)
            }
        }
        
        self.is_navigating = False
        self.current_goal_handle = None # To store the current goal handle for cancellation

    def room_command_callback(self, msg):
        room_key = msg.data
        self.get_logger().info(f'Received command to navigate to room: {room_key}')
        if room_key == 'stop':
            self.cancel_navigation()
        elif self.is_navigating:
            self.get_logger().warn('Robot is currently navigating. Please wait for current goal to complete or send an emergency stop.')
        else:
            self.navigate_to_room(room_key)

    def send_goal(self, x, y, orientation_z, orientation_w):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = orientation_z
        goal_msg.pose.pose.orientation.w = orientation_w

        self.get_logger().info(f'Sending goal: x={x:.2f}, y={y:.2f}')
        self.is_navigating = True
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected!')
            self.is_navigating = False
            return

        self.get_logger().info('Goal accepted! Robot is moving...')
        self.current_goal_handle = goal_handle # Store the goal handle
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        # result = future.result().result # The result object itself is not directly used here

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal reached successfully!')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('Goal was canceled!')
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error('Goal was aborted!')
        else:
            self.get_logger().error(f'Goal returned with status: {status}')

        self.is_navigating = False
        self.current_goal_handle = None

    def navigate_to_room(self, room_key):
        if room_key in self.rooms:
            room = self.rooms[room_key]
            self.get_logger().info(f'Navigating to {room["name"]}...')
            x, y, oz, ow = room['position']
            self.send_goal(x, y, oz, ow)
            return True
        else:
            self.get_logger().error(f'Invalid room selection: {room_key}!')
            return False

    def cancel_navigation(self):
        if self.is_navigating and self.current_goal_handle:
            self.get_logger().info('Cancelling current navigation goal...')
            future = self.current_goal_handle.cancel_goal_async()
            future.add_done_callback(self.cancel_goal_response)
        else:
            self.get_logger().info('No active navigation goal to cancel.')

    def cancel_goal_response(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal successfully cancelled.')
        else:
            self.get_logger().info('Goal cancellation failed.')
        self.is_navigating = False
        self.current_goal_handle = None

def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    
    print("\nWaypoint Follower is ready and awaiting commands!")
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()
