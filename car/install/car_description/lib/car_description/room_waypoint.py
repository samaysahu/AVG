#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import threading

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')
        self.get_logger().info('Waypoint Follower Node has been started.')

        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('Action server ready!')

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
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Goal reached successfully!')
        self.is_navigating = False

    def navigate_to_room(self, room_key):
        if room_key in self.rooms:
            room = self.rooms[room_key]
            self.get_logger().info(f'Navigating to {room["name"]}...')
            x, y, oz, ow = room['position']
            self.send_goal(x, y, oz, ow)
            return True
        else:
            print('Invalid room selection!')
            return False

def display_menu():
    print("\n" + "="*50)
    print("     ROBOT NAVIGATION SYSTEM")
    print("="*50)
    print("Please select a destination:")
    print("  [1] Room 1")
    print("  [2] Room 2")
    print("  [3] Room 3")
    print("  [4] Room 4")
    print("  [0] Home")
    print("  [q] Quit")
    print("="*50)

def user_input_thread(node):
    """Separate thread to handle user input without blocking ROS2 spinning"""
    while rclpy.ok():
        display_menu()
        choice = input("Enter your choice: ").strip().lower()
        
        if choice == 'q':
            print("Shutting down...")
            rclpy.shutdown()
            break
        elif choice in ['0', '1', '2', '3', '4']:
            if node.is_navigating:
                print("Robot is currently navigating. Please wait...")
            else:
                node.navigate_to_room(choice)
        else:
            print("Invalid choice! Please try again.")

def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    
    # Start user input in a separate thread
    input_thread = threading.Thread(target=user_input_thread, args=(node,), daemon=True)
    input_thread.start()
    
    print("\nWaypoint Follower is ready!")
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()