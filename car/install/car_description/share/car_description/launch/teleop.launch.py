import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    config_filepath = os.path.join(get_package_share_directory('car_description'), 'config', 'joystick.yaml')

    return LaunchDescription([
        Node(
            package='car_description',
            executable='simple_car_teleop.py',
            name='joystick_node',
            output='screen'
        ),
        Node(
            package='car_description',
            executable='car_teleop_node.py',
            name='car_teleop_node',
            output='screen'
        ),
    ])