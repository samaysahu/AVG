import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get package share directory (e.g., /home/samay/AVG/car/install/car_description/share/car_description)
    car_description_share_dir = get_package_share_directory('car_description')
    
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_yaml_file = LaunchConfiguration('map', default=os.path.join(
        car_description_share_dir, 'map', 'my_map.yaml'))
    
    # Include the car.launch.py (Gazebo and RViz)
    car_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(car_description_share_dir, 'launch', 'car.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Include the nav2_bringup.launch.py
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(car_description_share_dir, 'launch', 'nav2_bringup.launch.py')),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': os.path.join(car_description_share_dir, 'config', 'nav2_params.yaml') 
        }.items()
    )

    # Waypoint Follower Node
    waypoint_follower_node = Node(
        package='car_description',
        executable='room_waypoint.py',
        name='waypoint_follower',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # RQT GUI Node
    # Launch rqt_gui and automatically select the plugin
    rqt_gui_node = ExecuteProcess(
        cmd=['ros2', 'run', 'rqt_gui', 'rqt_gui', '--select-plugin', 'car_rqt_gui::CarRqtGuiPlugin'],
        output='screen',
        shell=True
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(car_description_share_dir, 'map', 'my_map.yaml'),
            description='Full path to map yaml file to load'),
        
        car_launch,
        nav2_bringup_launch,
        waypoint_follower_node,
        rqt_gui_node,
    ])
