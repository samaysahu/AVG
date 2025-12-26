from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():

    pkg_share = FindPackageShare(package='car_description')
    gazebo_ros_pkg_share = FindPackageShare(package='gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_slam = LaunchConfiguration('use_slam', default='false')  # Disable SLAM when using static map
    xacro_file_name = 'newcar.xacro'
    world_file_name = 'my_car_world.world'

    world_path = PathJoinSubstitution([pkg_share, 'worlds', world_file_name])
    urdf_file_path = PathJoinSubstitution([pkg_share, 'urdf', xacro_file_name])
    map_file_path = PathJoinSubstitution([pkg_share, 'map', 'my_map.yaml'])
    
    robot_description_config = Command(['xacro ', urdf_file_path])

    # Create the rviz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([pkg_share, 'config', 'car.rviz'])]
    )

    # Conditionally include SLAM (disabled by default when using static map)
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, 'launch', 'slam.launch.py'])
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(use_slam)
    )

    # Map server to load and publish the static map
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': map_file_path,
            'frame_id': 'map'  # Explicitly set the map frame
        }]
    )

    # Lifecycle manager to activate map_server (with delay to ensure map_server is ready)
    lifecycle_manager_node = TimerAction(
        period=5.0,  # Wait 5 seconds for map_server to be fully created and ready
        actions=[
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_map_server',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'autostart': True,
                    'node_names': ['map_server'],
                    'bond_timeout': 10.0  # Increase timeout for state transitions
                }]
            )
        ]
    )

    # Static transform from map to odom (only when SLAM is disabled)
    # This is a temporary identity transform - proper localization (AMCL) will replace this
    static_tf_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_map_to_odom',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
            '--frame-id', 'map', '--child-frame-id', 'odom'
        ],
        condition=UnlessCondition(use_slam)  # Only publish when SLAM is disabled
    )

    # Map republisher to republish map with RViz-compatible QoS
    map_republisher_node = TimerAction(
        period=4.0,  # Wait for map_server to be activated
        actions=[
            Node(
                package='car_description',
                executable='map_republisher.py',
                name='map_republisher',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )

    # Nav2 bringup
    nav2_bringup = TimerAction(
        period=5.0,  # Wait for map_server and transforms to be ready
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([pkg_share, 'launch', 'nav2_bringup.launch.py'])
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'autostart': 'true'
                }.items()
            )
        ]
    )

    # Cmd_vel merger to combine navigation and teleop commands
    cmd_vel_merger_node = Node(
        package='car_description',
        executable='cmd_vel_merger.py',
        name='cmd_vel_merger',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'use_slam',
            default_value='false',
            description='Enable SLAM (disable when using static map)'),

        SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=[
                PathJoinSubstitution([pkg_share, '..']),
                os.path.join(os.getenv('GAZEBO_MODEL_PATH', ''), '')
            ]
        ),
        SetEnvironmentVariable(
            name='GAZEBO_PLUGIN_PATH',
            value=[
                PathJoinSubstitution([pkg_share, '..', 'lib']),
                os.path.join(os.getenv('GAZEBO_PLUGIN_PATH', ''), '')
            ]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([gazebo_ros_pkg_share, 'launch', 'gazebo.launch.py'])
            ),
            launch_arguments={'world': world_path}.items()
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description_config}]
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'my_robot'],
            output='screen'
        ),

        slam_launch,
        map_server_node,
        lifecycle_manager_node,
        static_tf_map_to_odom,
        map_republisher_node,
        cmd_vel_merger_node,
        nav2_bringup,
        rviz_node
    ])