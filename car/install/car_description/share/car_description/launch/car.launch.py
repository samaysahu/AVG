from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():

    pkg_share = FindPackageShare(package='car_description')
    gazebo_ros_pkg_share = FindPackageShare(package='gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    xacro_file_name = 'newcar.xacro'
    world_file_name = 'my_car_world.world'

    world_path = PathJoinSubstitution([pkg_share, 'worlds', world_file_name])
    urdf_file_path = PathJoinSubstitution([pkg_share, 'urdf', xacro_file_name])
    
    robot_description_config = Command(['xacro ', urdf_file_path])

    # Create the rviz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([pkg_share, 'config', 'car.rviz'])]
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, 'launch', 'slam.launch.py'])
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

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
        rviz_node
    ])