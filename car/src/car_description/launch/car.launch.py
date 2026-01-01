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
    use_joint_state_gui = LaunchConfiguration('use_joint_state_gui', default='false')  # GUI for controlling arm joints
    car_xacro_file_name = 'newcar.xacro'
    arm_xacro_file_name = 'robotiarm.xacro'
    world_file_name = 'main'
    
    # Arm position in the world (adjust these to place arm in different room)
    # These must match between static transform and Gazebo spawn
    # IMPORTANT: This is the position of arm_world (root link) in the map frame
    # The URDF joint offsets base_link by (3, 0, 0.075) from arm_world
    # So if arm_world is at (x, y, z), base_link will be at (x+3, y, z+0.075)
    # Car is at (5.0, 5.0, 0.5) - arm should be in a different location
    # Setting arm_world to (-2.0, 0.5, 0.0) so base_link ends up at (1.0, 0.5, 0.075) - away from car
    arm_x_pos = '-2.0'  # X position of arm_world in map frame (base_link will be at -2.0+3=1.0)
    arm_y_pos = '0.5'   # Y position of arm_world in map frame  
    arm_z_pos = '0.0'   # Z position of arm_world in map frame (base_link will be at 0.0+0.075=0.075)

    world_path = PathJoinSubstitution([pkg_share, 'car_world', world_file_name])
    car_urdf_file_path = PathJoinSubstitution([pkg_share, 'urdf', car_xacro_file_name])
    arm_urdf_file_path = PathJoinSubstitution([pkg_share, 'urdf', arm_xacro_file_name])
    map_file_path = PathJoinSubstitution([pkg_share, 'map', 'my_map.yaml'])
    
    car_robot_description_config = Command(['xacro ', car_urdf_file_path])
    arm_robot_description_config = Command(['xacro ', arm_urdf_file_path])

    # Create the rviz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
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

    # Static transform from map to arm_world (arm's root frame)
    # This must match the Gazebo spawn position exactly
    # Note: The arm's root frame is 'arm_world' (from the URDF) to avoid conflicts
    # The URDF joint offsets base_link by (3, 0, 0.075) from arm_world
    # So arm_world position determines where base_link will be: base_link = arm_world + (3, 0, 0.075)
    static_tf_map_to_arm_world = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_map_to_arm_world',
        arguments=[
            '--x', arm_x_pos, '--y', arm_y_pos, '--z', arm_z_pos,
            '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
            '--frame-id', 'map', '--child-frame-id', 'arm_world'
        ]
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
        period=10.0,  # Wait for map_server and transforms to be ready
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
        DeclareLaunchArgument(
            'use_joint_state_gui',
            default_value='false',
            description='Use joint_state_publisher_gui for controlling arm joints'),

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

        # Robot state publisher for car
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': car_robot_description_config}]
        ),

        # Joint state publisher for car
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'use_sim_time': use_sim_time}],
            condition=UnlessCondition(use_joint_state_gui)
        ),

        # Joint state publisher GUI for controlling arm joints
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(use_joint_state_gui)
        ),

        # Spawn car in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'my_robot', '-x', '5.0', '-y', '5.0', '-z', '0.5'],
            output='screen'
        ),

        # Robot state publisher for arm (publishes to /arm/robot_description)
        # Note: Not using frame_prefix to keep URDF frame names consistent
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='arm_robot_state_publisher',
            namespace='arm',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': arm_robot_description_config}],
            remappings=[('/arm/joint_states', '/arm/joint_states')]
        ),

        # Joint state publisher for arm
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='arm_joint_state_publisher',
            namespace='arm',
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[('/arm/joint_states', '/arm/joint_states')],
            condition=UnlessCondition(use_joint_state_gui)
        ),

        # Joint state publisher GUI for arm (when enabled)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='arm_joint_state_publisher_gui',
            namespace='arm',
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[('/arm/joint_states', '/arm/joint_states')],
            condition=IfCondition(use_joint_state_gui)
        ),

        # Spawn arm in Gazebo (different room - adjust arm_x_pos, arm_y_pos, arm_z_pos variables above)
        # Position must match static_tf_map_to_arm_world exactly
        # The URDF joint offsets base_link by 0.075m, so spawning arm_world at z=0.0 means base_link sits at z=0.075
        # Using TimerAction to ensure robot_state_publisher is ready
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    arguments=['-topic', '/arm/robot_description', '-entity', 'robotic_arm', '-x', arm_x_pos, '-y', arm_y_pos, '-z', arm_z_pos],
                    output='screen'
                )
            ]
        ),

        slam_launch,
        map_server_node,
        lifecycle_manager_node,
        static_tf_map_to_odom,
        static_tf_map_to_arm_world,
        map_republisher_node,
        cmd_vel_merger_node,
        nav2_bringup,
        rviz_node
    ])