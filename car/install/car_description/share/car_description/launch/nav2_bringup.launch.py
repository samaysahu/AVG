import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    pkg_share = get_package_share_directory('car_description')
    
    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    use_lifecycle_mgr = LaunchConfiguration('use_lifecycle_mgr')
    map_subscribe_transient_local = LaunchConfiguration('map_subscribe_transient_local')

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_share, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_use_lifecycle_mgr_cmd = DeclareLaunchArgument(
        'use_lifecycle_mgr',
        default_value='true',
        description='Whether to launch the lifecycle manager')

    declare_map_subscribe_transient_local_cmd = DeclareLaunchArgument(
        'map_subscribe_transient_local', default_value='false',
        description='Whether to set the map subscriber QoS to transient local')

    # Use the params file directly (no namespace or rewrites needed)
    # RewrittenYaml doesn't work well with LaunchConfiguration, so use the default path directly
    default_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    # Nodes launching commands
    lifecycle_nodes = ['controller_server',
                       'planner_server',
                       'bt_navigator',
                       'waypoint_follower',
                       'velocity_smoother']

    # Start the lifecycle manager
    start_lifecycle_manager_cmd = Node(
        condition=IfCondition(use_lifecycle_mgr),
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': lifecycle_nodes}])

    # Start the navigation nodes
    start_nav2_controller_cmd = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[default_params_file, {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel', 'cmd_vel_nav')])

    start_nav2_planner_cmd = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[default_params_file, {'use_sim_time': use_sim_time}])

    start_nav2_recoveries_cmd = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[default_params_file, {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel', 'cmd_vel_nav')])

    start_nav2_bt_navigator_cmd = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[default_params_file, {'use_sim_time': use_sim_time}])

    start_nav2_waypoint_follower_cmd = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[default_params_file, {'use_sim_time': use_sim_time}])

    start_nav2_velocity_smoother_cmd = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[default_params_file, {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel_in', 'cmd_vel_nav'),
                    ('cmd_vel_out', 'cmd_vel')])

    # Start AMCL
    start_amcl_cmd = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[default_params_file, {'use_sim_time': use_sim_time}])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_lifecycle_mgr_cmd)
    ld.add_action(declare_map_subscribe_transient_local_cmd)

    # Add the nodes to the launch description
    ld.add_action(start_lifecycle_manager_cmd)
    ld.add_action(start_amcl_cmd)
    ld.add_action(start_nav2_controller_cmd)
    ld.add_action(start_nav2_planner_cmd)
    ld.add_action(start_nav2_recoveries_cmd)
    ld.add_action(start_nav2_bt_navigator_cmd)
    ld.add_action(start_nav2_waypoint_follower_cmd)
    ld.add_action(start_nav2_velocity_smoother_cmd)

    return ld

