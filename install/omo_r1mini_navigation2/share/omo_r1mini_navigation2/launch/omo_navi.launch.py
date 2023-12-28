import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    # Get the launch directory
    omo_nav2_pkg_dir = get_package_share_directory('omo_r1mini_navigation2')

    omo_bringup_pkg_dir = get_package_share_directory('omo_r1mini_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')

    lifecycle_nodes = ['controller_server',
                       'planner_server',
                       'bt_navigator']

    param_substitutions = {
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'default_bt_xml_filename': default_bt_xml_filename}

    configured_params = RewrittenYaml(
            source_file=params_file,
            root_key='',
            param_rewrites=param_substitutions,
            convert_types=True)

    # Declare the launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_autostart = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(omo_nav2_pkg_dir, 'param', 'my_omo_nav2_params.yaml'),
        # default_value=os.path.join(omo_nav2_pkg_dir, 'param', 'omo_r1mini.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_bt_xml_cmd = DeclareLaunchArgument(
        'default_bt_xml_filename',
        default_value=os.path.join(omo_nav2_pkg_dir, 'bt',
            'pure_bt.xml'),
        description='Full path to the behavior tree xml file to use')

    map_odom_static_tf_publisher_cmd = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen')

    # start_odom_base_link_static_transform_publisher_cmd = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
    #     output='screen')

    controller_cmd = Node(
            package='nav2_controller',
            executable='controller_server',
            # prefix=['xterm -e gdb -ex run --args'],
            output='screen',
            parameters=[configured_params])

    planner_cmd = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[configured_params])

    bt_navigator_cmd = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[configured_params])

    lifecycle_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': lifecycle_nodes}])


    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_params_file_cmd)

    ld.add_action(declare_autostart)
    ld.add_action(declare_bt_xml_cmd)
    ld.add_action(controller_cmd)
    ld.add_action(planner_cmd)

    ld.add_action(bt_navigator_cmd)
    ld.add_action(lifecycle_cmd)

    ld.add_action(map_odom_static_tf_publisher_cmd)
    # ld.add_action(start_odom_base_link_static_transform_publisher_cmd)

    return ld
