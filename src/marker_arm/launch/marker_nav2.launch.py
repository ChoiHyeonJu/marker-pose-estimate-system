import os
from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    marker_arm_path = get_package_share_path('marker_arm')

    launch_path = os.path.join(marker_arm_path, 'launch')

    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')

    # Launch configuration variables specific to simulation
    lifecycle_nodes = ['controller_server',
                       'planner_server',
                       'bt_navigator']

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true')

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
    declare_autostart = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(marker_arm_path, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_bt_xml_cmd = DeclareLaunchArgument(
        'default_bt_xml_filename',
        default_value=os.path.join(marker_arm_path, 'bt', 'pure_bt.xml'),
        description='Full path to the behavior tree xml file to use')

    rviz_config_file = LaunchConfiguration('rviz_config_file')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(marker_arm_path, 'rviz', 'nav2_default_view.rviz'),
        description='Full path to the RVIZ config file to use')

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

    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen')

    return LaunchDescription([
        declare_use_sim_time,
        declare_params_file_cmd,
        declare_autostart,
        declare_bt_xml_cmd,
        declare_rviz_config_file_cmd,
        controller_cmd,
        planner_cmd,
        bt_navigator_cmd,
        lifecycle_cmd,
        # rviz_cmd
    ])
