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
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    omo_bringup_pkg_dir = os.path.join(get_package_share_directory('omo_r1mini_bringup'), 'launch')

    omo_nav2_pkg_dir = os.path.join(get_package_share_directory('omo_r1mini_navigation2'), 'launch')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true')

    omo_bringup = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([omo_bringup_pkg_dir, '/omo_r1mini_t265_bringup.launch.py']),
      launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    omo_nav2 = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([omo_nav2_pkg_dir, '/omo_navi.launch.py']),
      launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    rviz_nav2 = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([omo_nav2_pkg_dir, '/navigation2_rviz.launch.py']),
      launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time)
    ld.add_action(omo_bringup)
    ld.add_action(omo_nav2)

    # ld.add_action(rviz_nav2)


    return ld
