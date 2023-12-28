#!/usr/bin/env python3

# Author: Bishop Pearson

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node

def generate_launch_description():
  omo_r1mini_mcu_parameter = LaunchConfiguration(
    'omo_r1mini_mcu_parameter',
    default=os.path.join(
      get_package_share_directory('omo_r1mini_bringup'),
      'param/omo_r1mini_mcu.yaml'
    )
  )

  omo_r1mini_lidar_parameter = LaunchConfiguration(
    'omo_r1mini_lidar_parameter',
    default=os.path.join(
      get_package_share_directory('omo_r1mini_bringup'),
      'param/omo_r1mini_G6_lidar.yaml'
    )
  )

  use_sim_time = LaunchConfiguration('use_sim_time', default='false')

  omo_r1mini_description_dir = LaunchConfiguration(
    'omo_r1mini_description_dir',
    default=os.path.join(
      get_package_share_directory('omo_r1mini_description'),
      'launch'
    )
  )

  realsense_t265_dir = LaunchConfiguration(
    'realsense_t265_dir',
    default=os.path.join(
      get_package_share_directory('realsense2_camera'),
      'launch'
    )
  )

  robot_localization_file_path = os.path.join(get_package_share_directory('omo_r1mini_bringup'), 'param/omo_ekf.yaml')

  return LaunchDescription([
    DeclareLaunchArgument(
      'omo_r1mini_mcu_parameter',
      default_value=omo_r1mini_mcu_parameter
    ),

    DeclareLaunchArgument(
      'omo_r1mini_lidar_parameter',
      default_value=omo_r1mini_lidar_parameter
    ),

    IncludeLaunchDescription(
      PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/omo_r1mini_mcu.launch.py']),
      launch_arguments={'omo_r1mini_mcu_parameter': omo_r1mini_mcu_parameter}.items()
    ),

    IncludeLaunchDescription(
      PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/omo_r1mini_lidar.launch.py']),
      launch_arguments={'omo_r1mini_lidar_parameter': omo_r1mini_lidar_parameter}.items()
    ),

    IncludeLaunchDescription(
      PythonLaunchDescriptionSource([realsense_t265_dir, '/rs_t265_launch.py']),
      launch_arguments={'use_sim_time': use_sim_time}.items(),
    ),

    IncludeLaunchDescription(
      PythonLaunchDescriptionSource([omo_r1mini_description_dir, '/omo_r1mini_t265_state_publisher.launch.py']),
      launch_arguments={'use_sim_time': use_sim_time}.items(),
    ),

    Node(
      package='tf2_ros',
      executable='static_transform_publisher',
      arguments=['0.05', '0', '0.11', '0', '0', '0', 'odom', 'camera_odom_frame'],
      output='screen'
    ),

    # Node(
    #   package='robot_localization',
    #   executable='ekf_node',
    #   name='ekf_filter_node',
    #   # remappings=[
    #   #     ('/odometry/filtered', '/odom')],
    #   output='screen',
    #   parameters=[robot_localization_file_path,
    #               {'use_sim_time': use_sim_time}])

    # Node(
    #   package='tf2_ros',
    #   executable='static_transform_publisher',
    #   arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
    #   output='screen'
    # )
  ])
