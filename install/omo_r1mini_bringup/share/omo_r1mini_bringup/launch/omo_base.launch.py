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

  omo_urdf_file = os.path.join(
      get_package_share_directory('omo_r1mini_description'),
      'urdf', 'omo_simple.urdf')

  omo_urdf = LaunchConfiguration('omo_urdf', default=omo_urdf_file)

  omo_r1mini_description_dir = LaunchConfiguration(
    'omo_r1mini_description_dir',
    default=os.path.join(
      get_package_share_directory('omo_r1mini_description'),
      'launch'
    )
  )

  # realsense_t265_dir = LaunchConfiguration(
  #   'realsense_t265_dir',
  #   default=os.path.join(
  #     get_package_share_directory('realsense2_camera'),
  #     'launch'
  #   )
  # )

  joy_config_path = os.path.join(get_package_share_directory('omo_r1mini_bringup'), 'param/ps5_config.yaml')


  # static_tf_camera_odom_node = Node(
  #   package='tf2_ros',
  #   executable='static_transform_publisher',
  #   arguments=['0.05', '0', '0.11', '0', '0', '0', 'odom', 'camera_odom_frame'],
  #   output='screen'
  # )

  joy_node = Node(
    package='joy',
    executable='joy_node',
    name='joy_node',
    output='screen'
  )

  teleop_joy_twist_node = Node(
    package='teleop_twist_joy',
    executable='teleop_node',
    name='teleop_twist_joy_node',
    parameters=[joy_config_path],
    output='screen'
  )


  return LaunchDescription([
    DeclareLaunchArgument(
      'omo_r1mini_mcu_parameter',
      default_value=omo_r1mini_mcu_parameter
    ),

    DeclareLaunchArgument(
      'omo_urdf',
      default_value=omo_urdf),

    IncludeLaunchDescription(
      PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/omo_r1mini_mcu.launch.py']),
      launch_arguments={'omo_r1mini_mcu_parameter': omo_r1mini_mcu_parameter}.items()
    ),

    IncludeLaunchDescription(
      PythonLaunchDescriptionSource([omo_r1mini_description_dir, '/omo_state_publisher.launch.py']),
      launch_arguments={'omo_urdf': omo_urdf}.items()
    ),

    # Uncomment two lines below for joystick control
    # joy_node,
    # teleop_joy_twist_node

  ])
