from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    exp_logger_path = get_package_share_path('exp_logger')


    params_path = os.path.join(
        exp_logger_path,
        'params',
        'exp_logger_params.yaml'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='mov_rviz2',
        output='screen',
    )

    exp_logger_node = Node(
        package='exp_logger',
        executable='exp_logger',
        name='exp_logger',
        output='screen',
        parameters=[params_path]
    )


    return LaunchDescription([
        rviz_node,
        exp_logger_node
    ])
