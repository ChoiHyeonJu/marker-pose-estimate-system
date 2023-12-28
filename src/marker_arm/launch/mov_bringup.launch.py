from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os
import xacro

def generate_launch_description():
    marker_arm_path = get_package_share_path('marker_arm')

    robot_file = "marker_mov.urdf.xacro"
    urdf_file = os.path.join(marker_arm_path, "urdf", robot_file)
    robot_urdf =  xacro.process_file(urdf_file)
    robot_description = {'robot_description': robot_urdf.toxml()}

    model_arg = DeclareLaunchArgument(name='model', default_value=str(urdf_file),
                                      description='Absolute path to robot urdf file')


    camera_params_path = os.path.join(
        marker_arm_path,
        'params',
        'mov_cam_params.yaml'
    )

    # arducam_params_path = os.path.join(
    #     marker_arm_path,
    #     'params',
    #     'arducam_params.yaml'
    # )

    aruco_params_path = os.path.join(
        marker_arm_path,
        'params',
        'mov_aruco_params.yaml'
    )

    perception_params_path = os.path.join(
        marker_arm_path,
        'params',
        'mov_perception_params.yaml'
    )

    control_params_path = os.path.join(
        marker_arm_path,
        'params',
        'mov_control_params.yaml'
    )

    # arducam_node = Node(
    #     package='arducam_node',
    #     executable='arducam_node',
    #     name='mov_cam_node',
    #     output='screen',
    #     parameters=[arducam_params_path],
    # )

    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='mov_cam_node',
        output='screen',
        parameters=[camera_params_path],
        remappings=[
            ('/camera_info', '/mov_cam_node/camera_info'),
            ('/image_raw', '/mov_cam_node/image_raw'),
            ('/image_raw/compressed', '/mov_cam_node/image_raw/compressed'),
            ('/image_raw/compressedDepth', '/mov_cam_node/image_raw/compressedDepth'),
            ('/image_raw/theora', '/mov_cam_node/image_raw/theora')
        ]
    )

    aruco_node = Node(
        package='ros2_aruco',
        executable='aruco_node',
        name='mov_aruco_node',
        output='screen',
        parameters=[aruco_params_path],
    )

    marker_arm_perception_node = Node(
        package='marker_arm',
        executable='marker_arm_perception_node',
        name='mov_perception_node',
        output='screen',
        parameters=[perception_params_path]
    )

    marker_arm_control_node = Node(
        package='marker_arm',
        executable='marker_arm_control_node',
        name='mov_control_node',
        output='screen',
        parameters=[control_params_path]
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='mov_state_publisher',
        output='screen',
        parameters=[robot_description],
        namespace='/mov_state_publisher',
        remappings=[
            ('/mov_state_publisher/joint_states', '/mov_control_node/joint_states')
        ]
    )


    # static_tf_t265_node = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     arguments=['-0.06', '0', '0', '0', '0', '0', 'camera_pose_frame', 'marker_mov/arm_base'],
    #     output='screen'
    # )
    static_tf_t265_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['-0.06', '0', '-0.055', '0', '0', '0', 'camera_pose_frame', 'marker_mov/arm_base'],
        output='screen'
    )

    static_tf_connect_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0.145', '0', '0', '0', 'base_link', 'marker_mov/arm_base'],
        output='screen'
    )

    static_tmp_tf_connect_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0.2', '1.57', '0', '1.57', 'fix_base', 'marker_frame'],
        output='screen'
    )

    realsense2_camera_dir = LaunchConfiguration(
        'realsense2_camera_dir',
        default=os.path.join(
            get_package_share_directory('realsense2_camera'),
            'launch'
        )
    )

    omo_r1mini_bringup_dir = LaunchConfiguration(
        'omo_r1mini_bringup_dir',
        default=os.path.join(
            get_package_share_directory('omo_r1mini_bringup'),
            'launch'
        )
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='mov_rviz2',
        output='screen',
    )

    return LaunchDescription([
        usb_cam_node,
        aruco_node,
        model_arg,
        robot_state_publisher_node,
        marker_arm_perception_node,
        marker_arm_control_node,
        static_tf_t265_node,
        #rviz_node,

        # T265 stuff
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([realsense2_camera_dir, '/rs_launch.py']),
            launch_arguments={'device_type': 't265'}.items()
        )

        # omo robot stuff
        # static_tf_connect_node,
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([omo_r1mini_bringup_dir, '/omo_base.launch.py']),
        # )
    ])
