"""
This node locates Aruco AR markers in images and publishes their ids and poses.

Subscriptions:
   /camera/image_raw (sensor_msgs.msg.Image)
   /camera/camera_info (sensor_msgs.msg.CameraInfo)
   /camera/camera_info (sensor_msgs.msg.CameraInfo)

Published Topics:
    /aruco_poses (geometry_msgs.msg.PoseArray)
       Pose of all detected markers (suitable for rviz visualization)

    /aruco_markers (ros2_aruco_interfaces.msg.ArucoMarkers)
       Provides an array of all poses along with the corresponding
       marker ids.

Parameters:
    marker_size - size of the markers in meters (default .0625)
    aruco_dictionary_id - dictionary that was used to generate markers
                          (default DICT_5X5_250)
    image_topic - image topic to subscribe to (default /camera/image_raw)
    camera_info_topic - camera info topic to subscribe to
                         (default /camera/camera_info)

Author: Nathan Sprague
Version: 10/26/2020

"""

import rclpy
import rclpy.node
from rclpy.qos import qos_profile_sensor_data
import cv2
from cv_bridge import CvBridge
import numpy as np

from ros2_aruco import transformations

from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool


class ArucoNode(rclpy.node.Node):

    def __init__(self):
        super().__init__('aruco_node')

        # Declare and read parameters
        self.declare_parameter("marker_size", .0625)
        self.declare_parameter("aruco_dictionary_id", "DICT_5X5_250")
        self.declare_parameter("target_marker_id", 50)
        self.declare_parameter("min_dist", 0.15)
        self.declare_parameter("max_dist", 4.0)
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("camera_frame", None)
        self.declare_parameter("img_show", False)

        self.marker_size = self.get_parameter("marker_size").get_parameter_value().double_value
        self.min_dist = self.get_parameter("min_dist").get_parameter_value().double_value
        self.max_dist = self.get_parameter("max_dist").get_parameter_value().double_value
        dictionary_id_name = self.get_parameter(
            "aruco_dictionary_id").get_parameter_value().string_value
        self.target_marker_id = self.get_parameter("target_marker_id").get_parameter_value().integer_value
        image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        info_topic = self.get_parameter("camera_info_topic").get_parameter_value().string_value
        self.camera_frame = self.get_parameter("camera_frame").get_parameter_value().string_value
        self.img_show = self.get_parameter("img_show").get_parameter_value().bool_value
        # Make sure we have a valid dictionary id:
        try:
            dictionary_id = cv2.aruco.__getattribute__(dictionary_id_name)
            if type(dictionary_id) != type(cv2.aruco.DICT_5X5_100):
                raise AttributeError
        except AttributeError:
            self.get_logger().error("bad aruco_dictionary_id: {}".format(dictionary_id_name))
            options = "\n".join([s for s in dir(cv2.aruco) if s.startswith("DICT")])
            self.get_logger().error("valid options: {}".format(options))

        # Set up subscriptions
        self.info_sub = self.create_subscription(CameraInfo,
                                                 info_topic,
                                                 self.info_callback,
                                                 qos_profile_sensor_data)

        self.create_subscription(Image, image_topic,
                                 self.image_callback, qos_profile_sensor_data)

        # Set up publishers
        self.poses_pub = self.create_publisher(PoseStamped, '~/marker_pose', 10)

        self.detect_pub = self.create_publisher(Bool, '~/marker_detect', 10)
        self.detect_msg = Bool()
        self.detect_msg.data = False

        # Set up fields for camera parameters
        self.info_msg = None
        self.intrinsic_mat = None
        self.distortion = None

        self.aruco_dictionary = cv2.aruco.Dictionary_get(dictionary_id)
        self.aruco_parameters = cv2.aruco.DetectorParameters_create()
        self.bridge = CvBridge()
        self.marker_pose = PoseStamped()



    def info_callback(self, info_msg):
        self.info_msg = info_msg
        self.intrinsic_mat = np.reshape(np.array(self.info_msg.k), (3, 3))
        self.distortion = np.array(self.info_msg.d)
        self.get_logger().warn("Success receiving camera info! Now destroy subscription")
        # Assume that camera parameters will remain the same...
        self.destroy_subscription(self.info_sub)

    def image_callback(self, img_msg):

        if self.info_msg is None:
            self.get_logger().warn("No camera info has been received!")
            return

        cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='mono8')
        # cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')

        corners, marker_ids, rejected = cv2.aruco.detectMarkers(cv_image,
                                                                self.aruco_dictionary,
                                                                parameters=self.aruco_parameters)
        
        self.detect_msg.data = False
        if marker_ids is not None:
            # self.get_logger().warn("marker_ids {}".format(marker_ids))
            if len(marker_ids) == 1 and marker_ids[0] == self.target_marker_id:
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners,
                                                                    self.marker_size, self.intrinsic_mat,
                                                                    self.distortion)
                self.marker_pose.pose.position.x = tvecs[0][0][0]
                self.marker_pose.pose.position.y = tvecs[0][0][1]
                self.marker_pose.pose.position.z = tvecs[0][0][2]
                # self.get_logger().warn("dist {}".format(pose.pose.position.z))
                if self.marker_pose.pose.position.z > self.min_dist and self.marker_pose.pose.position.z < self.max_dist:
                    rot_matrix = np.eye(4)
                    rot_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[0][0]))[0]
                    quat = transformations.quaternion_from_matrix(rot_matrix)

                    self.marker_pose.pose.orientation.x = quat[0]
                    self.marker_pose.pose.orientation.y = quat[1]
                    self.marker_pose.pose.orientation.z = quat[2]
                    self.marker_pose.pose.orientation.w = quat[3]
                    self.poses_pub.publish(self.marker_pose)

                    self.detect_msg.data = True
                    if self.img_show:
                        self.add_axis(cv_image, corners[0], rvecs, tvecs)

        self.detect_pub.publish(self.detect_msg)

        # This makes 10 frames slow! Just use it for debugging
        if self.img_show:
            cv2.imshow('Image', cv_image)
            cv2.waitKey(1)




    def add_axis(self, cv_image, corner, rvec, tvec):
        x1 = (int(corner[0][0][0]),int(corner[0][0][1]))
        x2 = (int(corner[0][1][0]),int(corner[0][1][1]))
        x3 = (int(corner[0][2][0]),int(corner[0][2][1]))
        x4 = (int(corner[0][3][0]),int(corner[0][3][1]))

        cv2.line(cv_image, x1, x2, (255,0,0), 5)
        cv2.line(cv_image, x2, x3, (255,0,0), 5)
        cv2.line(cv_image, x3, x4, (255,0,0), 5)
        cv2.line(cv_image, x4, x1, (255,0,0), 5)

        cv_image = cv2.drawFrameAxes(cv_image, self.intrinsic_mat, self.distortion, rvec, tvec, 0.1, 5)


def main():
    rclpy.init()
    node = ArucoNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
