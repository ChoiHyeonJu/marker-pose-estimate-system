#ifndef READ_WRITE_NODE_HPP_
#define READ_WRITE_NODE_HPP_

#include <cstdio>
#include <memory>
#include <string>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"


#include "tf2/exceptions.h"
#include "tf2/utils.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/msg/transform.h"
#include "sensor_msgs/msg/joint_state.hpp"

#include "nav_msgs/msg/odometry.hpp"

#include "opencv2/core.hpp"
#include "opencv2/video.hpp"

class MarkerArmPerceptionNode : public rclcpp::Node
{
public:
  MarkerArmPerceptionNode();
  virtual ~MarkerArmPerceptionNode();

private:
  int period_detect_ms_;
  std::string body_base_frame_;
  std::string opp_base_frame_;
  std::string fix_base_frame_;
  std::string cam_frame_;
  std::string link0_axis_frame_;
  std::string vis_opp_marker_frame_;
  std::string opp_marker_frame_;
  std::string vis_ego_marker_frame_;
  std::string ego_marker_frame_;
  std::string vis_opp_base_frame_;
  std::string yaw_frame_;
  std::string pitch_frame_;
  std::string marker_pose_topic_;
  std::string opp_odom_frame_;
  double kf_vel_damp_;
  double combine_m2m_alpha_;
  double update_m2m_alpha_;
  // geometry_msgs::msg::PoseStamped mes_marker_pose_;
  // geometry_msgs::msg::PoseStamped marker_pose_;
  // geometry_msgs::msg::PoseStamped arm2marker_pose_;

  geometry_msgs::msg::TransformStamped arm2marker_tf_;
  geometry_msgs::msg::TransformStamped marker2marker_tf_;
  geometry_msgs::msg::TransformStamped mes_marker2marker_tf_;
  geometry_msgs::msg::TransformStamped est_marker2marker_tf_;
  geometry_msgs::msg::TransformStamped mes_cam2marker_tf_;
  geometry_msgs::msg::TransformStamped est_cam2marker_tf_;
  geometry_msgs::msg::TransformStamped FB2OD_tf_;


  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  double joint_err_angs_[2];
  bool is_fix_robot_;
  bool marker_detected_;
  bool filter_initialized_;
  bool pub_odom_tf_;
  bool surface_mode_;
  bool FB2RB_tf_init_set_;
  double FB2RB_thres_ang_;
  double FB2OD_set_err_thres_dist_;
  int FB2RB_counter_;
  int fail_combine_;

  sensor_msgs::msg::JointState joint_err_state_;


  cv::KalmanFilter kf_arm2marker_;
  cv::Mat_<float> m_arm2marker_;
  cv::Mat_<float> s_arm2marker_;
  cv::Mat_<float> trans_;
  cv::Mat_<float> trans_damp_;

  cv::KalmanFilter kf_odom_;
  cv::Mat_<float> m_odom_;
  cv::Mat_<float> s_odom_;

  nav_msgs::msg::Odometry marker_odom_;
  nav_msgs::msg::Odometry gt_marker_odom_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr marker_odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr gt_marker_odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_err_pub_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr marker_pose_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  void transAverage(
      const double& alpha,
      geometry_msgs::msg::TransformStamped& transform1,
      geometry_msgs::msg::TransformStamped& transform2,
      geometry_msgs::msg::TransformStamped& avgTransform);
  void slerpAverage(
      const double& alpha,
      geometry_msgs::msg::TransformStamped& transform1,
      geometry_msgs::msg::TransformStamped& transform2,
      geometry_msgs::msg::TransformStamped& avgTransform);
  void markerPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr _msg);
  double getJointErr(const std::string ref_frame, const std::string curr_frame,
      const geometry_msgs::msg::TransformStamped curr_marker_tf);
  void markerTf2TimerCallback();
  void initKalmanFilter();
};

#endif  // READ_WRITE_NODE_HPP_
