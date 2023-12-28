// Copyright 2021 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef READ_WRITE_NODE_HPP_
#define READ_WRITE_NODE_HPP_

#include <cstdio>
#include <memory>
#include <string>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_msgs/msg/bool.hpp"

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"


class MarkerArmControlNode : public rclcpp::Node
{
public:

  MarkerArmControlNode();
  virtual ~MarkerArmControlNode();

private:
  uint8_t dxl_error_;
  bool enable_torques_[2];
  int dxl_comm_result_;
  int period_tf2_ms_;
  std::string dynamixel_port_;

  std::string joint_err_states_topic_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_err_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr deactivate_sub_;

  dynamixel::PortHandler * portHandler_;
  dynamixel::PacketHandler * packetHandler_;
  rclcpp::TimerBase::SharedPtr timer_;
  double max_control_gain_;
  double min_control_gain_;
  double yaw_control_coeff_;
  double pitch_init_ang_;
  double control_coeff_[2];
  double max_ang_err_[2];
  int pitch_limits_[2];
  bool init_curr_ang_;
  int curr_ang_pulses_[2];
  int prev_ang_pulses_[2];

  uint8_t joint_ids_[2];
  double joint_curr_angs_[2];
  double joint_cmd_angs_[2];
  bool halt_motor_;
  int no_marker_cnt_;
  bool exp_deactivation_;
  bool deactivate_;

  void controlAng(const int &goal_position, const int &joint_id);
  void jointSetAngs();
  void setupDynamixel();
  void jointPubTimerCallback();
  void deactivateCallback(const std_msgs::msg::Bool::SharedPtr _msg);
  void jointErrCallback(const sensor_msgs::msg::JointState::SharedPtr _msg);
  int radToPulse(const double &inp_ang);
  double pulseToRad(const int &inp_pulse);
};

#endif  // READ_WRITE_NODE_HPP_

