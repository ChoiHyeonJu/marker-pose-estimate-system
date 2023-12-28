#include <cstdio>
#include <memory>
#include <string>
#include <cmath>
#include <algorithm>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "marker_arm_control_node.hpp"

#define DEBUG_PRINT 0
// #define DEBUG_PRINT 1

// Control table address for X series (except XL-320)
#define ADDR_OPERATING_MODE 11
#define ADDR_HOMING_OFFSET 20
#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_POSITION 116
#define ADDR_PRESENT_POSITION 132

#define ADDR_PROFILE_VELOCITY 112
#define ADDR_MAX_POSITION_LIMIT 48
#define ADDR_MIN_POSITION_LIMIT 52

#define JOINT_1_ID 1
#define JOINT_2_ID 2

#define OP_POS_CONTROL_MODE 3
#define OP_EXT_POS_CONTROL_MODE 4

// Protocol version
#define PROTOCOL_VERSION 2.0  // Default Protocol version of DYNAMIXEL X series.

// Default setting
#define BAUDRATE 57600  // Default Baudrate of DYNAMIXEL X series
#define RAD_TO_PULSE 651.918  // 4096 (pulse per rot) / 2pi (rad)
#define PULSE_TO_RAD 0.00153  // 2pi (rad) / 4096 (pulse per rot)
#define MIN_OVER_RAD 9.5496// 60 sec / 2pi
#define VELOCITY_REVO_RESOLUTION 0.229 // 0.229 (rev/min) (given from https://emanual.robotis.com/docs/en/dxl/x/xw430-t333/ )
#define MAX_ROT_MIN 31 // (rev/min) (given from https://emanual.robotis.com/docs/en/dxl/x/xw430-t333/ )


template <typename T>
const T& clamp(const T& value, const T& minValue, const T& maxValue) {
    return (value < minValue) ? minValue : ((maxValue < value) ? maxValue : value);
}


using std::placeholders::_1;
using namespace std::chrono_literals;

std::string node_name = "marker_arm_control_node";

MarkerArmControlNode::MarkerArmControlNode(): Node(node_name)
{
  double max_up_pitch_ang, min_down_pitch_ang;
  this->declare_parameter("exp_deactivation", rclcpp::ParameterValue(false));
  this->declare_parameter("period_tf2_ms", rclcpp::ParameterValue(10)); 
  this->declare_parameter("yaw_enable_torque", rclcpp::ParameterValue(false)); // 몸통 쪽 조인트 off
  this->declare_parameter("pitch_enable_torque", rclcpp::ParameterValue(false)); // 카메라 쪽 조인트 off
  this->declare_parameter("pitch_init_ang", rclcpp::ParameterValue(0.0));  // 카메라 쪽 조인트 초기 각도 0도
  this->declare_parameter("dynamixel_port", rclcpp::ParameterValue("/dev/ttyUSB0")); // 다이나믹셀이 연결된 포트
  this->declare_parameter("min_control_gain", rclcpp::ParameterValue(0.1)); // 조인트들의 최소 게인 값
  this->declare_parameter("max_control_gain", rclcpp::ParameterValue(0.5)); // 조인트들의 최대 게인값
  this->declare_parameter("yaw_max_ang_err", rclcpp::ParameterValue(0.52)); // 30 deg // 몸통 쪽 조인트 최대 각도
  this->declare_parameter("pitch_max_ang_err", rclcpp::ParameterValue(0.39)); // 30 deg * 960(H) / 1280(W) // 카메라 쪽 조인트 최대 각도
  this->declare_parameter("max_up_pitch_ang", rclcpp::ParameterValue(1.4)); 
  this->declare_parameter("min_down_pitch_ang", rclcpp::ParameterValue(-0.35));
  this->declare_parameter("joint_err_states_topic", rclcpp::ParameterValue("joint_err_states")); // 실제 움직일 각도

  this->get_parameter("exp_deactivation", exp_deactivation_);
  this->get_parameter("period_tf2_ms", period_tf2_ms_);
  this->get_parameter("yaw_enable_torque", enable_torques_[0]);
  this->get_parameter("pitch_enable_torque", enable_torques_[1]);
  this->get_parameter("pitch_init_ang", pitch_init_ang_);
  this->get_parameter("dynamixel_port", dynamixel_port_);
  this->get_parameter("min_control_gain", min_control_gain_);
  this->get_parameter("max_control_gain", max_control_gain_);
  this->get_parameter("yaw_max_ang_err", max_ang_err_[0]);
  this->get_parameter("pitch_max_ang_err", max_ang_err_[1]);
  this->get_parameter("max_up_pitch_ang", max_up_pitch_ang);
  this->get_parameter("min_down_pitch_ang", min_down_pitch_ang);
  this->get_parameter("joint_err_states_topic", joint_err_states_topic_);

  deactivate_ = false;
  pitch_init_ang_ += M_PI_2; //90도
  min_down_pitch_ang += M_PI_2;//90도
  max_up_pitch_ang += M_PI_2;//90도
  pitch_limits_[0] = radToPulse(min_down_pitch_ang); //각도 90를 다이나믹셀이 움직일 수 있는 펄스로 바꾸는거임 다이나믹셀은 0-4096임
  pitch_limits_[1] = radToPulse(max_up_pitch_ang);


  control_coeff_[0] = (max_control_gain_ - min_control_gain_) / pow(max_ang_err_[0], 2);
  control_coeff_[1] = (max_control_gain_ - min_control_gain_) / pow(max_ang_err_[1], 2);

  RCLCPP_WARN(this->get_logger(), "Enable torque (yaw:%d, pitch:%d)", enable_torques_[0], enable_torques_[1]);

  no_marker_cnt_ = 0;

  dxl_error_ = 0;
  dxl_comm_result_ = COMM_TX_FAIL;

  const char *c_device_name = dynamixel_port_.c_str();
  portHandler_ = dynamixel::PortHandler::getPortHandler(c_device_name);
  packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  dxl_comm_result_ = portHandler_->openPort();
  if (dxl_comm_result_ == false) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open the port!");
    return;
  } else {
    RCLCPP_INFO(this->get_logger(), "Succeeded to open the port. %s", dynamixel_port_.c_str());
  }

  // Set the baudrate of the serial port (use DYNAMIXEL Baudrate)
  dxl_comm_result_ = portHandler_->setBaudRate(BAUDRATE);
  if (dxl_comm_result_ == false) {
    RCLCPP_ERROR(this->get_logger(), "Failed to set the baudrate %d", BAUDRATE);
    return;
  } else {
    RCLCPP_INFO(this->get_logger(), "Succeeded to set the baudrate %d", BAUDRATE);
  }

  joint_ids_[0] = JOINT_1_ID; // 몸통 쪽 조인트
  joint_ids_[1] = JOINT_2_ID; // 카메라 쪽 조인트
  joint_cmd_angs_[0] = 0;
  joint_cmd_angs_[1] = pitch_init_ang_; // 카메라 쪽 조인트는 90도가 초기 각도
  joint_curr_angs_[0] = 0;
  joint_curr_angs_[1] = 0;

  setupDynamixel();

  init_curr_ang_ = true;
  jointSetAngs();


  RCLCPP_INFO(this->get_logger(), "Run Dynamicxel Pos Control node");

  int8_t qos_depth = 10;
  const auto QOS_RKL10V =
    rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

  joint_state_pub_ =
    this->create_publisher<sensor_msgs::msg::JointState>("~/joint_states", 10);

  joint_err_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    joint_err_states_topic_,
    QOS_RKL10V,
    std::bind(&MarkerArmControlNode::jointErrCallback, this, _1));

  deactivate_sub_ = this->create_subscription<std_msgs::msg::Bool>( 
    "/deactivate",
    QOS_RKL10V,
    std::bind(&MarkerArmControlNode::deactivateCallback, this, _1));

  timer_ = this->create_wall_timer(std::chrono::milliseconds(period_tf2_ms_),
    std::bind(&MarkerArmControlNode::jointPubTimerCallback, this));

}


void MarkerArmControlNode::jointPubTimerCallback() // 조인트 실제로 움직이고 이 값을 joint state pub에 pub해서 rviz에서도 움직이게 함
{
  for (int i = 0; i < 2; i++) {
    prev_ang_pulses_[i] = curr_ang_pulses_[i];
    dxl_comm_result_ = packetHandler_->read4ByteTxRx(
      portHandler_,
      joint_ids_[i],
      ADDR_PRESENT_POSITION,
      reinterpret_cast<uint32_t *>(&curr_ang_pulses_[i]),
      &dxl_error_
    );
    if (dxl_comm_result_ == COMM_SUCCESS) {
      joint_curr_angs_[i] = pulseToRad(curr_ang_pulses_[i]);
      joint_curr_angs_[i] -= M_PI_2;
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to receive angle. err=%d", dxl_comm_result_);
    }
  }

  // RCLCPP_INFO(this->get_logger(), "ID(%d) rad: %.2f(%d) / ID(%d) rad: %.2f(%d)",
  //                 joint_ids_[0], joint_curr_angs_[0], curr_ang_pulses[0],
  //                 joint_ids_[1], joint_curr_angs_[1], curr_ang_pulses[1]);
  auto joint_state = sensor_msgs::msg::JointState();
  joint_state.header.stamp = this->now();
  joint_state.name = {"joint1", "joint2"};  // Example joint names
  joint_state.position = {joint_curr_angs_[0], joint_curr_angs_[1]};  // Example joint positions
  joint_state_pub_->publish(joint_state);
}

void MarkerArmControlNode::jointErrCallback(const sensor_msgs::msg::JointState::SharedPtr _msg) // 만약 조인트 각도가 둘 다 0 0 이면 초기 위치를 찾도록 하는 함수
{
  if (deactivate_) {
    joint_cmd_angs_[0] = 0;
    joint_cmd_angs_[1] = 0;
  } else {
    init_curr_ang_ = false;
    double ang_errs[2] = {_msg->position[0], _msg->position[1]};
    // RCLCPP_WARN(this->get_logger(), "YYaw (%f), PPitch (%f)", ang_errs[0], ang_errs[1]);
    for (int i = 0; i < 2; i++) {
      ang_errs[i] = clamp(ang_errs[i], -max_ang_err_[i], max_ang_err_[i]);
      double control_gain = control_coeff_[i] * std::pow(ang_errs[i], 2) + min_control_gain_;
      // RCLCPP_WARN(this->get_logger(), "[%d] coeff (%f), gain (%f)", i, control_coeff_[i], control_gain);

      joint_cmd_angs_[i] = control_gain * ang_errs[i];
      if (fabs(joint_cmd_angs_[i]) < 0.01) {
        joint_cmd_angs_[i] = 0;
      }
    }
    // RCLCPP_WARN(this->get_logger(), "Err: YYaw (%f), PPitch (%f)", ang_errs[0], ang_errs[1]);
    // RCLCPP_WARN(this->get_logger(), "Cmd: YYaw (%f), PPitch (%f)", joint_cmd_angs_[0], joint_cmd_angs_[1]);
  }
  jointSetAngs();
}

void MarkerArmControlNode::deactivateCallback(const std_msgs::msg::Bool::SharedPtr _msg)// detect_sub이 false면 이 문구를 터미널에 출력하는 함수인듯
{
  if (exp_deactivation_ && !deactivate_ && _msg->data) {
    RCLCPP_WARN(this->get_logger(), "Received deactivation!!!");
    deactivate_ = true;
  }
}

void MarkerArmControlNode::jointSetAngs() // 라디안 각도를 펄스로 바꾸고 목표 펄스 위치를 계산
{
  int strt_id = 0;
  if (init_curr_ang_) {
    strt_id = 1;
  }
  for (int i = strt_id; i < 2; i++) {
    double joint_cmd_ang = joint_cmd_angs_[i];
    int rad2pulse = radToPulse(joint_cmd_ang);
    int goal_position = 0.5 * (curr_ang_pulses_[i] + prev_ang_pulses_[i]) + rad2pulse;
    if (joint_ids_[i] == JOINT_2_ID) {
      goal_position = clamp(goal_position, pitch_limits_[0], pitch_limits_[1]);
    }
    controlAng(goal_position, joint_ids_[i]);
    // dxl_comm_result_ = packetHandler_->write4ByteTxRx(
    //   portHandler_,
    //   (uint8_t) joint_ids_[i],
    //   ADDR_GOAL_POSITION,
    //   goal_position,
    //   &dxl_error_
    // );

    // if (dxl_comm_result_ != COMM_SUCCESS) {
    //   RCLCPP_ERROR(this->get_logger(), "%s", packetHandler_->getTxRxResult(dxl_comm_result_));
    // } else if (dxl_error_ != 0) {
    //   RCLCPP_ERROR(this->get_logger(), "[%d] %s", i, packetHandler_->getRxPacketError(dxl_error_));
    // } else {
    //   if (DEBUG_PRINT) {
    //     RCLCPP_INFO(this->get_logger(), "Set [%d] [Target ang: %.2f rad] (%d)", i, joint_cmd_ang, goal_position);
    //   }
    // }
  }
}

void MarkerArmControlNode::controlAng(const int &goal_position, const int &joint_id) { // 각도를 제어하기 위한 설정
  dxl_comm_result_ = packetHandler_->write4ByteTxRx(
    portHandler_,
    (uint8_t) joint_id,
    ADDR_GOAL_POSITION,
    goal_position,
    &dxl_error_
  );

  if (dxl_comm_result_ != COMM_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "%s", packetHandler_->getTxRxResult(dxl_comm_result_));
  } else if (dxl_error_ != 0) {
    RCLCPP_ERROR(this->get_logger(), "[%d] %s", joint_id, packetHandler_->getRxPacketError(dxl_error_));
  }
}


void MarkerArmControlNode::setupDynamixel() // 다이나믹셀 사용하기 위한 설정
{
  uint8_t op_modes[2] = {OP_EXT_POS_CONTROL_MODE, OP_POS_CONTROL_MODE};
  for (int i = 0; i < 2; i++) {
    dxl_comm_result_ = packetHandler_->write1ByteTxRx(
      portHandler_,
      (uint8_t) joint_ids_[i],
      ADDR_OPERATING_MODE,
      op_modes[i],
      &dxl_error_
    );

    if (dxl_comm_result_ != COMM_SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Failed to set Position Control Mode.");
    } else {
      RCLCPP_INFO(this->get_logger(), "Succeeded to set Position Control Mode.");
    }

    // This is for setting min ~ max range for link2. But didn't work for some reason.
    // if (joint_ids_[i] == JOINT_2_ID) {
    //   double ang_offset = 0.05;
    //   uint32_t max_ang_pulse = radToPulse(max_up_pitch_ang_ + ang_offset + M_PI_2);
    //   uint32_t min_ang_pulse = radToPulse(min_down_pitch_ang_ - ang_offset + M_PI_2);
    //   dxl_comm_result_ = packetHandler_->write4ByteTxRx(
    //     portHandler_,
    //     (uint8_t) joint_ids_[i],
    //     ADDR_MAX_POSITION_LIMIT,
    //     max_ang_pulse,
    //     &dxl_error_
    //   );
    //   if (dxl_comm_result_ != COMM_SUCCESS) {
    //     RCLCPP_ERROR(this->get_logger(), "Failed to set max_angle.");
    //   } else {
    //     RCLCPP_INFO(this->get_logger(), "Succeeded to set max_angle %.2f rad (%d).",
    //       max_up_pitch_ang_, max_ang_pulse);
    //   }

    //   dxl_comm_result_ = packetHandler_->write4ByteTxRx(
    //     portHandler_,
    //     (uint8_t) joint_ids_[i],
    //     ADDR_MIN_POSITION_LIMIT,
    //     min_ang_pulse,
    //     &dxl_error_
    //   );
    //   if (dxl_comm_result_ != COMM_SUCCESS) {
    //     RCLCPP_ERROR(this->get_logger(), "Failed to set min_angle.");
    //   } else {
    //     RCLCPP_INFO(this->get_logger(), "Succeeded to set min_angle %.2f rad (%d).",
    //       min_down_pitch_ang_, min_ang_pulse);
    //   }
    // }


    // Enable Torque of DYNAMIXEL
    dxl_comm_result_ = packetHandler_->write1ByteTxRx(
      portHandler_,
      (uint8_t) joint_ids_[i],
      ADDR_TORQUE_ENABLE,
      enable_torques_[i],
      &dxl_error_
    );

    if (dxl_comm_result_ != COMM_SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Failed to enable torque.");
    } else {
      RCLCPP_INFO(this->get_logger(), "Succeeded to enable torque.");
    }
  }
}



int MarkerArmControlNode::radToPulse(const double &target_ang)
{
  int goal_position = (int)(target_ang * RAD_TO_PULSE);
  return goal_position;
}

double MarkerArmControlNode::pulseToRad(const int &inp_pulse)
{
  double out_rad = inp_pulse * PULSE_TO_RAD;
  return out_rad;
}

MarkerArmControlNode::~MarkerArmControlNode()
{
  packetHandler_->write1ByteTxRx(
    portHandler_,
    (uint8_t) BROADCAST_ID,
    ADDR_TORQUE_ENABLE,
    0,
    &dxl_error_
  );
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto marker_arm_control_node = std::make_shared<MarkerArmControlNode>();
  rclcpp::spin(marker_arm_control_node);
  rclcpp::shutdown();

  return 0;
}

