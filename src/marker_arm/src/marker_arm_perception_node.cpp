#include <cstdio>
#include <memory>
#include <string>
#include <cmath>

#include "marker_arm_perception_node.hpp"


#define DEBUG_PRINT 0
// #define DEBUG_PRINT 1


using std::placeholders::_1;
using namespace std::chrono_literals;

std::string node_name = "marker_arm_perception_node";

MarkerArmPerceptionNode::MarkerArmPerceptionNode(): Node(node_name)
{
  this->declare_parameter("is_fix_robot", rclcpp::ParameterValue(false));
  this->declare_parameter("period_detect_ms", rclcpp::ParameterValue(30));
  this->declare_parameter("body_base_frame", rclcpp::ParameterValue("base_footprint"));
  this->declare_parameter("opp_base_frame", rclcpp::ParameterValue("fix_base"));
  this->declare_parameter("link0_axis_frame", rclcpp::ParameterValue("arm_base"));
  this->declare_parameter("yaw_frame", rclcpp::ParameterValue("link1"));
  this->declare_parameter("pitch_frame", rclcpp::ParameterValue("link2"));
  this->declare_parameter("cam_frame", rclcpp::ParameterValue("cam_frame"));
  this->declare_parameter("ego_marker_frame", rclcpp::ParameterValue("marker_mov/marker_frame"));
  this->declare_parameter("vis_ego_marker_frame", rclcpp::ParameterValue("vis_marker_frame"));
  this->declare_parameter("opp_marker_frame", rclcpp::ParameterValue("marker_frame"));
  this->declare_parameter("vis_opp_marker_frame", rclcpp::ParameterValue("vis_opp_marker_frame"));
  this->declare_parameter("vis_opp_base_frame", rclcpp::ParameterValue("vis_opp_base_frame"));
  this->declare_parameter("opp_odom_frame", rclcpp::ParameterValue("opp_odom_frame"));
  this->declare_parameter("kf_vel_damp", rclcpp::ParameterValue(0.99));
  this->declare_parameter("combine_m2m_alpha", rclcpp::ParameterValue(0.95));
  this->declare_parameter("update_m2m_alpha", rclcpp::ParameterValue(0.95));
  this->declare_parameter("marker_pose_topic", rclcpp::ParameterValue("marker_pose"));
  this->declare_parameter("pub_odom_tf", rclcpp::ParameterValue(false));
  this->declare_parameter("surface_mode", rclcpp::ParameterValue(true));
  this->declare_parameter("FB2OD_set_err_thres_dist", rclcpp::ParameterValue(0.015));



  this->get_parameter("is_fix_robot", is_fix_robot_);
  this->get_parameter("period_detect_ms", period_detect_ms_);
  this->get_parameter("body_base_frame", body_base_frame_);
  this->get_parameter("opp_base_frame", opp_base_frame_);
  this->get_parameter("link0_axis_frame", link0_axis_frame_);
  this->get_parameter("cam_frame", cam_frame_);
  this->get_parameter("yaw_frame", yaw_frame_);
  this->get_parameter("pitch_frame", pitch_frame_);
  this->get_parameter("vis_ego_marker_frame", vis_ego_marker_frame_);
  this->get_parameter("ego_marker_frame", ego_marker_frame_);
  this->get_parameter("opp_marker_frame", opp_marker_frame_);
  this->get_parameter("vis_opp_marker_frame", vis_opp_marker_frame_);
  this->get_parameter("vis_opp_base_frame", vis_opp_base_frame_);
  this->get_parameter("opp_odom_frame", opp_odom_frame_);
  this->get_parameter("kf_vel_damp", kf_vel_damp_);
  this->get_parameter("combine_m2m_alpha", combine_m2m_alpha_);
  this->get_parameter("update_m2m_alpha", update_m2m_alpha_);
  this->get_parameter("marker_pose_topic", marker_pose_topic_);
  this->get_parameter("pub_odom_tf", pub_odom_tf_);
  this->get_parameter("surface_mode", surface_mode_);
  this->get_parameter("FB2OD_set_err_thres_dist", FB2OD_set_err_thres_dist_);

  RCLCPP_INFO(this->get_logger(), "Run Marker Arm Perception node");
  FB2RB_counter_ = 0;
  fail_combine_ = 0;

  marker_detected_ = false;
  filter_initialized_ = false;
  FB2RB_tf_init_set_ = false;
  FB2RB_thres_ang_ = 0.026;

  int8_t qos_depth = 10;
  const auto QOS_RKL10V =
    rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

  joint_err_angs_[0] = 0;
  joint_err_angs_[1] = 0;

  mes_cam2marker_tf_.header.frame_id = cam_frame_.c_str();
  mes_cam2marker_tf_.child_frame_id = vis_opp_marker_frame_.c_str();
  est_cam2marker_tf_ = mes_cam2marker_tf_;

  marker2marker_tf_.header.frame_id = ego_marker_frame_.c_str();
  marker2marker_tf_.child_frame_id = vis_opp_marker_frame_.c_str();
  mes_marker2marker_tf_ = marker2marker_tf_;
  est_marker2marker_tf_ = marker2marker_tf_;

  FB2OD_tf_.header.frame_id = body_base_frame_.c_str();
  FB2OD_tf_.child_frame_id = opp_odom_frame_.c_str();


  initKalmanFilter();

  tf_buffer_ =
    std::make_unique<tf2_ros::Buffer>(this->get_clock());

  tf_listener_ =
    std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Initialize the transform broadcaster
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  marker_odom_.header.frame_id = body_base_frame_.c_str();
  marker_odom_.child_frame_id = opp_base_frame_.c_str();

  gt_marker_odom_.header.frame_id = body_base_frame_.c_str();
  gt_marker_odom_.child_frame_id = opp_base_frame_.c_str();



  marker_odom_pub_ =
    this->create_publisher<nav_msgs::msg::Odometry>("~/marker_odom", 10);

  gt_marker_odom_pub_ = //gt 는 ground trust인듯
    this->create_publisher<nav_msgs::msg::Odometry>("~/gt_marker_odom", 10);

  joint_err_pub_ =
    this->create_publisher<sensor_msgs::msg::JointState>("~/joint_err_states", 10);

  marker_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    marker_pose_topic_,
    QOS_RKL10V,
    std::bind(&MarkerArmPerceptionNode::markerPoseCallback, this, _1));

  timer_ = this->create_wall_timer(std::chrono::milliseconds(period_detect_ms_),
    std::bind(&MarkerArmPerceptionNode::markerTf2TimerCallback, this));

}

double MarkerArmPerceptionNode::getJointErr(const std::string ref_frame, const std::string curr_frame,
  const geometry_msgs::msg::TransformStamped curr_marker_tf)
{
  geometry_msgs::msg::TransformStamped ref2curr_tf = tf_buffer_->lookupTransform(
    ref_frame, curr_frame, tf2::TimePointZero);
  geometry_msgs::msg::TransformStamped ref_marker_tf;
  tf2::doTransform(curr_marker_tf, ref_marker_tf, ref2curr_tf);
  double ref_joint_err = std::atan2(
      ref_marker_tf.transform.translation.y, ref_marker_tf.transform.translation.x);
  return ref_joint_err;
}

void MarkerArmPerceptionNode::initKalmanFilter()
{
  int state_size = 6;     // 상태 벡터 크기
  int meas_size = 3;      // 측정 벡터 크기
  int contr_size = 0;     // 제어 벡터 크기

  s_arm2marker_.create(state_size, 1);
  m_arm2marker_.create(meas_size, 1);
  kf_arm2marker_ = cv::KalmanFilter(state_size, meas_size, contr_size, CV_32F);

  s_odom_.create(state_size, 1);
  m_odom_.create(meas_size, 1);
  kf_odom_ = cv::KalmanFilter(state_size, meas_size, contr_size, CV_32F);

  double period_detect_sec = period_detect_ms_ * 0.001;
  trans_.create(state_size, state_size);
  cv::setIdentity(trans_); // opencv에서 단위 행렬로 초기화
  trans_(0, 3) = period_detect_sec;
  trans_(1, 4) = period_detect_sec;
  trans_(2, 5) = period_detect_sec;

  trans_damp_.create(state_size, state_size);
  trans_.copyTo(trans_damp_); // opencv에서 이미지(mat)을 복사 함. trans_ 이미지를 trans_damp_에 복사
  trans_damp_(3, 3) = kf_vel_damp_;
  trans_damp_(4, 4) = kf_vel_damp_;
  trans_damp_(5, 5) = kf_vel_damp_;

  kf_arm2marker_.transitionMatrix = trans_; //transitionmatrix : 현재상테에서 다음상태로 어떻게 변화하는지 나타내는 행렬, 시스템 동적 모델 나타냄
  kf_odom_.transitionMatrix = trans_damp_;

  cv::Mat_<float> meas(meas_size, state_size);
  meas.setTo(cv::Scalar(0)); //ROI 연산에서 사용. setto는 모든 픽셀을 0으로 설정 한다는 뜻
  meas(0, 0) = 1.0f;
  meas(1, 1) = 1.0f;
  meas(2, 2) = 1.0f;
  kf_arm2marker_.measurementMatrix = meas; //measurementmatrix : 성태변수와 실제 측정값간의 관계를 정의하는 행렬, 측정값ㄷ과 예측값 간의 차이를 계산하여 정확한 상태를 추정.
  kf_odom_.measurementMatrix = meas;

  cv::setIdentity(kf_arm2marker_.processNoiseCov, cv::Scalar::all(1e-2));
  cv::setIdentity(kf_arm2marker_.measurementNoiseCov, cv::Scalar::all(1e-3));
  cv::setIdentity(kf_arm2marker_.errorCovPost, cv::Scalar::all(1.0f));

  cv::setIdentity(kf_odom_.processNoiseCov, cv::Scalar::all(1e-2));
  cv::setIdentity(kf_odom_.measurementNoiseCov, cv::Scalar::all(1e-3));
  cv::setIdentity(kf_odom_.errorCovPost, cv::Scalar::all(1.0f));
}

void MarkerArmPerceptionNode::transAverage( // 마커 위치를 평균적으로 계산 한다는 건가. x,y,z만
    const double& alpha,
    geometry_msgs::msg::TransformStamped& transform1,
    geometry_msgs::msg::TransformStamped& transform2,
    geometry_msgs::msg::TransformStamped& avgTransform)
{
  double beta = 1.0 - alpha;
  avgTransform.transform.translation.x =
    alpha * transform1.transform.translation.x + beta * transform2.transform.translation.x;
  avgTransform.transform.translation.y =
    alpha * transform1.transform.translation.y + beta * transform2.transform.translation.y;
  avgTransform.transform.translation.z =
    alpha * transform1.transform.translation.z + beta * transform2.transform.translation.z;
}

// Function to calculate the SLERP (Spherical Linear Interpolation) average of two quaternions.
// Input: two geometry_msgs::TransformStamped variables containing rotation information.
// Output: The SLERP average of the two quaternions.
void MarkerArmPerceptionNode::slerpAverage(// 두개 쿼터니언 값 사이를 표현, 구면 선형 보간
    const double& alpha,
    geometry_msgs::msg::TransformStamped& transform1, // 이전 위치의 쿼터니온 값
    geometry_msgs::msg::TransformStamped& transform2, // 이후 위치의 쿼터니온 값
    geometry_msgs::msg::TransformStamped& avgTransform)
{
  // Extract quaternion from the TransformStamped messages
  tf2::Quaternion quaternion1(
      transform1.transform.rotation.x,
      transform1.transform.rotation.y,
      transform1.transform.rotation.z,
      transform1.transform.rotation.w
  );

  tf2::Quaternion quaternion2(
      transform2.transform.rotation.x,
      transform2.transform.rotation.y,
      transform2.transform.rotation.z,
      transform2.transform.rotation.w
  );

  // Perform SLERP (Spherical Linear Interpolation) to find the average of the two quaternions
  // Weight for averaging (0.5 for midpoint, adjust as needed)
  tf2::Quaternion slerp_average = quaternion1.slerp(quaternion2, alpha);

  avgTransform.transform.rotation.x = slerp_average.x();
  avgTransform.transform.rotation.y = slerp_average.y();
  avgTransform.transform.rotation.z = slerp_average.z();
  avgTransform.transform.rotation.w = slerp_average.w();

}

void MarkerArmPerceptionNode::markerTf2TimerCallback()
{
  try {
    s_arm2marker_ = kf_arm2marker_.predict();
    if (marker_detected_) {
      kf_arm2marker_.transitionMatrix = trans_;

      geometry_msgs::msg::TransformStamped arm2cam_tf = tf_buffer_->lookupTransform( //link0과 카메라 사이
          link0_axis_frame_, cam_frame_, tf2::TimePointZero);
      geometry_msgs::msg::TransformStamped mes_arm2marker_tf;
      tf2::doTransform(mes_cam2marker_tf_, mes_arm2marker_tf, arm2cam_tf);

      m_arm2marker_(0) = mes_arm2marker_tf.transform.translation.x;
      m_arm2marker_(1) = mes_arm2marker_tf.transform.translation.y;
      m_arm2marker_(2) = mes_arm2marker_tf.transform.translation.z;
      s_arm2marker_ = kf_arm2marker_.correct(m_arm2marker_);

      // seperate simple quaternion filtering
      double quat_filter_weight = 0.1;
      geometry_msgs::msg::TransformStamped tmp_tf;
      slerpAverage(quat_filter_weight, arm2marker_tf_, mes_arm2marker_tf, arm2marker_tf_);

    } else {
      kf_arm2marker_.transitionMatrix = trans_damp_;
    }
    arm2marker_tf_.transform.translation.x = s_arm2marker_(0);
    arm2marker_tf_.transform.translation.y = s_arm2marker_(1);
    arm2marker_tf_.transform.translation.z = s_arm2marker_(2);

    geometry_msgs::msg::TransformStamped cam2arm_tf = tf_buffer_->lookupTransform( //카메라와 link0 사이
        cam_frame_, link0_axis_frame_, tf2::TimePointZero);//cam이 자꾸 움직여서 urdf값만 가지고는 부족해서 두개 링크를 lookup 함수로 측정


    tf2::doTransform(arm2marker_tf_, est_cam2marker_tf_, cam2arm_tf);
    joint_err_angs_[0] = getJointErr(yaw_frame_, cam_frame_, est_cam2marker_tf_); //몸통쪽 조인트 각도
    joint_err_angs_[1] = getJointErr(pitch_frame_, cam_frame_, est_cam2marker_tf_);//카메라쪽 조인트 각도
    // RCLCPP_INFO(this->get_logger(), "[error ] %f, %f", joint_err_angs_[0], joint_err_angs_[1]);

    geometry_msgs::msg::TransformStamped ego_marker2arm_tf = tf_buffer_->lookupTransform( //마커의 중심과 link0 사이
        ego_marker_frame_, link0_axis_frame_, tf2::TimePointZero);
    tf2::doTransform(arm2marker_tf_, marker2marker_tf_, ego_marker2arm_tf);

    // broadcast ego_marker to vis_opp_marker TF only when the marker is detected!
    if (marker_detected_) {
      marker2marker_tf_.header.stamp = this->get_clock()->now();
      tf_broadcaster_->sendTransform(marker2marker_tf_);
    }

    //이 밑에서 부터 오직 fix robot의 마커 설정에 대한 코드
    // For the fixed or master marker module only.
    if (is_fix_robot_ && pub_odom_tf_) {
      bool opp_marker_detected = false;
      geometry_msgs::msg::TransformStamped mov_marker2marker_tf;
      try {
        mov_marker2marker_tf = tf_buffer_->lookupTransform( //
            vis_ego_marker_frame_, opp_marker_frame_, tf2::TimePointZero);
        // mov_marker2marker_tf = tf_buffer_->lookupTransform(
        //     vis_ego_marker_frame_, opp_marker_frame_, this->get_clock()->now(), 5ms);
        double ego_time = marker2marker_tf_.header.stamp.sec + marker2marker_tf_.header.stamp.nanosec * 1e-9;
        double rem_time = mov_marker2marker_tf.header.stamp.sec + mov_marker2marker_tf.header.stamp.nanosec * 1e-9;
        double time_diff = ego_time - rem_time;

        // RCLCPP_INFO(this->get_logger(), "ego %f, remote %f [Diff] %f", ego_time, rem_time, time_diff);
        if (time_diff < 0.5) {  //0.2 sec
          opp_marker_detected = true;
        } else {
          RCLCPP_WARN(this->get_logger(), "[Timeout] %f", time_diff);
        }
      } catch (tf2::TransformException& ex) { //try의 예외출력 키워드
        RCLCPP_ERROR(this->get_logger(), "[1]Failed to lookup transform: %s", ex.what());
      }

      bool detection_combine = true;
      // Combine measurement of ego and remote modules
      if (marker_detected_ && opp_marker_detected) { //내 마커와 상대 마커가 감지 됐다면
        fail_combine_ = 0;
        // RCLCPP_INFO(this->get_logger(), "[1] use both");
        transAverage(combine_m2m_alpha_, marker2marker_tf_, mov_marker2marker_tf, mes_marker2marker_tf_);
        slerpAverage(combine_m2m_alpha_, marker2marker_tf_, mov_marker2marker_tf, mes_marker2marker_tf_);
      } else if (marker_detected_) { //내 마커만 감지 된다면
        fail_combine_ = 0;
        // RCLCPP_WARN(this->get_logger(), "[2] use mine");
        mes_marker2marker_tf_ = marker2marker_tf_;
        // transAverage(combine_m2m_alpha_, marker2marker_tf_, mes_marker2marker_tf_, mes_marker2marker_tf_);
        // slerpAverage(combine_m2m_alpha_, marker2marker_tf_, mes_marker2marker_tf_, mes_marker2marker_tf_);
      } else if (opp_marker_detected) { //상대 마커만 감지 된다면
        fail_combine_ = 0;
        // RCLCPP_ERROR(this->get_logger(), "[3] use remote");
        mes_marker2marker_tf_ = mov_marker2marker_tf;
        // transAverage(combine_m2m_alpha_, mes_marker2marker_tf_, mov_marker2marker_tf, mes_marker2marker_tf_);
        // slerpAverage(combine_m2m_alpha_, mes_marker2marker_tf_, mov_marker2marker_tf, mes_marker2marker_tf_);
      } else { //내 마커, 상대 마커 둘 다 감지 안 되면 출력 됨
        RCLCPP_ERROR(this->get_logger(), "[4] all failed to detect");
        fail_combine_++;
      }
      if (fail_combine_ > 10) {
        detection_combine = false;
      }
      if (detection_combine) { // 내 마커 위치 실제값, 측정값 각각 두개 와 상대 마커 위치 실제값, 측정값 각각 두개를 평균내는 것
        transAverage(update_m2m_alpha_, est_marker2marker_tf_, mes_marker2marker_tf_, est_marker2marker_tf_);
        slerpAverage(update_m2m_alpha_, est_marker2marker_tf_, mes_marker2marker_tf_, est_marker2marker_tf_);

        if (!filter_initialized_) {
          est_marker2marker_tf_ = mes_marker2marker_tf_;
        }

        // from FB (Fixed robot base) to FM (fixed robot marker)      //yaml 파일 보면 MB MM 같았는데,,  1
        geometry_msgs::msg::TransformStamped FB2FM_tf = tf_buffer_->lookupTransform(
          body_base_frame_, ego_marker_frame_, tf2::TimePointZero);

        // from FB to RM
        geometry_msgs::msg::TransformStamped FB2RM_tf; //2
        tf2::doTransform(est_marker2marker_tf_, FB2RM_tf, FB2FM_tf);

        try {
          geometry_msgs::msg::TransformStamped RM2RB_tf = tf_buffer_->lookupTransform( //3
              opp_marker_frame_, opp_base_frame_, tf2::TimePointZero);

          geometry_msgs::msg::TransformStamped mes_FB2RB_tf; //1,2,3의 코사인 법칙으로 FB2RB를 계산 할 수 있게 됨
          tf2::doTransform(RM2RB_tf, mes_FB2RB_tf, FB2RM_tf);

          // only keep yaw and clear roll and pitch
          if (surface_mode_) { //true인 경우 로봇이 평면상에서 움직인다고 가정
            double curr_yaw = tf2::getYaw(mes_FB2RB_tf.transform.rotation);  // include tf2/utils.h 현재 로봇의 방향정보를 보정
            tf2::Quaternion yaw2q;
            yaw2q.setRPY(0, 0, curr_yaw);
            yaw2q = yaw2q.normalize(); //쿼터니언을 정규화
            mes_FB2RB_tf.transform.rotation.x = yaw2q.x();
            mes_FB2RB_tf.transform.rotation.y = yaw2q.y();
            mes_FB2RB_tf.transform.rotation.z = yaw2q.z();
            mes_FB2RB_tf.transform.rotation.w = yaw2q.w();// 쿼터니언을 다시 transform.rotation으로 하여 방향 정보 보정
          }

          geometry_msgs::msg::TransformStamped est_FB2RB_tf = mes_FB2RB_tf;
          est_FB2RB_tf.header.frame_id = body_base_frame_.c_str();
          est_FB2RB_tf.child_frame_id = vis_opp_base_frame_.c_str();
          // est_FB2RB_tf.child_frame_id = opp_base_frame_.c_str();

          kf_odom_.predict(); //로봇의 위치와 방향에 대한 예측값 계산
          m_odom_(0) = mes_FB2RB_tf.transform.translation.x; //로봇의 위치 정보를 m_odom에 저장, 관측된 위치 정보
          m_odom_(1) = mes_FB2RB_tf.transform.translation.y;
          m_odom_(2) = mes_FB2RB_tf.transform.translation.z;
          s_odom_ = kf_odom_.correct(m_odom_); // m_odom(관측된 위치정보)를 기반으로 예측된 로봇의 위치와 방향을 보정하여 s_odom에 저장

          est_FB2RB_tf.transform.translation.x = s_odom_(0);
          est_FB2RB_tf.transform.translation.y = s_odom_(1);
          est_FB2RB_tf.transform.translation.z = s_odom_(2);//위치 정보를 업데이트 
          if (surface_mode_) {
            est_FB2RB_tf.transform.translation.z = 0; //true인 경우 z좌표를 0으로 하여 로봇의 위치를 평면 상으로 고정
          }

          if (FB2RB_tf_init_set_) { //odometry 발행 할 수 있을때 실행
            if (filter_initialized_) {
              // Publish odom
              // marker_odom_.header = est_FB2RB_tf.header;
              // marker_odom_.child_frame_id = est_FB2RB_tf.child_frame_id;
              // marker_odom_.header.stamp = this->get_clock()->now();
              marker_odom_.pose.pose.position.x = est_FB2RB_tf.transform.translation.x;//mov odometry의 위치와 방향 설정
              marker_odom_.pose.pose.position.y = est_FB2RB_tf.transform.translation.y;
              marker_odom_.pose.pose.position.z = est_FB2RB_tf.transform.translation.z;
              if (surface_mode_) {
                marker_odom_.pose.pose.position.z = 0; //로봇을 평면 상에 고정
              }
              marker_odom_.pose.pose.orientation = est_FB2RB_tf.transform.rotation;

              try {
                geometry_msgs::msg::TransformStamped OD2RB_tf = tf_buffer_->lookupTransform(
                      opp_odom_frame_, opp_base_frame_, tf2::TimePointZero);//외부 시스템에서 받은 odometry date를 현재 시스템의 프레임으로 변환

                geometry_msgs::msg::TransformStamped gt_FB2RB_tf;// 변환한 값을 gt_FB2RB_tf에 저장
                tf2::doTransform(OD2RB_tf, gt_FB2RB_tf, FB2OD_tf_);
                gt_marker_odom_.header.stamp = this->get_clock()->now(); //gt_FB2RB_tf 오도메트리 기반 gt_marker_odom 설정
                gt_marker_odom_.pose.pose.position.x = gt_FB2RB_tf.transform.translation.x;//marker odom 발행->칼만필터로 보정된 로봇의 위치와 방향정보를 포함한 오도메트리 발행
                gt_marker_odom_.pose.pose.position.y = gt_FB2RB_tf.transform.translation.y;
                gt_marker_odom_.pose.pose.position.z = gt_FB2RB_tf.transform.translation.z;
                gt_marker_odom_.pose.pose.orientation = gt_FB2RB_tf.transform.rotation;

                marker_odom_.header.stamp = gt_marker_odom_.header.stamp;
                marker_odom_pub_->publish(marker_odom_);
                gt_marker_odom_pub_->publish(gt_marker_odom_);
              } catch (tf2::TransformException& ex) {
                RCLCPP_ERROR(this->get_logger(), "[4]Failed to lookup transform: %s", ex.what());
              }
            }

            FB2OD_tf_.header.stamp = this->get_clock()->now();
            tf_broadcaster_->sendTransform(FB2OD_tf_);

            // Broadcast tf
            est_FB2RB_tf.header.stamp = marker2marker_tf_.header.stamp;
            tf_broadcaster_->sendTransform(est_FB2RB_tf);

          } else {//odometry 발행 할 수 없을때
            est_FB2RB_tf.header.stamp = marker2marker_tf_.header.stamp;
            tf_broadcaster_->sendTransform(est_FB2RB_tf);
            RCLCPP_INFO(this->get_logger(), "Ang err (%d) = (%.3f, %.3f)", FB2RB_counter_, fabs(joint_err_angs_[0]), fabs(joint_err_angs_[1]));


            if (fabs(joint_err_angs_[0]) < FB2RB_thres_ang_ && fabs(joint_err_angs_[1]) < FB2RB_thres_ang_) { //현재의 조인트 에러각도가 FB2RB_thres_ang 보다 작은지 확인
              FB2RB_counter_++;
              if (FB2RB_counter_ > 100) { // 작을때 실행
                try {
                  geometry_msgs::msg::TransformStamped RB2OD_tf = tf_buffer_->lookupTransform(
                      opp_base_frame_, opp_odom_frame_, tf2::TimePointZero);

                  // Assume both are initially on the same flat surface
                  est_FB2RB_tf.transform.translation.z = 0; // 로봇이 z축으로 이동하지 않는다고 설정
                  est_FB2RB_tf.transform.rotation.x = 0; //로봇의 x축 회전을 0으로 설정하여 로봇의 회전을 기준으로 정렬된다는 의미
                  est_FB2RB_tf.transform.rotation.y = 0; // y 축의 회전을 0으로 설정, 로봇의 회전을 기준으로 정렬된다는 의미
                  tf2::doTransform(RB2OD_tf, FB2OD_tf_, est_FB2RB_tf); //오도메트리 변환

                  RCLCPP_ERROR(this->get_logger(), "Set FB2OD!"); //오도메트리 데이터가 설정되어 로그에 기록
                  FB2RB_tf_init_set_ = true;

                } catch (tf2::TransformException& ex) { 
                  RCLCPP_ERROR(this->get_logger(), "[3]Failed to lookup transform: %s", ex.what());
                }
              }
            } else {
              RCLCPP_INFO(this->get_logger(), "max cnt = %d", FB2RB_counter_);//에러 각도가 클때 
              FB2RB_counter_ = 0;
            }
          }


        } catch (tf2::TransformException& ex) {
          RCLCPP_ERROR(this->get_logger(), "[2]Failed to lookup transform: %s", ex.what());
        }
      }
    }

    if (filter_initialized_) {
      joint_err_state_.header.stamp = this->get_clock()->now();
      joint_err_state_.name = {"joint1_err", "joint2_err"};  // Example joint names
      joint_err_state_.position = {joint_err_angs_[0], joint_err_angs_[1]};  // Example joint positions
      joint_err_pub_->publish(joint_err_state_);
      // RCLCPP_INFO(this->get_logger(), "[pub] Yaw, Pitch: %f, %f", joint_err_angs_[0], joint_err_angs_[1]);
    }
  } catch (tf2::TransformException& ex) {
    RCLCPP_ERROR(this->get_logger(), "Failed to lookup transform: %s", ex.what());
  }

  if (marker_detected_) {
    marker_detected_ = false;
    if (!filter_initialized_) {
      filter_initialized_ = true;
    }
  }
}

void MarkerArmPerceptionNode::markerPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr _msg)
{
  marker_detected_ = true;
  mes_cam2marker_tf_.transform.translation.x = _msg->pose.position.x;
  mes_cam2marker_tf_.transform.translation.y = _msg->pose.position.y;
  mes_cam2marker_tf_.transform.translation.z = _msg->pose.position.z;
  mes_cam2marker_tf_.transform.rotation = _msg->pose.orientation;
}

MarkerArmPerceptionNode::~MarkerArmPerceptionNode()
{

}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto marker_arm_perception_node = std::make_shared<MarkerArmPerceptionNode>();
  rclcpp::spin(marker_arm_perception_node);
  rclcpp::shutdown();
  return 0;
}

