#include "spot_micro_motion_cmd.h"

#include <eigen3/Eigen/Geometry>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "spot_micro_motion_cmd.h"
#include "spot_micro_kinematics/spot_micro_kinematics.h"
#include "i2cpwm_board/msg/servo.hpp"
#include "i2cpwm_board/msg/servo_array.hpp"
#include "i2cpwm_board/msg/servo_config.hpp"
#include "i2cpwm_board/srv/servos_config.hpp"
#include "spot_micro_idle.h"
#include "utils.h"

using namespace smk;
using namespace Eigen;
using namespace geometry_msgs::msg;
typedef std::vector<std::pair<std::string,std::string>> VectorStringPairs;

SpotMicroMotionCmd::SpotMicroMotionCmd() : Node("spot_micro_motion_cmd") {

  if (smnc_.debug_mode) {
    std::cout<<"from Constructor \n";
  }

  cmd_ = Command();
  state_ = std::make_unique<SpotMicroIdleState>();
  readInConfigParameters();
  sm_ = smk::SpotMicroKinematics(0.0f, 0.0f, 0.0f, smnc_.smc);

  body_state_cmd_.euler_angs = {.phi = 0.0f, .theta = 0.0f, .psi = 0.0f};
  body_state_cmd_.xyz_pos = {.x = 0.0f, .y = smnc_.lie_down_height, .z = 0.0f};
  body_state_cmd_.leg_feet_pos = getLieDownStance();

  sm_.setBodyState(body_state_cmd_);

  robot_odometry_.euler_angs = {.phi = 0.0f, .theta = 0.0f, .psi = 0.0f};
  robot_odometry_.xyz_pos = {.x = 0.0f, .y = 0.0f, .z = 0.0f};

  for (int i = 1; i <= smnc_.num_servos; i++) {
    i2cpwm_board::msg::Servo temp_servo;
    temp_servo.servo = i;
    temp_servo.value = 0.0f;
    servo_array_.servos.push_back(temp_servo);
  }

  servo_array_absolute_.servos = servo_array_.servos;

  stand_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/stand_cmd", 1, std::bind(&SpotMicroMotionCmd::standCommandCallback, this, std::placeholders::_1));
  idle_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/idle_cmd", 1, std::bind(&SpotMicroMotionCmd::idleCommandCallback, this, std::placeholders::_1));
  walk_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/walk_cmd", 1, std::bind(&SpotMicroMotionCmd::walkCommandCallback, this, std::placeholders::_1));
  body_angle_cmd_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
      "/angle_cmd", 1, std::bind(&SpotMicroMotionCmd::angleCommandCallback, this, std::placeholders::_1));
  vel_cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 1, std::bind(&SpotMicroMotionCmd::velCommandCallback, this, std::placeholders::_1));

  servos_absolute_pub_ = this->create_publisher<i2cpwm_board::msg::ServoArray>("servos_absolute", 1);
  servos_proportional_pub_ = this->create_publisher<i2cpwm_board::msg::ServoArray>("servos_proportional", 1);
  servos_config_client_ = this->create_client<i2cpwm_board::srv::ServosConfig>("config_servos");
  body_state_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("body_state", 1);
  lcd_state_pub_ = this->create_publisher<std_msgs::msg::String>("lcd_state", 1);
  lcd_vel_cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("lcd_vel_cmd", 1);
  lcd_angle_cmd_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("lcd_angle_cmd", 1);

  lcd_state_string_msg_.data = "Idle";

  lcd_vel_cmd_msg_.linear.x = 0.0f;
  lcd_vel_cmd_msg_.linear.y = 0.0f;
  lcd_vel_cmd_msg_.linear.z = 0.0f;
  lcd_vel_cmd_msg_.angular.x = 0.0f;
  lcd_vel_cmd_msg_.angular.y = 0.0f;
  lcd_vel_cmd_msg_.angular.z = 0.0f;

  lcd_angle_cmd_msg_.x = 0.0f;
  lcd_angle_cmd_msg_.y = 0.0f;
  lcd_angle_cmd_msg_.z = 0.0f;

  if (smnc_.plot_mode) {
    for (int i = 0; i < 18; i++) {
      body_state_msg_.data.push_back(0.0f);
    }
  }

  transform_br_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  static_transform_br_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

  publishStaticTransforms();
}

SpotMicroMotionCmd::~SpotMicroMotionCmd() {
  if (smnc_.debug_mode) {
    std::cout<<"from Destructor \n";
  }
}

void SpotMicroMotionCmd::runOnce() {
  if (smnc_.debug_mode) {
    std::cout<<"from Runonce \n";
  }

  handleInputCommands();
  resetEventCommands();

  if (smnc_.plot_mode) {
    publishBodyState();
  }

  publishLcdMonitorData();
  publishDynamicTransforms();

  if (smnc_.publish_odom) {
    integrateOdometry();
  }
}

bool SpotMicroMotionCmd::publishServoConfiguration() {
  auto request = std::make_shared<i2cpwm_board::srv::ServosConfig::Request>();

  for (auto iter = smnc_.servo_config.begin();
       iter != smnc_.servo_config.end();
       ++iter) {

    std::map<std::string, float> servo_config_params = iter->second;
    i2cpwm_board::msg::ServoConfig temp_servo_config;
    temp_servo_config.center = static_cast<int32_t>(servo_config_params["center"]);
    temp_servo_config.range = static_cast<int32_t>(servo_config_params["range"]);
    temp_servo_config.servo = static_cast<int32_t>(servo_config_params["num"]);
    temp_servo_config.direction = static_cast<int32_t>(servo_config_params["direction"]);

    request->servos.push_back(temp_servo_config);
  }

  while (!servos_config_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service");
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "Waiting for servo config service...");
  }

  auto future = servos_config_client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(shared_from_this(), future) == rclcpp::FutureReturnCode::SUCCESS) {
    return future.get()->success;
  }

  if (!smnc_.debug_mode) {
    RCLCPP_ERROR(this->get_logger(), "Failed to call service servo_config");
  }
  return false;
}

void SpotMicroMotionCmd::setServoCommandMessageData() {
  sm_.setBodyState(body_state_cmd_);
  LegsJointAngles joint_angs = sm_.getLegsJointAngles();

  servo_cmds_rad_["RF_1"] = joint_angs.right_front.ang1;
  servo_cmds_rad_["RF_2"] = joint_angs.right_front.ang2;
  servo_cmds_rad_["RF_3"] = joint_angs.right_front.ang3;

  servo_cmds_rad_["RB_1"] = joint_angs.right_back.ang1;
  servo_cmds_rad_["RB_2"] = joint_angs.right_back.ang2;
  servo_cmds_rad_["RB_3"] = joint_angs.right_back.ang3;

  servo_cmds_rad_["LF_1"] = joint_angs.left_front.ang1;
  servo_cmds_rad_["LF_2"] = joint_angs.left_front.ang2;
  servo_cmds_rad_["LF_3"] = joint_angs.left_front.ang3;

  servo_cmds_rad_["LB_1"] = joint_angs.left_back.ang1;
  servo_cmds_rad_["LB_2"] = joint_angs.left_back.ang2;
  servo_cmds_rad_["LB_3"] = joint_angs.left_back.ang3;
}

void SpotMicroMotionCmd::publishServoProportionalCommand() {
  for (auto iter = smnc_.servo_config.begin();
       iter != smnc_.servo_config.end();
       ++iter) {

    std::string servo_name = iter->first;
    std::map<std::string, float> servo_config_params = iter->second;

    int servo_num = static_cast<int>(servo_config_params["num"]);
    float cmd_ang_rad = servo_cmds_rad_[servo_name];
    float center_ang_rad = servo_config_params["center_angle_deg"]*M_PI/180.0f;
    float servo_proportional_cmd = (cmd_ang_rad - center_ang_rad) /
                                   (smnc_.servo_max_angle_deg*M_PI/180.0f);

    if (servo_proportional_cmd > 1.0f) {
      servo_proportional_cmd = 1.0f;
      RCLCPP_WARN(this->get_logger(), "Proportional Command above +1.0 was computed, clipped to 1.0");
      RCLCPP_WARN(this->get_logger(), "Joint %s, Angle: %1.2f", servo_name.c_str(), cmd_ang_rad*180.0/M_PI);

    } else if (servo_proportional_cmd < -1.0f) {
      servo_proportional_cmd = -1.0f;
      RCLCPP_WARN(this->get_logger(), "Proportional Command below -1.0 was computed, clipped to -1.0");
      RCLCPP_WARN(this->get_logger(), "Joint %s, Angle: %1.2f", servo_name.c_str(), cmd_ang_rad*180.0/M_PI);
    }

    servo_array_.servos[servo_num-1].servo = servo_num;
    servo_array_.servos[servo_num-1].value = servo_proportional_cmd;
  }

  servos_proportional_pub_->publish(servo_array_);
}

void SpotMicroMotionCmd::publishZeroServoAbsoluteCommand() {
  servos_absolute_pub_->publish(servo_array_absolute_);
}

SpotMicroNodeConfig SpotMicroMotionCmd::getNodeConfig() {
  return smnc_;
}

LegsFootPos SpotMicroMotionCmd::getNeutralStance() {
  float len = smnc_.smc.body_length;
  float width = smnc_.smc.body_width;
  float l1 = smnc_.smc.hip_link_length;
  float f_offset = smnc_.stand_front_x_offset;
  float b_offset = smnc_.stand_back_x_offset;

  LegsFootPos neutral_stance;
  neutral_stance.right_back  = {.x = -len/2 + b_offset, .y = 0.0f, .z =  width/2 + l1};
  neutral_stance.right_front = {.x =  len/2 + f_offset, .y = 0.0f, .z =  width/2 + l1};
  neutral_stance.left_front  = {.x =  len/2 + f_offset, .y = 0.0f, .z = -width/2 - l1};
  neutral_stance.left_back   = {.x = -len/2 + b_offset, .y = 0.0f, .z = -width/2 - l1};

  return neutral_stance;
}

LegsFootPos SpotMicroMotionCmd::getLieDownStance() {
  float len = smnc_.smc.body_length;
  float width = smnc_.smc.body_width;
  float l1 = smnc_.smc.hip_link_length;
  float x_off = smnc_.lie_down_feet_x_offset;

  LegsFootPos lie_down_stance;
  lie_down_stance.right_back  = {.x = -len/2 + x_off, .y = 0.0f, .z =  width/2 + l1};
  lie_down_stance.right_front = {.x =  len/2 + x_off, .y = 0.0f, .z =  width/2 + l1};
  lie_down_stance.left_front  = {.x =  len/2 + x_off, .y = 0.0f, .z = -width/2 - l1};
  lie_down_stance.left_back   = {.x = -len/2 + x_off, .y = 0.0f, .z = -width/2 - l1};

  return lie_down_stance;
}

void SpotMicroMotionCmd::commandIdle() {
  cmd_.idle_cmd_ = true;
}

std::string SpotMicroMotionCmd::getCurrentStateName() {
  return state_->getCurrentStateName();
}

void SpotMicroMotionCmd::readInConfigParameters() {
  this->declare_parameter<double>("hip_link_length", 0.0);
  this->declare_parameter<double>("upper_leg_link_length", 0.0);
  this->declare_parameter<double>("lower_leg_link_length", 0.0);
  this->declare_parameter<double>("body_width", 0.0);
  this->declare_parameter<double>("body_length", 0.0);
  this->declare_parameter<double>("default_stand_height", 0.0);
  this->declare_parameter<double>("stand_front_x_offset", 0.0);
  this->declare_parameter<double>("stand_back_x_offset", 0.0);
  this->declare_parameter<double>("lie_down_height", 0.0);
  this->declare_parameter<double>("lie_down_foot_x_offset", 0.0);
  this->declare_parameter<int>("num_servos", 12);
  this->declare_parameter<double>("servo_max_angle_deg", 0.0);
  this->declare_parameter<double>("transit_tau", 0.0);
  this->declare_parameter<double>("transit_rl", 0.0);
  this->declare_parameter<double>("transit_angle_rl", 0.0);
  this->declare_parameter<double>("dt", 0.02);
  this->declare_parameter<bool>("debug_mode", false);
  this->declare_parameter<bool>("plot_mode", false);
  this->declare_parameter<double>("max_fwd_velocity", 0.0);
  this->declare_parameter<double>("max_side_velocity", 0.0);
  this->declare_parameter<double>("max_yaw_rate", 0.0);
  this->declare_parameter<double>("z_clearance", 0.0);
  this->declare_parameter<double>("alpha", 0.0);
  this->declare_parameter<double>("beta", 0.0);
  this->declare_parameter<int>("num_phases", 4);
  this->declare_parameter<std::vector<int64_t>>("rb_contact_phases", {});
  this->declare_parameter<std::vector<int64_t>>("rf_contact_phases", {});
  this->declare_parameter<std::vector<int64_t>>("lf_contact_phases", {});
  this->declare_parameter<std::vector<int64_t>>("lb_contact_phases", {});
  this->declare_parameter<double>("overlap_time", 0.0);
  this->declare_parameter<double>("swing_time", 0.0);
  this->declare_parameter<double>("foot_height_time_constant", 0.0);
  this->declare_parameter<std::vector<int64_t>>("body_shift_phases", {});
  this->declare_parameter<double>("fwd_body_balance_shift", 0.0);
  this->declare_parameter<double>("back_body_balance_shift", 0.0);
  this->declare_parameter<double>("side_body_balance_shift", 0.0);
  this->declare_parameter<bool>("publish_odom", false);
  this->declare_parameter<double>("lidar_x_pos", 0.0);
  this->declare_parameter<double>("lidar_y_pos", 0.0);
  this->declare_parameter<double>("lidar_z_pos", 0.0);
  this->declare_parameter<double>("lidar_yaw_angle", 0.0);

  smnc_.smc.hip_link_length = static_cast<float>(this->get_parameter("hip_link_length").as_double());
  smnc_.smc.upper_leg_link_length = static_cast<float>(this->get_parameter("upper_leg_link_length").as_double());
  smnc_.smc.lower_leg_link_length = static_cast<float>(this->get_parameter("lower_leg_link_length").as_double());
  smnc_.smc.body_width = static_cast<float>(this->get_parameter("body_width").as_double());
  smnc_.smc.body_length = static_cast<float>(this->get_parameter("body_length").as_double());
  smnc_.default_stand_height = static_cast<float>(this->get_parameter("default_stand_height").as_double());
  smnc_.stand_front_x_offset = static_cast<float>(this->get_parameter("stand_front_x_offset").as_double());
  smnc_.stand_back_x_offset = static_cast<float>(this->get_parameter("stand_back_x_offset").as_double());
  smnc_.lie_down_height = static_cast<float>(this->get_parameter("lie_down_height").as_double());
  smnc_.lie_down_feet_x_offset = static_cast<float>(this->get_parameter("lie_down_foot_x_offset").as_double());
  smnc_.num_servos = this->get_parameter("num_servos").as_int();
  smnc_.servo_max_angle_deg = static_cast<float>(this->get_parameter("servo_max_angle_deg").as_double());
  smnc_.transit_tau = static_cast<float>(this->get_parameter("transit_tau").as_double());
  smnc_.transit_rl = static_cast<float>(this->get_parameter("transit_rl").as_double());
  smnc_.transit_angle_rl = static_cast<float>(this->get_parameter("transit_angle_rl").as_double());
  smnc_.dt = static_cast<float>(this->get_parameter("dt").as_double());
  smnc_.debug_mode = this->get_parameter("debug_mode").as_bool();
  smnc_.plot_mode = this->get_parameter("plot_mode").as_bool();
  smnc_.max_fwd_velocity = static_cast<float>(this->get_parameter("max_fwd_velocity").as_double());
  smnc_.max_side_velocity = static_cast<float>(this->get_parameter("max_side_velocity").as_double());
  smnc_.max_yaw_rate = static_cast<float>(this->get_parameter("max_yaw_rate").as_double());
  smnc_.z_clearance = static_cast<float>(this->get_parameter("z_clearance").as_double());
  smnc_.alpha = static_cast<float>(this->get_parameter("alpha").as_double());
  smnc_.beta = static_cast<float>(this->get_parameter("beta").as_double());
  smnc_.num_phases = static_cast<int>(this->get_parameter("num_phases").as_int());
  smnc_.rb_contact_phases = this->get_parameter("rb_contact_phases").as_integer_array();
  smnc_.rf_contact_phases = this->get_parameter("rf_contact_phases").as_integer_array();
  smnc_.lf_contact_phases = this->get_parameter("lf_contact_phases").as_integer_array();
  smnc_.lb_contact_phases = this->get_parameter("lb_contact_phases").as_integer_array();
  smnc_.overlap_time = static_cast<float>(this->get_parameter("overlap_time").as_double());
  smnc_.swing_time = static_cast<float>(this->get_parameter("swing_time").as_double());
  smnc_.foot_height_time_constant = static_cast<float>(this->get_parameter("foot_height_time_constant").as_double());
  smnc_.body_shift_phases = this->get_parameter("body_shift_phases").as_integer_array();
  smnc_.fwd_body_balance_shift = static_cast<float>(this->get_parameter("fwd_body_balance_shift").as_double());
  smnc_.back_body_balance_shift = static_cast<float>(this->get_parameter("back_body_balance_shift").as_double());
  smnc_.side_body_balance_shift = static_cast<float>(this->get_parameter("side_body_balance_shift").as_double());
  smnc_.publish_odom = this->get_parameter("publish_odom").as_bool();
  smnc_.lidar_x_pos = static_cast<float>(this->get_parameter("lidar_x_pos").as_double());
  smnc_.lidar_y_pos = static_cast<float>(this->get_parameter("lidar_y_pos").as_double());
  smnc_.lidar_z_pos = static_cast<float>(this->get_parameter("lidar_z_pos").as_double());
  smnc_.lidar_yaw_angle = static_cast<float>(this->get_parameter("lidar_yaw_angle").as_double());

  smnc_.overlap_ticks = static_cast<int>(round(smnc_.overlap_time / smnc_.dt));
  smnc_.swing_ticks = static_cast<int>(round(smnc_.swing_time / smnc_.dt));

  if (smnc_.num_phases == 8) {
    smnc_.stance_ticks = 7 * smnc_.swing_ticks;
    smnc_.overlap_ticks = static_cast<int>(round(smnc_.overlap_time / smnc_.dt));
    smnc_.phase_ticks = std::vector<int64_t>
        {smnc_.swing_ticks, smnc_.swing_ticks, smnc_.swing_ticks, smnc_.swing_ticks,
         smnc_.swing_ticks, smnc_.swing_ticks, smnc_.swing_ticks, smnc_.swing_ticks};
    smnc_.phase_length = smnc_.num_phases * smnc_.swing_ticks;

  } else {
    smnc_.stance_ticks = 2 * smnc_.overlap_ticks + smnc_.swing_ticks;
    smnc_.overlap_ticks = static_cast<int>(round(smnc_.overlap_time / smnc_.dt));
    smnc_.phase_ticks = std::vector<int64_t>
        {smnc_.overlap_ticks, smnc_.swing_ticks, smnc_.overlap_ticks, smnc_.swing_ticks};
    smnc_.phase_length = 2 * smnc_.swing_ticks + 2 * smnc_.overlap_ticks;
  }

  for (auto iter = servo_cmds_rad_.begin();
       iter != servo_cmds_rad_.end();
       ++iter) {

    std::string servo_name = iter->first;
    this->declare_parameter<int>(servo_name + ".num", 0);
    this->declare_parameter<int>(servo_name + ".center", 0);
    this->declare_parameter<int>(servo_name + ".range", 0);
    this->declare_parameter<int>(servo_name + ".direction", 0);
    this->declare_parameter<double>(servo_name + ".center_angle_deg", 0.0);

    std::map<std::string, float> temp_map;
    temp_map["num"] = static_cast<float>(this->get_parameter(servo_name + ".num").as_int());
    temp_map["center"] = static_cast<float>(this->get_parameter(servo_name + ".center").as_int());
    temp_map["range"] = static_cast<float>(this->get_parameter(servo_name + ".range").as_int());
    temp_map["direction"] = static_cast<float>(this->get_parameter(servo_name + ".direction").as_int());
    temp_map["center_angle_deg"] = static_cast<float>(this->get_parameter(servo_name + ".center_angle_deg").as_double());
    smnc_.servo_config[servo_name] = temp_map;
  }
}

void SpotMicroMotionCmd::standCommandCallback(
    const std_msgs::msg::Bool::SharedPtr msg) {
  if (msg->data == true) {cmd_.stand_cmd_ = true;}
}

void SpotMicroMotionCmd::idleCommandCallback(
    const std_msgs::msg::Bool::SharedPtr msg) {
  if (msg->data == true) {cmd_.idle_cmd_ = true;}
}

void SpotMicroMotionCmd::walkCommandCallback(
    const std_msgs::msg::Bool::SharedPtr msg) {
  if (msg->data == true) {cmd_.walk_cmd_ = true;}
}

void SpotMicroMotionCmd::angleCommandCallback(
    const geometry_msgs::msg::Vector3::SharedPtr msg) {
  cmd_.phi_cmd_ = msg->x;
  cmd_.theta_cmd_ = msg->y;
  cmd_.psi_cmd_ = msg->z;
}

void SpotMicroMotionCmd::velCommandCallback(
    const geometry_msgs::msg::Twist::SharedPtr msg) {
  cmd_.x_vel_cmd_mps_ = msg->linear.x;
  cmd_.y_vel_cmd_mps_ = msg->linear.y;
  cmd_.yaw_rate_cmd_rps_ = msg->angular.z;
}

void SpotMicroMotionCmd::resetEventCommands() {
  cmd_.resetEventCmds();
}

void SpotMicroMotionCmd::handleInputCommands() {
  state_->handleInputCommands(sm_.getBodyState(), smnc_, cmd_, this, &body_state_cmd_);
}

void SpotMicroMotionCmd::changeState(std::unique_ptr<SpotMicroState> sms) {
  state_ = std::move(sms);
  state_->init(sm_.getBodyState(), smnc_, cmd_, this);
  cmd_.resetAllCommands();
}

void SpotMicroMotionCmd::publishBodyState() {
  body_state_msg_.data[0] = body_state_cmd_.leg_feet_pos.right_back.x;
  body_state_msg_.data[1] = body_state_cmd_.leg_feet_pos.right_back.y;
  body_state_msg_.data[2] = body_state_cmd_.leg_feet_pos.right_back.z;

  body_state_msg_.data[3] = body_state_cmd_.leg_feet_pos.right_front.x;
  body_state_msg_.data[4] = body_state_cmd_.leg_feet_pos.right_front.y;
  body_state_msg_.data[5] = body_state_cmd_.leg_feet_pos.right_front.z;

  body_state_msg_.data[6] = body_state_cmd_.leg_feet_pos.left_front.x;
  body_state_msg_.data[7] = body_state_cmd_.leg_feet_pos.left_front.y;
  body_state_msg_.data[8] = body_state_cmd_.leg_feet_pos.left_front.z;

  body_state_msg_.data[9] = body_state_cmd_.leg_feet_pos.left_back.x;
  body_state_msg_.data[10] = body_state_cmd_.leg_feet_pos.left_back.y;
  body_state_msg_.data[11] = body_state_cmd_.leg_feet_pos.left_back.z;

  body_state_msg_.data[12] = body_state_cmd_.xyz_pos.x;
  body_state_msg_.data[13] = body_state_cmd_.xyz_pos.y;
  body_state_msg_.data[14] = body_state_cmd_.xyz_pos.z;

  body_state_msg_.data[15] = body_state_cmd_.euler_angs.phi;
  body_state_msg_.data[16] = body_state_cmd_.euler_angs.theta;
  body_state_msg_.data[17] = body_state_cmd_.euler_angs.psi;

  body_state_pub_->publish(body_state_msg_);
}

void SpotMicroMotionCmd::publishLcdMonitorData() {
  lcd_state_string_msg_.data = getCurrentStateName();

  lcd_vel_cmd_msg_.linear.x = cmd_.getXSpeedCmd();
  lcd_vel_cmd_msg_.linear.y = cmd_.getYSpeedCmd();
  lcd_vel_cmd_msg_.angular.z = cmd_.getYawRateCmd();

  lcd_angle_cmd_msg_.x = cmd_.getPhiCmd();
  lcd_angle_cmd_msg_.y = cmd_.getThetaCmd();
  lcd_angle_cmd_msg_.z = cmd_.getPsiCmd();

  lcd_state_pub_->publish(lcd_state_string_msg_);
  lcd_vel_cmd_pub_->publish(lcd_vel_cmd_msg_);
  lcd_angle_cmd_pub_->publish(lcd_angle_cmd_msg_);
}

void SpotMicroMotionCmd::publishStaticTransforms() {

  TransformStamped tr_stamped;
  auto now = this->now();

  tr_stamped = createTransform("base_link", "front_link",
                               0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, now);
  static_transform_br_->sendTransform(tr_stamped);

  tr_stamped = createTransform("base_link", "rear_link",
                               0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, now);
  static_transform_br_->sendTransform(tr_stamped);

  float x_offset = smnc_.lidar_x_pos;
  float y_offset = smnc_.lidar_y_pos;
  float z_offset = smnc_.lidar_z_pos;
  float yaw_angle = smnc_.lidar_yaw_angle*M_PI/180.0;
  tr_stamped = createTransform("base_link", "lidar_link",
                               x_offset, y_offset, z_offset,
                               0.0, 0.0, yaw_angle, now);
  static_transform_br_->sendTransform(tr_stamped);

  const VectorStringPairs leg_cover_pairs {
      { "front_left_leg_link",  "front_left_leg_link_cover" },
      { "front_right_leg_link", "front_right_leg_link_cover"},
      { "rear_right_leg_link",  "rear_right_leg_link_cover" },
      { "rear_left_leg_link",   "rear_left_leg_link_cover" }};

  for (auto it = leg_cover_pairs.begin(); it != leg_cover_pairs.end(); it++) {
    tr_stamped = createTransform(it->first, it->second,
                               0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, now);
    static_transform_br_->sendTransform(tr_stamped);
  }

  const VectorStringPairs foot_toe_pairs {
      { "front_left_foot_link",  "front_left_toe_link" },
      { "front_right_foot_link", "front_right_toe_link"},
      { "rear_right_foot_link",  "rear_right_toe_link" },
      { "rear_left_foot_link",   "rear_left_toe_link" }};

  for (auto it = foot_toe_pairs.begin(); it != foot_toe_pairs.end(); it++) {
    tr_stamped = createTransform(it->first, it->second,
                               0.0, 0.0, -0.13,
                               0.0, 0.0, 0.0, now);
    static_transform_br_->sendTransform(tr_stamped);
  }
}

void SpotMicroMotionCmd::publishDynamicTransforms() {

  LegsJointAngles joint_angs = sm_.getLegsJointAngles();

  TransformStamped transform_stamped;
  Affine3d temp_trans;
  auto now = this->now();

  if (smnc_.publish_odom) {
    transform_stamped = eigAndFramesToTrans(getOdometryTransform(), "odom", "base_footprint", now);
    transform_br_->sendTransform(transform_stamped);
  }

  temp_trans = matrix4fToAffine3d(sm_.getBodyHt());

  temp_trans =  AngleAxisd(M_PI/2.0, Vector3d::UnitX()) *
                temp_trans *
                AngleAxisd(-M_PI/2.0, Vector3d::UnitX());

  transform_stamped = eigAndFramesToTrans(temp_trans, "base_footprint", "base_link", now);
  transform_br_->sendTransform(transform_stamped);

  transform_stamped = createTransform("base_link", "front_right_shoulder_link",
                                      smnc_.smc.body_length/2.0, -smnc_.smc.body_width/2.0, 0.0,
                                      joint_angs.right_front.ang1, 0.0, 0.0, now);
  transform_br_->sendTransform(transform_stamped);

  transform_stamped = createTransform("front_right_shoulder_link","front_right_leg_link",
                                      0.0, -smnc_.smc.hip_link_length, 0.0,
                                      0.0, -joint_angs.right_front.ang2, 0.0, now);
  transform_br_->sendTransform(transform_stamped);

  transform_stamped = createTransform("front_right_leg_link","front_right_foot_link",
                                      0.0, 0.0, -smnc_.smc.upper_leg_link_length,
                                      0.0, -joint_angs.right_front.ang3, 0.0, now);
  transform_br_->sendTransform(transform_stamped);

  transform_stamped = createTransform("base_link", "rear_right_shoulder_link",
                                      -smnc_.smc.body_length/2.0, -smnc_.smc.body_width/2.0, 0.0,
                                      joint_angs.right_back.ang1, 0.0, 0.0, now);
  transform_br_->sendTransform(transform_stamped);

  transform_stamped = createTransform("rear_right_shoulder_link","rear_right_leg_link",
                                      0.0, -smnc_.smc.hip_link_length, 0.0,
                                      0.0, -joint_angs.right_back.ang2, 0.0, now);
  transform_br_->sendTransform(transform_stamped);

  transform_stamped = createTransform("rear_right_leg_link","rear_right_foot_link",
                                      0.0, 0.0, -smnc_.smc.upper_leg_link_length,
                                      0.0, -joint_angs.right_back.ang3, 0.0, now);
  transform_br_->sendTransform(transform_stamped);

  transform_stamped = createTransform("base_link", "front_left_shoulder_link",
                                      smnc_.smc.body_length/2.0, smnc_.smc.body_width/2.0, 0.0,
                                      -joint_angs.left_front.ang1, 0.0, 0.0, now);
  transform_br_->sendTransform(transform_stamped);

  transform_stamped = createTransform("front_left_shoulder_link","front_left_leg_link",
                                      0.0, smnc_.smc.hip_link_length, 0.0,
                                      0.0, joint_angs.left_front.ang2, 0.0, now);
  transform_br_->sendTransform(transform_stamped);

  transform_stamped = createTransform("front_left_leg_link","front_left_foot_link",
                                      0.0, 0.0, -smnc_.smc.upper_leg_link_length,
                                      0.0, joint_angs.left_front.ang3, 0.0, now);
  transform_br_->sendTransform(transform_stamped);

  transform_stamped = createTransform("base_link", "rear_left_shoulder_link",
                                      -smnc_.smc.body_length/2.0, smnc_.smc.body_width/2.0, 0.0,
                                      -joint_angs.left_back.ang1, 0.0, 0.0, now);
  transform_br_->sendTransform(transform_stamped);

  transform_stamped = createTransform("rear_left_shoulder_link","rear_left_leg_link",
                                      0.0, smnc_.smc.hip_link_length, 0.0,
                                      0.0, joint_angs.left_back.ang2, 0.0, now);
  transform_br_->sendTransform(transform_stamped);

  transform_stamped = createTransform("rear_left_leg_link","rear_left_foot_link",
                                      0.0, 0.0, -smnc_.smc.upper_leg_link_length,
                                      0.0, joint_angs.left_back.ang3, 0.0, now);
  transform_br_->sendTransform(transform_stamped);
}

void SpotMicroMotionCmd::integrateOdometry() {
  float dt = smnc_.dt;
  float psi = robot_odometry_.euler_angs.psi;
  float x_spd = cmd_.getXSpeedCmd();
  float y_spd = -cmd_.getYSpeedCmd();
  float yaw_rate = -cmd_.getYawRateCmd();

  float x_dot = x_spd*cos(psi) - y_spd*sin(psi);
  float y_dot = x_spd*sin(psi) + y_spd*cos(psi);
  float yaw_dot = yaw_rate;

  robot_odometry_.xyz_pos.x += x_dot*dt;
  robot_odometry_.xyz_pos.y += y_dot*dt;
  robot_odometry_.euler_angs.psi += yaw_dot*dt;
}

Affine3d SpotMicroMotionCmd::getOdometryTransform() {
  Translation3d translation(robot_odometry_.xyz_pos.x, robot_odometry_.xyz_pos.y, 0.0);
  AngleAxisd rotation(robot_odometry_.euler_angs.psi, Vector3d::UnitZ());

  return (translation * rotation);
}
