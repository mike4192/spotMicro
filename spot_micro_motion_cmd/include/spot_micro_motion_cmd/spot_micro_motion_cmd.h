#pragma once
#ifndef SPOT_MICRO_MOTION_CMD
#define SPOT_MICRO_MOTION_CMD

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <i2cpwm_board/msg/servo.hpp>
#include <i2cpwm_board/msg/servo_array.hpp>
#include <i2cpwm_board/srv/servos_config.hpp>

#include "command.h"
#include "spot_micro_kinematics/spot_micro_kinematics.h"
#include "spot_micro_state.h"

struct SpotMicroNodeConfig {
  smk::SpotMicroConfig smc;
  float default_stand_height;
  float stand_front_x_offset;
  float stand_back_x_offset;
  float lie_down_height;
  float lie_down_feet_x_offset;
  int num_servos;
  float servo_max_angle_deg;
  std::map<std::string, std::map<std::string, float>> servo_config;
  float dt;
  float transit_tau;
  float transit_rl;
  float transit_angle_rl;
  bool debug_mode;
  bool plot_mode;
  float max_fwd_velocity;
  float max_side_velocity;
  float max_yaw_rate;
  float z_clearance;
  float alpha;
  float beta;
  int num_phases;
  std::vector<int64_t> rb_contact_phases;
  std::vector<int64_t> rf_contact_phases;
  std::vector<int64_t> lf_contact_phases;
  std::vector<int64_t> lb_contact_phases;
  float overlap_time;
  float swing_time;
  int overlap_ticks;
  int swing_ticks;
  int stance_ticks;
  std::vector<int64_t> phase_ticks;
  int phase_length;
  float foot_height_time_constant;
  std::vector<int64_t> body_shift_phases;
  float fwd_body_balance_shift;
  float side_body_balance_shift;
  float back_body_balance_shift;
  bool publish_odom;
  float lidar_x_pos;
  float lidar_y_pos;
  float lidar_z_pos;
  float lidar_yaw_angle;
};

class SpotMicroMotionCmd : public rclcpp::Node
{
 public:
  SpotMicroMotionCmd();
  ~SpotMicroMotionCmd();

  void runOnce();
  bool publishServoConfiguration();
  void setServoCommandMessageData();
  void publishServoProportionalCommand();
  void publishZeroServoAbsoluteCommand();
  SpotMicroNodeConfig getNodeConfig();
  smk::LegsFootPos getNeutralStance();
  smk::LegsFootPos getLieDownStance();
  void commandIdle();
  std::string getCurrentStateName();

 private:
  friend class SpotMicroState;

  std::unique_ptr<SpotMicroState> state_;
  Command cmd_;
  smk::SpotMicroKinematics sm_;
  SpotMicroNodeConfig smnc_;
  smk::BodyState body_state_cmd_;
  smk::BodyState robot_odometry_;
  std::map<std::string, float> servo_cmds_rad_ = {
    {"RF_3", 0.0f}, {"RF_2", 0.0f}, {"RF_1", 0.0f},
    {"RB_3", 0.0f}, {"RB_2", 0.0f}, {"RB_1", 0.0f},
    {"LB_3", 0.0f}, {"LB_2", 0.0f}, {"LB_1", 0.0f},
    {"LF_3", 0.0f}, {"LF_2", 0.0f}, {"LF_1", 0.0f}
  };

  void readInConfigParameters();

  i2cpwm_board::msg::ServoArray servo_array_;
  i2cpwm_board::msg::ServoArray servo_array_absolute_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stand_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr idle_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr walk_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_cmd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr body_angle_cmd_sub_;
  rclcpp::Publisher<i2cpwm_board::msg::ServoArray>::SharedPtr servos_absolute_pub_;
  rclcpp::Publisher<i2cpwm_board::msg::ServoArray>::SharedPtr servos_proportional_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr body_state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr lcd_vel_cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr lcd_angle_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr lcd_state_pub_;
  rclcpp::Client<i2cpwm_board::srv::ServosConfig>::SharedPtr servos_config_client_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> transform_br_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_transform_br_;

  std_msgs::msg::Float32MultiArray body_state_msg_;
  std_msgs::msg::String lcd_state_string_msg_;
  geometry_msgs::msg::Twist lcd_vel_cmd_msg_;
  geometry_msgs::msg::Vector3 lcd_angle_cmd_msg_;

  void standCommandCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void idleCommandCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void walkCommandCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void angleCommandCallback(const geometry_msgs::msg::Vector3::SharedPtr msg);
  void velCommandCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void resetEventCommands();
  void handleInputCommands();
  void changeState(std::unique_ptr<SpotMicroState> sms);
  void publishBodyState();
  void publishLcdMonitorData();
  void publishStaticTransforms();
  void publishDynamicTransforms();
  void integrateOdometry();
  Eigen::Affine3d getOdometryTransform();
};
#endif
