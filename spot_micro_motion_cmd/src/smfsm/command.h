#pragma once

#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"


class Command {
 public:
  
  float x_vel_cmd_mps_;
  float y_vel_cmd_mps_;
  float yaw_rate_cmd_rps_;
  float phi_cmd_;
  float theta_cmd_;
  float psi_cmd_;

  bool idle_cmd_;
  bool walk_cmd_;
  bool stand_cmd_;
 
  // Constructor
  Command()
      : x_vel_cmd_mps_(0.0)
      , y_vel_cmd_mps_(0.0)
      , yaw_rate_cmd_rps_(0.0)
      , phi_cmd_(0.0f)
      , theta_cmd_(0.0f)
      , psi_cmd_(0.0f)
      , idle_cmd_(false)
      , walk_cmd_(false)
      , stand_cmd_(false)
      { }

  bool getStandCmd() const {
      // stand cmd status getter.
      return stand_cmd_;
  }

  bool getIdleCmd() const {
    return idle_cmd_;
  }

  bool getWalkCmd() const {
     return walk_cmd_;
  } 
 
  float getXSpeedCmd() const {
    return x_vel_cmd_mps_;
  }
 
  float getYSpeedCmd() const {
    return y_vel_cmd_mps_;
  }

  float getYawRateCmd() const {
    return yaw_rate_cmd_rps_;
  }

  float getPhiCmd() const {
    return phi_cmd_;
  }

  float getThetaCmd() const {
    return theta_cmd_;
  }

  float getPsiCmd() const {
    return psi_cmd_;
  }

  void resetAllCommands() {
    x_vel_cmd_mps_ = 0;
    y_vel_cmd_mps_ = 0;
    yaw_rate_cmd_rps_ = 0;
    phi_cmd_ = 0;
    theta_cmd_ = 0;
    psi_cmd_ = 0;
  }
 
  void resetEventCmds() {
    // Reset all event commands to false
    idle_cmd_ = false;
    walk_cmd_ = false;
    stand_cmd_ = false;
  }
};
