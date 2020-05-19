#pragma once

#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"


class Command {
 public:
  
  float x_vel_cmd_mps_;
  float y_vel_cmd_mps_;
  float yaw_rate_cmd_rps_;
  
  bool idle_cmd_;
  bool rest_cmd_;
  bool walk_cmd_;
  bool stand_cmd_;
 
  // Constructor
  Command()
      : x_vel_cmd_mps_(0.0)
      , y_vel_cmd_mps_(0.0)
      , yaw_rate_cmd_rps_(0.0)
      , idle_cmd_(false)
      , rest_cmd_(false)
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

  void resetEventCmds() {
    // Reset all event commands to false
    idle_cmd_ = false;
    rest_cmd_ = false;
    walk_cmd_ = false;
    stand_cmd_ = false;
  }
};
