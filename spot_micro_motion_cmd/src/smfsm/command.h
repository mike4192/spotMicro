#pragma once

#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"


namespace smfsm {

class Command {
 public:
  
  float x_vel_cmd_mps;
  float y_vel_cmd_mps;
  float yaw_rate_cmd_rps;
  
  bool idle_cmd;
  bool rest_cmd;
  bool walk_cmd;
  bool stand_cmd;
 
  // Constructor
  Command()
      : x_vel_cmd_mps(0.0)
      , y_vel_cmd_mps(0.0)
      , yaw_rate_cmd_rps(0.0)
      , idle_cmd(false)
      , rest_cmd(false)
      , walk_cmd(false)
      , stand_cmd(false)
      { }

  bool getStandCmd()
  {
      // stand cmd status getter. If cmd state is true, sets it false and return true, 
      // otherwise returns false
      if (stand_cmd == true)
      {
          stand_cmd = false;
          return true;
      }
      else
      {
          return false;
      }
  }
};
}
