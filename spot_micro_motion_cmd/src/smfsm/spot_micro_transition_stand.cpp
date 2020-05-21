#include "spot_micro_transition_stand.h"
#include "spot_micro_idle.h"
#include "spot_micro_stand.h"
#include "spot_micro_motion_cmd.h"

SpotMicroTransitionStandState::SpotMicroTransitionStandState() {
  // Construcotr, doesn't need to do anything, for now...
  std::cout << "SpotMicroTransitionStandState Ctor" << std::endl;

  // Need to create 6 first order filter object, ignore transiting feet poition
  // for now 
  rlfof = RateLmtdFirstOrderFilter(0.02, 1.0f, 0.0f, 1000.0f);

}

SpotMicroTransitionStandState::~SpotMicroTransitionStandState() {
  std::cout << "SpotMicroTransitionStandState Dtor" << std::endl;
}


void SpotMicroTransitionStandState::init(SpotMicroMotionCmd& smmc, const Command& cmd) {
  // Set initial state and end state
  
  // Get starting feet positions
  start_feet_pos_ = smmc.sm_.getLegsFootPos();
  
  // Create end state feet positions, a default foot stance
  float len = smmc.smnc_.smc.body_length; // body length
  float width = smmc.smnc_.smc.body_width; // body width
  float l1 = smmc.smnc_.smc.hip_link_length; // liength of the hip link
  // TODO: add stance offset parameters

  end_feet_pos_.right_back  = {.x = -len/2, .y = 0.0f, .z =  width/2 + l1};
  end_feet_pos_.right_front = {.x =  len/2, .y = 0.0f, .z =  width/2 + l1};
  end_feet_pos_.left_front  = {.x =  len/2, .y = 0.0f, .z = -width/2 - l1};
  end_feet_pos_.left_back   = {.x = -len/2, .y = 0.0f, .z = -width/2 - l1};

  // Get starting body state
  start_body_state_ = smmc.sm_.getBodyState();

  // End body state is x=0, z=0, y= stand height, phi, theta, psi = 0
  end_body_state_.euler_angs.phi = 0.0f;
  end_body_state_.euler_angs.theta = 0.0f;
  end_body_state_.euler_angs.psi = 0.0f;

  end_body_state_.xyz_pos.x = 0.0f;
  end_body_state_.xyz_pos.y = smmc.smnc_.default_stand_height;
  end_body_state_.xyz_pos.z = 0.0f;
}


void SpotMicroTransitionStandState::handleInputCommands(SpotMicroMotionCmd& smmc, const Command& cmd) {
  std::cout << "In Spot Micro Idle State" << std::endl;
  
  // Check if desired end state reached, if so, change to stand state
  if (cmd.getStandCmd() == true) {
    changeState(smmc, std::make_unique<SpotMicroStandState>());
  
  } else {
    // Otherwise, otherwise, set transitory body/feet position/orientation, and
    // set and send servo proportional command
    smmc.publishZeroServoAbsoluteCommand();
  }

}


