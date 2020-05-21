#include "spot_micro_transition_stand.h"
#include "spot_micro_idle.h"
#include "spot_micro_stand.h"
#include "spot_micro_motion_cmd.h"

SpotMicroTransitionStandState::SpotMicroTransitionStandState() {
  // Construcotr, doesn't need to do anything, for now...
  std::cout << "SpotMicroTransitionStandState Ctor" << std::endl;

}

SpotMicroTransitionStandState::~SpotMicroTransitionStandState() {
  std::cout << "SpotMicroTransitionStandState Dtor" << std::endl;
}

//TODO Rename transition to stand
void SpotMicroTransitionStandState::init(SpotMicroMotionCmd& smmc, const Command& cmd) {
  // Set initial state and end state
  
  float len = smmc.smnc_.smc.body_length; // body length
  float width = smmc.smnc_.smc.body_width; // body width
  float l1 = smmc.smnc_.smc.hip_link_length; // liength of the hip link
  // TODO: add stance offset parameters
  end_feet_pos_.right_back  = {.x = -len/2, .y = 0.0f, .z =  width/2 + l1};
  end_feet_pos_.right_front = {.x =  len/2, .y = 0.0f, .z =  width/2 + l1};
  end_feet_pos_.left_front  = {.x =  len/2, .y = 0.0f, .z = -width/2 - l1};
  end_feet_pos_.left_back   = {.x = -len/2, .y = 0.0f, .z = -width/2 - l1};

  start_feet_pos_ = smmc.sm_.getLegsFootPos();

  // Get starting body positions from smk method TODO
  
  // End body position is x=0, z=0, y= stand height
  
  // Get starting body angles from smk method TODO
  
  // End body angles are x, y, z 0

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


