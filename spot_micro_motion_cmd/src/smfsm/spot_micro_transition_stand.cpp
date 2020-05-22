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


void SpotMicroTransitionStandState::init(SpotMicroMotionCmd* smmc, 
                    const smk::BodyState& body_state,
                    const SpotMicroNodeConfig& smnc,
                    const Command& cmd) {
  // Set initial state and end state
  
  // Get starting body state
  start_body_state_ = body_state;
 
  // Create end state 
  // Create end state feet positions, a default foot stance
  float len = smnc.smc.body_length; // body length
  float width = smnc.smc.body_width; // body width
  float l1 = smnc.smc.hip_link_length; // liength of the hip link
  // TODO: add stance offset parameters

  end_body_state_.leg_feet_pos.right_back  = {.x = -len/2, .y = 0.0f, .z =  width/2 + l1};
  end_body_state_.leg_feet_pos.right_front = {.x =  len/2, .y = 0.0f, .z =  width/2 + l1};
  end_body_state_.leg_feet_pos.left_front  = {.x =  len/2, .y = 0.0f, .z = -width/2 - l1};
  end_body_state_.leg_feet_pos.left_back   = {.x = -len/2, .y = 0.0f, .z = -width/2 - l1};

  // End body state position and angles
  end_body_state_.euler_angs.phi = 0.0f;
  end_body_state_.euler_angs.theta = 0.0f;
  end_body_state_.euler_angs.psi = 0.0f;

  end_body_state_.xyz_pos.x = 0.0f;
  end_body_state_.xyz_pos.y = smnc.default_stand_height;
  end_body_state_.xyz_pos.z = 0.0f;
}


void SpotMicroTransitionStandState::handleInputCommands(SpotMicroMotionCmd* smmc,
                                   const smk::BodyState& body_state,
                                   const SpotMicroNodeConfig& smnc,
                                   const Command& cmd) {
  std::cout << "In Spot Micro Idle State" << std::endl;
  
  // Check if desired end state reached, if so, change to stand state
  if (false) {
    //changeState(smmc, std::make_unique<SpotMicroStandState>());
  
  } else {
    // Otherwise, otherwise, set transitory body/feet position/orientation, and
    // set and send servo proportional command
    smmc->publishZeroServoAbsoluteCommand();
  }

}


