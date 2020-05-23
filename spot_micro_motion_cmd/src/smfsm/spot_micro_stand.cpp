#include "spot_micro_stand.h"

#include "spot_micro_idle.h"
#include "spot_micro_motion_cmd.h"

SpotMicroStandState::SpotMicroStandState() {
  // Construcotr, doesn't need to do anything, for now...
  std::cout << "SpotMicroStandState Ctor" << std::endl;
}

SpotMicroStandState::~SpotMicroStandState() {
  std::cout << "SpotMicroStandState Dtor" << std::endl;
}

void SpotMicroStandState::handleInputCommands(const smk::BodyState& body_state,
                                   const SpotMicroNodeConfig& smnc,
                                   const Command& cmd,
                                   SpotMicroMotionCmd* smmc, 
                                   smk::BodyState* body_state_cmd) {
  std::cout << "In Spot Micro Stand State" << std::endl;

  

  if (cmd.getIdleCmd() == true) {
    // Call parent class's change state method
    changeState(smmc, std::make_unique<SpotMicroIdleState>());
  } else {
    body_state_cmd->euler_angs = cmd_state_.euler_angs;

    body_state_cmd->xyz_pos = cmd_state_.xyz_pos;

    body_state_cmd->leg_feet_pos = cmd_state_.leg_feet_pos;

    // Set and publish command
    smmc->setServoCommandMessageData();
    smmc->publishServoProportionalCommand();
  }
}



void SpotMicroStandState::init(SpotMicroMotionCmd* smmc,
                               const smk::BodyState& body_state,
                               const SpotMicroNodeConfig& smnc,
                               const Command& cmd) {
  // Set default stance
  float len = smnc.smc.body_length; // body length
  float width = smnc.smc.body_width; // body width
  float l1 = smnc.smc.hip_link_length; // liength of the hip link
  // TODO: Incorporate stance offset parameters
  cmd_state_.leg_feet_pos.right_back  = {.x = -len/2, .y = 0.0f, .z =  width/2 + l1};
  cmd_state_.leg_feet_pos.right_front = {.x =  len/2, .y = 0.0f, .z =  width/2 + l1};
  cmd_state_.leg_feet_pos.left_front  = {.x =  len/2, .y = 0.0f, .z = -width/2 - l1};
  cmd_state_.leg_feet_pos.left_back   = {.x = -len/2, .y = 0.0f, .z = -width/2 - l1};

  // End body state position and angles
  cmd_state_.euler_angs.phi = 0.0f;
  cmd_state_.euler_angs.theta = 0.0f;
  cmd_state_.euler_angs.psi = 0.0f;

  cmd_state_.xyz_pos.x = 0.0f;
  cmd_state_.xyz_pos.y = smnc.default_stand_height;
  cmd_state_.xyz_pos.z = 0.0f;
}

