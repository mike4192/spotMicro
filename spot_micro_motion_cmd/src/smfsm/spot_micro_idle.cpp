#include "spot_micro_idle.h"
#include "spot_micro_stand.h"
#include "spot_micro_motion_cmd.h"

SpotMicroIdleState::SpotMicroIdleState() {
  // Construcotr, doesn't need to do anything, for now...
  std::cout << "SpotMicroIdleState Ctor" << std::endl;
}

SpotMicroIdleState::~SpotMicroIdleState() {
  std::cout << "SpotMicroIdleState Dtor" << std::endl;
}

void SpotMicroIdleState::handleInputCommands(SpotMicroMotionCmd& smmc, const Command& cmd) {
  std::cout << "In Spot Micro Idle State" << std::endl;
  
  // Check if stand command issued, if so, transition to stand state
  if (cmd.getStandCmd() == true) {
    changeState(smmc, std::make_unique<SpotMicroStandState>());
  
  } else {
    // Otherwise, just command idle servo commands
    smmc.publishZeroServoAbsoluteCommand();
  }

}

