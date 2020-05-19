#include "spot_micro_stand.h"
#include "spot_micro_idle.h"


SpotMicroStandState::SpotMicroStandState() {
  // Construcotr, doesn't need to do anything, for now...
  std::cout << "SpotMicroStandState Ctor" << std::endl;
}

SpotMicroStandState::~SpotMicroStandState() {
  std::cout << "SpotMicroStandState Dtor" << std::endl;
}

void SpotMicroStandState::handleInputCommands(SpotMicroMotionCmd& smmc, const Command& cmd) {
  std::cout << "In Spot Micro Stand State" << std::endl;

  if (cmd.getIdleCmd() == true) {
    // Call parent class's change state method
    changeState(smmc, std::make_unique<SpotMicroIdleState>());
  }
}

