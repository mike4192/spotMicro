#include "spot_micro_state.h"
#include "spot_micro_motion_cmd.h"



// Constructor
SpotMicroState::SpotMicroState() {
  // Nothing needs to be done
  std::cout << "SpotMicroState Ctor" << std::endl;
}


SpotMicroState::~SpotMicroState() {
  std::cout << "SpotMicroState Dtor" << std::endl;
}


void SpotMicroState::handleInputCommands(SpotMicroMotionCmd& smmc, const Command& cmd) {}

void SpotMicroState::init(SpotMicroMotionCmd& smmc, const Command& cmd) {}

void SpotMicroState::changeState(SpotMicroMotionCmd& smmc, std::unique_ptr<SpotMicroState> sms) {
  //fsm.changeState(std::move(sms));
  smmc.changeState(std::move(sms));
}

