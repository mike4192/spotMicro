#include "spot_micro_kinematics/spot_micro_kinematics.h"

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


void SpotMicroState::handleInputCommands(SpotMicroMotionCmd* smmc,
                                         const smk::BodyState& body_state,
                                         const SpotMicroNodeConfig& smnc,
                                         const Command& cmd) {}

void SpotMicroState::init(SpotMicroMotionCmd* smmc, 
                          const smk::BodyState& body_state,
                          const SpotMicroNodeConfig& smnc,
                          const Command& cmd) {}

void SpotMicroState::changeState(SpotMicroMotionCmd* smmc, std::unique_ptr<SpotMicroState> sms) {
  smmc->changeState(std::move(sms));
}

