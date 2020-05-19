#include "spot_micro_stand.h"

namespace smfsm {

SpotMicroStandState::SpotMicroStandState() {
  // Construcotr, doesn't need to do anything, for now...
  std::cout << "SpotMicroStandState Ctor" << std::endl;
}

SpotMicroStandState::~SpotMicroStandState() {
  std::cout << "SpotMicroStandState Dtor" << std::endl;
}

void SpotMicroStandState::handleInputCommands(SpotMicroFsm& fsm, Command& cmd) {
  std::cout << "In Spot Micro Stand State" << std::endl;

}

}
