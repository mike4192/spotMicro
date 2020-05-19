#include "spot_micro_idle.h"
#include "spot_micro_stand.h"

namespace smfsm {

SpotMicroIdleState::SpotMicroIdleState() {
  // Construcotr, doesn't need to do anything, for now...
  std::cout << "SpotMicroIdleState Ctor" << std::endl;
}

SpotMicroIdleState::~SpotMicroIdleState() {
  std::cout << "SpotMicroIdleState Dtor" << std::endl;
}

void SpotMicroIdleState::handleInputCommands(SpotMicroFsm& fsm, Command& cmd) {
  std::cout << "In Spot Micro Idle State" << std::endl;
  // Check if stand command issued, if so, transition to stand state

  if (cmd.getStandCmd() == true)
  {
    changeState(fsm, std::make_unique<SpotMicroStandState>());
  }
}

}
