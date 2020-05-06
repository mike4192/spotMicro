#include "spot_micro_idle.h"
#include "spot_micro_fsm.h"
#include "spot_micro_state.h"

namespace smfsm {

SpotMicroIdleState::SpotMicroIdleState() {
  // Construcotr, doesn't need to do anything, for now...
}

void SpotMicroIdleState::handleInputCommands(SpotMicroFsm* fsm, Command& cmd) {
  std::cout << "In Spot Micro Idle State" << std::endl;
}

void SpotMicroIdleState::update(SpotMicroFsm* fsm) {}

}
