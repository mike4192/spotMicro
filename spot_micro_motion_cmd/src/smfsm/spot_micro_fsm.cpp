#include "spot_micro_fsm.h"
#include "spot_micro_state.h"
#include "spot_micro_idle.h"

namespace smfsm {

// Initialize static spot micro idle state member in spot micro state object
// SpotMicroIdleState SpotMicroFsm::idle = SpotMicroIdleState();

// Constructor
SpotMicroFsm::SpotMicroFsm() {
 // _state = new SpotMicroIdleState();
  _state = std::make_unique<SpotMicroIdleState>();
}


void SpotMicroFsm::handleInputCommands(Command& cmd) {
  // Delegate to state object to handle input
  _state->handleInputCommands(*this, cmd);
}

void SpotMicroFsm::changeState(std::unique_ptr<SpotMicroState> sms) {
  _state = std::move(sms);
}

}
