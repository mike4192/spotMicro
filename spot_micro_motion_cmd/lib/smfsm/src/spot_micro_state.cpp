#include "spot_micro_state.h"
#include "spot_micro_fsm.h"


namespace smfsm {

// Constructor
SpotMicroState::SpotMicroState() {
  // Nothing needs to be done
}

void SpotMicroState::handleInputCommands(SpotMicroFsm* fsm, Command& cmd) {}
void SpotMicroState::update(SpotMicroFsm* fsm) {}

void SpotMicroState::changeState(SpotMicroFsm* fsm, SpotMicroState* sms) {
  fsm->changeState(sms);
}

}
