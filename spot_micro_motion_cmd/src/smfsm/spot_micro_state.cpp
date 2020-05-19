#include "spot_micro_state.h"
#include "spot_micro_fsm.h"


namespace smfsm {

// Constructor
SpotMicroState::SpotMicroState() {
  // Nothing needs to be done
  std::cout << "SpotMicroState Ctor" << std::endl;
}

SpotMicroState::~SpotMicroState() {
  std::cout << "SpotMicroState Dtor" << std::endl;
}
void SpotMicroState::handleInputCommands(SpotMicroFsm& fsm, Command& cmd) {}

void SpotMicroState::changeState(SpotMicroFsm& fsm, std::unique_ptr<SpotMicroState> sms) {
  fsm.changeState(std::move(sms));
}

}
