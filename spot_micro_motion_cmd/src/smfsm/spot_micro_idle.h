#pragma once

#include "spot_micro_state.h"
//#include "spot_micro_fsm.h"
#include "command.h"
#include <iostream>

namespace smfsm {

class SpotMicroIdleState : public SpotMicroState {
 public:
  SpotMicroIdleState(); // Constructor
  ~SpotMicroIdleState(); // Destructor
  virtual void handleInputCommands(SpotMicroFsm& fsm, Command& cmd);

};

}
