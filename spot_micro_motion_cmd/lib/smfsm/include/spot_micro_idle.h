#pragma once

#include "spot_micro_state.h"
#include "spot_micro_fsm.h"
#include "command.h"
#include <iostream>

namespace smfsm {

class SpotMicroIdleState : public SpotMicroState {
 public:
  // Constructor
  SpotMicroIdleState();

  virtual void handleInputCommands(SpotMicroFsm* fsm, Command& cmd);
  virtual void update(SpotMicroFsm* fsm);

};

}
