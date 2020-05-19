#pragma once

#include "spot_micro_state.h"
#include "command.h"
#include <iostream>

namespace smfsm {

class SpotMicroStandState : public SpotMicroState {
 public:
  SpotMicroStandState(); // Constructor
  ~SpotMicroStandState(); // Destructor
  virtual void handleInputCommands(SpotMicroFsm& fsm, Command& cmd);

};

}