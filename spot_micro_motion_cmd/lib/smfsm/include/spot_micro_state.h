#pragma once

#include "command.h"
#include "spot_micro_fsm.h"



namespace smfsm {

class SpotMicroFsm;

class SpotMicroState {
 public:

  // Constructor
  SpotMicroState();

  virtual void handleInputCommands(SpotMicroFsm* fsm, Command& cmd);
  virtual void update(SpotMicroFsm* fsm);

 protected:
  void changeState(SpotMicroFsm* fsm, SpotMicroState* sms);
};

}
