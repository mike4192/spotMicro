#pragma once

#include "command.h"
//#include "spot_micro_fsm.h"



namespace smfsm {

class SpotMicroFsm;

class SpotMicroState {
 public:

  
  SpotMicroState(); // Constructor
  virtual ~SpotMicroState(); // Destructor

  virtual void handleInputCommands(SpotMicroFsm& fsm, Command& cmd);

 protected:
  void changeState(SpotMicroFsm& fsm, std::unique_ptr<SpotMicroState> sms);
};

}
