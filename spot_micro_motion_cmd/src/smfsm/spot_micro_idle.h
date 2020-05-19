#pragma once

#include <iostream>

#include "spot_micro_state.h"
#include "command.h"


class SpotMicroIdleState : public SpotMicroState {
 public:
  SpotMicroIdleState(); // Constructor
  ~SpotMicroIdleState(); // Destructor
  virtual void handleInputCommands(SpotMicroMotionCmd& smmc, const Command& cmd);

};

