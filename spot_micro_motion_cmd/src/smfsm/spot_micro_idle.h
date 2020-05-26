#pragma once

#include <iostream>

#include "spot_micro_state.h"
#include "command.h"


class SpotMicroIdleState : public SpotMicroState {
 public:
  SpotMicroIdleState(); // Constructor
  ~SpotMicroIdleState(); // Destructor
  virtual void handleInputCommands(const smk::BodyState& body_state,
                                   const SpotMicroNodeConfig& smnc,
                                   const Command& cmd,
                                   SpotMicroMotionCmd* smmc, 
                                   smk::BodyState* body_state_cmd);

  virtual std::string getCurrentStateName() {
    return "Idle";
  }
};

