#pragma once

#include "spot_micro_state.h"
#include "command.h"
#include <iostream>


class SpotMicroStandState : public SpotMicroState {
 public:
  SpotMicroStandState(); // Constructor
  ~SpotMicroStandState(); // Destructor
  virtual void handleInputCommands(SpotMicroMotionCmd* smmc, 
                                   const smk::BodyState& body_state,
                                   const SpotMicroNodeConfig& smnc,
                                   const Command& cmd);

};

