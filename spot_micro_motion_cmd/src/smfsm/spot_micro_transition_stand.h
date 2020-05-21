#pragma once

#include <iostream>

#include "spot_micro_state.h"
#include "command.h"

#include "spot_micro_kinematics/spot_micro_kinematics.h"

class SpotMicroTransitionStandState : public SpotMicroState {
 public:
  SpotMicroTransitionStandState(); // Constructor
  ~SpotMicroTransitionStandState(); // Destructor

  virtual void handleInputCommands(SpotMicroMotionCmd& smmc, const Command& cmd);

  virtual void init(SpotMicroMotionCmd& smmc, const Command& cmd);

 private:
  smk::LegsFootPos start_feet_pos_;
  smk::LegsFootPos end_feet_pos_;

  smk::Point start_body_pos_;
  smk::Point end_body_pos_;

  float start_phi_, start_theta_, start_psi_; 
  float end_phi_, end_theta_, end_psi_; 
  
};

