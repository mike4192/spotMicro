#pragma once

#include <iostream>

#include "spot_micro_state.h"
#include "command.h"
#include "rate_limited_first_order_filter.h"

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

  smk::BodyState start_body_state_;
  smk::BodyState end_body_state_;
  RateLmtdFirstOrderFilter rlfof; 
};

