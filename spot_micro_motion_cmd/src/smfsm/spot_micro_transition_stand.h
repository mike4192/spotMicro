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

  virtual void handleInputCommands(SpotMicroMotionCmd* smmc,
                                   const smk::BodyState& body_state,
                                   const SpotMicroNodeConfig& smnc,
                                   const Command& cmd);

  virtual void init(SpotMicroMotionCmd* smmc, 
                    const smk::BodyState& body_state,
                    const SpotMicroNodeConfig& smnc,
                    const Command& cmd);

 private:
  smk::BodyState start_body_state_;
  smk::BodyState end_body_state_;
  RateLmtdFirstOrderFilter rlfof; 
};

