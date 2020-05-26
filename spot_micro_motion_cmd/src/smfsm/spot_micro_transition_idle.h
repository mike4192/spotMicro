#pragma once

#include "spot_micro_state.h"
#include "command.h"
#include "rate_limited_first_order_filter.h"

#include "spot_micro_kinematics/spot_micro_kinematics.h"

class SpotMicroTransitionIdleState : public SpotMicroState {
 public:
  SpotMicroTransitionIdleState(); // Constructor
  ~SpotMicroTransitionIdleState(); // Destructor

  virtual void handleInputCommands(const smk::BodyState& body_state,
                                   const SpotMicroNodeConfig& smnc,
                                   const Command& cmd,
                                   SpotMicroMotionCmd* smmc,
                                   smk::BodyState* body_state_cmd);

  virtual void init(const smk::BodyState& body_state,
                    const SpotMicroNodeConfig& smnc,
                    const Command& cmd,
                    SpotMicroMotionCmd* smmc); 

  // Returns current state name as a string
  virtual std::string getCurrentStateName() {
    return "Transit Idle";
  }
 private:
  smk::BodyState start_body_state_;
  smk::BodyState end_body_state_;
  RateLmtdFirstOrderFilter rlfof; 
  BodyStateFilters body_state_filters_;
};


