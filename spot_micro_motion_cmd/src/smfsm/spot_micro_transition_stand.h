#pragma once

#include <iostream>

#include "spot_micro_state.h"
#include "command.h"
#include "rate_limited_first_order_filter.h"

#include "spot_micro_kinematics/spot_micro_kinematics.h"

// Struct holding a filter for three things relative to axes
struct XyzFilters {
  RateLmtdFirstOrderFilter x;
  RateLmtdFirstOrderFilter y;
  RateLmtdFirstOrderFilter z;
};

// Convenience structure for hold filters for all possible transitory states
struct BodyStateFilters {
  XyzFilters leg_right_back;
  XyzFilters leg_right_front;
  XyzFilters leg_left_front;
  XyzFilters leg_left_back;
  XyzFilters body_pos;
  XyzFilters body_angs;
};

class SpotMicroTransitionStandState : public SpotMicroState {
 public:
  SpotMicroTransitionStandState(); // Constructor
  ~SpotMicroTransitionStandState(); // Destructor

  virtual void handleInputCommands(const smk::BodyState& body_state,
                                   const SpotMicroNodeConfig& smnc,
                                   const Command& cmd,
                                   SpotMicroMotionCmd* smmc,
                                   smk::BodyState* body_state_cmd);

  virtual void init(SpotMicroMotionCmd* smmc, 
                    const smk::BodyState& body_state,
                    const SpotMicroNodeConfig& smnc,
                    const Command& cmd);

 private:
  smk::BodyState start_body_state_;
  smk::BodyState end_body_state_;
  RateLmtdFirstOrderFilter rlfof; 
  BodyStateFilters body_state_filters_;
};

