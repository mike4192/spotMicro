#pragma once
#include "spot_micro_kinematics/spot_micro_kinematics.h"

#include "rate_limited_first_order_filter.h"
#include "spot_micro_kinematics/spot_micro_kinematics.h"
#include "command.h"

// Forward declaration of classes and structs. Can't include
// spot_micro_motion_cmd.h here otherwise get circular dependence compile errors
class SpotMicroMotionCmd;
struct SpotMicroNodeConfig;


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

class SpotMicroState {
 public:

  // Constructor
  SpotMicroState(); 

  // Destructor
  virtual ~SpotMicroState();

  // virtual method to handle input commands, may change SpotMicroMotionCmd object
  // passed by reference.
  virtual void handleInputCommands(const smk::BodyState& body_state,
                                   const SpotMicroNodeConfig& smnc,
                                   const Command& cmd,
                                   SpotMicroMotionCmd* smmc, 
                                   smk::BodyState* body_state_cmd) {}

  virtual void init(const smk::BodyState& body_state,
                    const SpotMicroNodeConfig& smnc,
                    const Command& cmd,
                    SpotMicroMotionCmd* smmc) {}

  virtual std::string getCurrentStateName() {
    return "None";
  }

 protected:

  // Calls SpotMicroMotionCmd's method to change the currently active state
  void changeState(SpotMicroMotionCmd* smmc, std::unique_ptr<SpotMicroState> sms);

  // Initializes set of filters controlling body state values to values
  // contained in body_state
  virtual void initBodyStateFilters(float dt, float tau, float rl, float rl_ang,
                                    const smk::BodyState& body_state,
                                    BodyStateFilters* body_state_filters);

  // Sets body state filter commands to the values contained in body_state
  virtual void setBodyStateFilterCommands(const smk::BodyState& body_state,
                                          BodyStateFilters* body_state_filters);

  // Calls run timestep method for all body state filters
  virtual void runFilters(BodyStateFilters* body_state_filters);

  // Assigns current filter value to body state
  virtual void assignFilterValuesToBodyState(
      const BodyStateFilters& body_state_filters,
      smk::BodyState* body_state);

  // Checks equality of body state structs to an absolute tolerance. Returns
  // true if all absolute value differences of body state values are within
  // tolernace
  virtual bool checkBodyStateEquality(const smk::BodyState& body_state1,
                                      const smk::BodyState& body_state2,
                                      float tol);

};

