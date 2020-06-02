#pragma once

#include "spot_micro_motion_cmd.h"
#include "spot_micro_state.h"
#include "command.h"

struct ContactFeet {
  bool right_back_in_swing;
  bool right_front_in_swing;
  bool left_front_in_swing;
  bool left_back_in_swing;\
};

class SpotMicroWalkState : public SpotMicroState {
 public:
  SpotMicroWalkState(); // Constructor
  ~SpotMicroWalkState(); // Destructor

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
    return "Walk";
  }

 private:

  SpotMicroNodeConfig smnc_;
  smk::BodyState cmd_state_;

  int ticks_;
  int phase_index_;
  int subphase_ticks_;
  ContactFeet contact_feet_states_;

  // Updates the integer phase index cooresponding to the current phase the
  // robot gait should be in. Updates the subphase ticks, the ticks since the
  // start of the current phase. And updates the contact feet states
  // representing which feet are in swing and stance phases
  void updatePhaseData();


  // Steps the gait controller one timestep, sets the feet command state, and
  // possibly other command states (such as xyz position, euler angles) if
  // necessary
  smk::LegsFootPos stepGait(const smk::BodyState& body_state,
                            const Command& cmd,
                            const SpotMicroNodeConfig& smnc,
                            const smk::LegsFootPos& default_stance_feet_pos);

  // Returns new foot position incremented by stance controller
  smk::Point stanceController(const smk::Point& foot_pos,
                              const Command& cmd,
                              const SpotMicroNodeConfig& smnc);

  // Returns new foot position incremented by swing leg controller
  smk::Point swingLegController(const smk::Point& foot_pos,
                                const Command& cmd,
                                const SpotMicroNodeConfig& smnc,
                                float swing_proportion,
                                const smk::Point& default_stance_foot_pos);

  // Steps the body shift controller that shifts the body xyz position 
  // to maintain balance during the gait cycle
  smk::Point stepBodyShift(const smk::BodyState& body_state,
                           const Command& cmd,
                           const SpotMicroNodeConfig& smnc);
};

  
