#pragma once
#include "spot_micro_kinematics/spot_micro_kinematics.h"

#include "command.h"

// Forward declaration of classes and structs. Can't include
// spot_micro_motion_cmd.h here otherwise get circular dependence compile errors
class SpotMicroMotionCmd;
struct SpotMicroNodeConfig;

class SpotMicroState {
 public:

  // Constructor
  SpotMicroState(); 

  // Destructor
  virtual ~SpotMicroState();

  // virtual method to handle input commands, may change SpotMicroMotionCmd object
  // passed by reference.
  virtual void handleInputCommands(SpotMicroMotionCmd* smmc, 
                                   const smk::BodyState& body_state,
                                   const SpotMicroNodeConfig& smnc,
                                   const Command& cmd);

  virtual void init(SpotMicroMotionCmd* smmc, 
                    const smk::BodyState& body_state,
                    const SpotMicroNodeConfig& smnc,
                    const Command& cmd);
 protected:

  // Calls SpotMicroMotionCmd's method to change the currently active state
  void changeState(SpotMicroMotionCmd* smmc, std::unique_ptr<SpotMicroState> sms);
};

