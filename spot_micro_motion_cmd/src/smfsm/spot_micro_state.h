#pragma once

#include "command.h"


class SpotMicroMotionCmd;

class SpotMicroState {
 public:

  // Constructor
  SpotMicroState(); 

  // Destructor
  virtual ~SpotMicroState();

  // virtual method to handle input commands, may change SpotMicroMotionCmd object
  // passed by reference.
  virtual void handleInputCommands(SpotMicroMotionCmd& smmc, const Command& cmd);

  virtual void init(SpotMicroMotionCmd& smmc, const Command& cmd);
 protected:

  // Calls SpotMicroMotionCmd's method to change the currently active state
  void changeState(SpotMicroMotionCmd& smmc, std::unique_ptr<SpotMicroState> sms);
};

