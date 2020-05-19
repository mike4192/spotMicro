#include <memory>

#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"

#include "spot_micro_motion_cmd.h"
#include "spot_micro_kinematics/spot_micro_kinematics.h"
#include "i2cpwm_board/Servo.h"
#include "i2cpwm_board/ServoArray.h"

#include "spot_micro_idle.h"


using namespace smk;


void SpotMicroMotionCmd::standCommandCallback(
    const std_msgs::Bool::ConstPtr& msg) {
  // Toggle event true only on rising edge from external command
  if (msg->data == true) {cmd_.stand_cmd_ = true;}
}


void SpotMicroMotionCmd::idleCommandCallback(
    const std_msgs::Bool::ConstPtr& msg) {
  // Toggle event true only on rising edge from external command
  if (msg->data == true) {cmd_.idle_cmd_ = true;}
}


void SpotMicroMotionCmd::resetEventCommands() {
  // Reset all event commands, setting all command states false if they were true 
  cmd_.resetEventCmds(); 
}


//constructor method
SpotMicroMotionCmd::SpotMicroMotionCmd(ros::NodeHandle &nh, ros::NodeHandle &pnh) {

  nh_ = nh;
  std::cout<<"from Constructor \n";

  // Initialize Command 
  cmd_ = Command();

  // Initialize state to Idle state
  state_ = std::make_unique<SpotMicroIdleState>();

  // Initialize spot micro kinematics object 
  // TODO: Convert to reading values from configuration/parameter server
  smk::SpotMicroConfig smc = {.hip_link_length = 0.055f,
                              .upper_leg_link_length = 0.1075f,
                              .lower_leg_link_length = 0.130f,
                              .body_width = 0.078f,
                              .body_length = 0.186};
    
  smk::SpotMicroKinematics sm = smk::SpotMicroKinematics(0.0f, 0.0f, 0.0f, smc);

  // storing the values in the member variable
  // get the parameters or configurations and store them in member variables
 



  // Initialize publishers and subscribers
    
  // stand cmd event subscriber 
  stand_sub_ = nh.subscribe("/stand_cmd", 1, &SpotMicroMotionCmd::standCommandCallback, this);
    
  // idle cmd event subscriber
  idle_sub_ = nh.subscribe("/idle_cmd", 1, &SpotMicroMotionCmd::idleCommandCallback, this);

  // walk cmd event subscriber
    
  // servos_absolute publisher
  servos_absolute_pub_ = nh.advertise<i2cpwm_board::ServoArray>("servos_absolute", 500);

  // Servos proportional publisher
    
  // Servos configuration publisher
}

// Destructor method
SpotMicroMotionCmd::~SpotMicroMotionCmd() {

  std::cout<<"from Distructor \n";
  // Free up the memory assigned from heap
}

void SpotMicroMotionCmd::runOnce() {

  std::cout<<"from Runonce \n";

  // Call method to handle input commands
  handleInputCommands();

  // Consume all event commands.
  // This resets all event commands if they were true. Doing this enforces a rising edge detection
  resetEventCommands();
}

void SpotMicroMotionCmd::handleInputCommands() {
  // Delegate input handling to state
  state_->handleInputCommands(*this, cmd_);
}


void SpotMicroMotionCmd::changeState(std::unique_ptr<SpotMicroState> sms) {
  // Change the active state
  state_ = std::move(sms);

  // TODO: Call init method of new state?
}



