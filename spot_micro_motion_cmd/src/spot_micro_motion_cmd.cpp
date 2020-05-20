
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"

#include "spot_micro_motion_cmd.h"
#include "spot_micro_kinematics/spot_micro_kinematics.h"
#include "i2cpwm_board/Servo.h"
#include "i2cpwm_board/ServoArray.h"
#include "i2cpwm_board/ServoConfig.h"
#include "i2cpwm_board/ServoConfigArray.h"
#include "i2cpwm_board/ServosConfig.h"



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
  pnh_ = pnh;

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
  servos_absolute_pub_ = nh.advertise<i2cpwm_board::ServoArray>("servos_absolute", 1);

  // Servos proportional publisher
  servos_proportional_pub_ = nh.advertise<i2cpwm_board::ServoArray>("servos_proportional",1);  
  
  // Servos configuration publisher
  servos_config_client_ = nh.serviceClient<i2cpwm_board::ServosConfig>("config_servos");
  
}

// Destructor method
SpotMicroMotionCmd::~SpotMicroMotionCmd() {

  std::cout<<"from Distructor \n";
  // Free up the memory assigned from heap
}


void SpotMicroMotionCmd::readInConfigParameters() {

  // Read in and save parameters 
  // Use private node handle for getting params so just the relative
  // parameter name can be used (as opposed to the global name, e.g.:
  // /spot_micro_motion_cmd/param1
  pnh_.getParam("hip_link_length", smnc_.smc.hip_link_length);
  pnh_.getParam("upper_leg_link_length", smnc_.smc.upper_leg_link_length);
  pnh_.getParam("lower_leg_link_length", smnc_.smc.lower_leg_link_length);
  pnh_.getParam("body_width", smnc_.smc.body_width);
  pnh_.getParam("body_length", smnc_.smc.body_length);
  pnh_.getParam("default_stand_height", smnc_.default_stand_height);
  pnh_.getParam("lie_down_height", smnc_.lie_down_height);
  pnh_.getParam("num_servos", smnc_.num_servos);
  pnh_.getParam("servo_max_angle_deg", smnc_.servo_max_angle_deg);

  std::map<std::string, float> temp_map;
  // Iterate over servo names, as defined in the map servo_cmds_rad, to populate
  // the servo config map in smnc_
  for(std::map<std::string, float>::iterator 
      iter = servo_cmds_rad_.begin();
      iter != servo_cmds_rad_.end();
      ++iter) {

    std::string servo_name = iter->first; // Get key, string of the servo name
    
    pnh_.getParam(servo_name, temp_map); // Read the parameter. Parameter name must match servo name
    smnc_.servo_config[servo_name] = temp_map; // Assing in servo config to map in the struct
  }
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



