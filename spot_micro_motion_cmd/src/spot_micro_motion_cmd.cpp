
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"

#include "spot_micro_motion_cmd.h"
#include "spot_micro_kinematics/spot_micro_kinematics.h"
#include "i2cpwm_board/Servo.h"
#include "i2cpwm_board/ServoArray.h"
#include "i2cpwm_board/ServoConfig.h"
#include "i2cpwm_board/ServosConfig.h"



#include "spot_micro_idle.h"


using namespace smk;


void SpotMicroMotionCmd::standCommandCallback(
    const std_msgs::Bool::ConstPtr& msg) {
  if (msg->data == true) {cmd_.stand_cmd_ = true;}
}


void SpotMicroMotionCmd::idleCommandCallback(
    const std_msgs::Bool::ConstPtr& msg) {
  if (msg->data == true) {cmd_.idle_cmd_ = true;}
}


void SpotMicroMotionCmd::walkCommandCallback(
    const std_msgs::Bool::ConstPtr& msg) {
  if (msg->data == true) {cmd_.walk_cmd_ = true;}
}


void SpotMicroMotionCmd::resetEventCommands() {
  // Reset all event commands, setting all command states false if they were true 
  cmd_.resetEventCmds(); 
}




// Constructor
SpotMicroMotionCmd::SpotMicroMotionCmd(ros::NodeHandle &nh, ros::NodeHandle &pnh) {

  nh_ = nh;
  pnh_ = pnh;

  std::cout<<"from Constructor \n";

  // Initialize Command 
  cmd_ = Command();

  // Initialize state to Idle state
  state_ = std::make_unique<SpotMicroIdleState>();

  // Read in config parameters into smnc_
  readInConfigParameters();

  // Initialize spot micro kinematics object of this class
  sm_ = smk::SpotMicroKinematics(0.0f, 0.0f, 0.0f, smnc_.smc);

  // Set an initial body height and stance cmd for idle mode
  float len   = smnc_.smc.body_length;
  float width = smnc_.smc.body_width;
  float l1    = smnc_.smc.hip_link_length;

  body_state_cmd_.euler_angs = {.phi = 0.0f, .theta = 0.0f, .psi = 0.0f};
  body_state_cmd_.xyz_pos = {.x = 0.0f, .y = smnc_.lie_down_height, .z = 0.0f};
  body_state_cmd_.leg_feet_pos.right_back  = {.x = -len/2, .y = 0.0f, .z =  width/2 + l1};
  body_state_cmd_.leg_feet_pos.right_front = {.x =  len/2, .y = 0.0f, .z =  width/2 + l1};
  body_state_cmd_.leg_feet_pos.left_front  = {.x =  len/2, .y = 0.0f, .z = -width/2 - l1};
  body_state_cmd_.leg_feet_pos.left_back   = {.x = -len/2, .y = 0.0f, .z = -width/2 - l1};

  // Set the spot micro kinematics object to this initial command
  sm_.setBodyState(body_state_cmd_);


  // Initialize servo array message with 12 servo objects
  for (int i = 1; i <= smnc_.num_servos; i++) {
    i2cpwm_board::Servo temp_servo;
    temp_servo.servo = i;
    temp_servo.value = 0;
    servo_array_.servos.push_back(temp_servo);
  }

  // Initialize servo array absolute message with 12 servo object with a value of
  // zero, just copy servo_array_msg since it's already correct 
  servo_array_absolute_.servos = servo_array_.servos;

  // Initialize publishers and subscribers
  // stand cmd event subscriber 
  stand_sub_ = nh.subscribe("/stand_cmd", 1, &SpotMicroMotionCmd::standCommandCallback, this);
    
  // idle cmd event subscriber
  idle_sub_ = nh.subscribe("/idle_cmd", 1, &SpotMicroMotionCmd::idleCommandCallback, this);

  // walk cmd event subscriber
  walk_sub_ = nh.subscribe("/walk_cmd", 1, &SpotMicroMotionCmd::walkCommandCallback, this);
 
  // speed command subscriber
  

  // position command subscriber
  
  // body rate command subscriber
  
  // body angle command subscriber
  

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

  // Temporary map for populating map in smnc_
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


bool SpotMicroMotionCmd::publishServoConfiguration() {  
  // Create a temporary servo config
  i2cpwm_board::ServoConfig temp_servo_config;
  i2cpwm_board::ServosConfig temp_servo_config_array;

  // Loop through servo configuration dictionary in smnc_, append servo to
  for (std::map<std::string, std::map<std::string, float>>::iterator
       iter = smnc_.servo_config.begin();
       iter != smnc_.servo_config.end();
       ++iter) {

    std::map<std::string, float> servo_config_params = iter->second;
    temp_servo_config.center = servo_config_params["center"];
    temp_servo_config.range = servo_config_params["range"];
    temp_servo_config.servo = servo_config_params["num"];
    temp_servo_config.direction = servo_config_params["direction"];

    // Append to temp_servo_config_array
    temp_servo_config_array.request.servos.push_back(temp_servo_config);
  }

  // call the client service, return true if succesful false if not
  if (!servos_config_client_.call(temp_servo_config_array)) {
    ROS_ERROR("Failed to call service servo_config");
    return false;
  }

  return true;
}


void SpotMicroMotionCmd::setServoCommandMessageData() {

  // Set the state of the spot micro kinematics object by setting the foot
  // positions, body position, and body orientation. Retrieve joint angles and
  // set the servo cmd message data
  sm_.setBodyState(body_state_cmd_);
  LegsJointAngles joint_angs = sm_.getLegsJointAngles();

  // Set the angles for the servo command message
  servo_cmds_rad_["RF_1"] = joint_angs.right_front.ang1;
  servo_cmds_rad_["RF_2"] = joint_angs.right_front.ang2;
  servo_cmds_rad_["RF_3"] = joint_angs.right_front.ang3;
 
  servo_cmds_rad_["RB_1"] = joint_angs.right_back.ang1;
  servo_cmds_rad_["RB_2"] = joint_angs.right_back.ang2;
  servo_cmds_rad_["RB_3"] = joint_angs.right_back.ang3;
 
  servo_cmds_rad_["LF_1"] = joint_angs.left_front.ang1;
  servo_cmds_rad_["LF_2"] = joint_angs.left_front.ang2;
  servo_cmds_rad_["LF_3"] = joint_angs.left_front.ang3;
 
  servo_cmds_rad_["LB_1"] = joint_angs.left_back.ang1;
  servo_cmds_rad_["LB_2"] = joint_angs.left_back.ang2;
  servo_cmds_rad_["LB_3"] = joint_angs.left_back.ang3;
}


void SpotMicroMotionCmd::publishServoProportionalCommand() {
  for (std::map<std::string, std::map<std::string, float>>::iterator
       iter = smnc_.servo_config.begin();
       iter != smnc_.servo_config.end();
       ++iter) {
 
    std::string servo_name = iter->first;
    std::map<std::string, float> servo_config_params = iter->second;
    
    int servo_num = servo_config_params["num"];
    float cmd_ang_rad = servo_cmds_rad_[servo_name];
    float center_ang_rad = servo_config_params["center_angle_deg"]*M_PI/180.0f;
    float servo_proportional_cmd = (cmd_ang_rad - center_ang_rad) /
                                   (smnc_.servo_max_angle_deg*M_PI/180.0f);
 
    if (servo_proportional_cmd > 1.0f) {
      servo_proportional_cmd = 1.0f;
      ROS_WARN("Proportional Command above +1.0 was computed, clipped to 1.0");
      ROS_WARN("Joint %s, Angle: %1.2f", servo_name.c_str(), cmd_ang_rad*180.0/M_PI);
 
    } else if (servo_proportional_cmd < -1.0f) {
      servo_proportional_cmd = -1.0f;
      ROS_WARN("Proportional Command below -1.0 was computed, clipped to -1.0");
      ROS_WARN("Joint %s, Angle: %1.2f", servo_name.c_str(), cmd_ang_rad*180.0/M_PI);
    }
 
    servo_array_.servos[servo_num-1].servo = servo_num;
    servo_array_.servos[servo_num-1].value = servo_proportional_cmd; 
 }

 // Publish message
 servos_proportional_pub_.publish(servo_array_);
}


void SpotMicroMotionCmd::publishZeroServoAbsoluteCommand() {
  // Publish the servo absolute message
  servos_absolute_pub_.publish(servo_array_absolute_);
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
  state_->handleInputCommands(this, sm_.getBodyState(), smnc_, cmd_);
}


void SpotMicroMotionCmd::changeState(std::unique_ptr<SpotMicroState> sms) {
  // Change the active state
  state_ = std::move(sms);

  // TODO: Call init method of new state?
  state_->init(this, sm_.getBodyState(), smnc_, cmd_);
}



