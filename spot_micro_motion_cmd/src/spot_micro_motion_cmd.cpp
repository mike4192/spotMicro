#include "spot_micro_motion_cmd.hpp"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "spot_micro_kinematics/spot_micro_kinematics.h"

void SpotMicroMotionCmd::standCommandCallback(const std_msgs::Bool::ConstPtr& msg)
{
    // Toggle event true only on rising edge from external command
    if (msg->data == true)
    {
        cmd.stand_cmd = true;
    }
}

void SpotMicroMotionCmd::consumeEventCommands()
{
    // Consume all event commands, setting all command states false if they were true 
    cmd.getStandCmd(); 
}
//constructor method
SpotMicroMotionCmd::SpotMicroMotionCmd(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{
    nh_ = nh;
    std::cout<<"from Constructor \n";

    // Initialize Command and Spot Micro Fini State Machine Objects
    cmd = smfsm::Command();
    fsm = smfsm::SpotMicroFsm();
   
   // Initialize spot micro kinematics object 
    smk::SpotMicroConfig smc = {.hip_link_length = 0.03,
                                .upper_leg_link_length = 0.07f,
                                .lower_leg_link_length = 0.07f,
                                .body_width = 0.1f,
                                .body_length = 0.1f};
    
    smk::SpotMicroKinematics sm = smk::SpotMicroKinematics(0.0f, 0.0f, 0.0f, smc);

    // storing the values in the member variable
    // get the parameters or configurations and store them in member variables
    // initialize the publisher and subscribers
    _standSub = nh.subscribe("/stand_cmd", 1, &SpotMicroMotionCmd::standCommandCallback, this); // Defining a subsriber

}

// Distructor method
SpotMicroMotionCmd::~SpotMicroMotionCmd()
{
    std::cout<<"from Distructor \n";
    // Free up the memory assigned from heap
}

void SpotMicroMotionCmd::runOnce()
{
    std::cout<<"from Runonce \n";

    fsm.handleInputCommands(cmd);

    // Consume all event commands.
    // This resets all event commands if they were true. Doing this enforces a rising edge detection
    consumeEventCommands();
}
