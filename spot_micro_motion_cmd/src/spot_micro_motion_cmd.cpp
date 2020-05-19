#include "spot_micro_motion_cmd.hpp"

//constructor method
SpotMicroMotionCmd::SpotMicroMotionCmd(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{
    nh_ = nh;
    std::cout<<"from Constructor \n";
    // storing the values in the member variable
    // get the parameters or configurations and store them in member variables
    // initialize the publisher and subscribers
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
}
