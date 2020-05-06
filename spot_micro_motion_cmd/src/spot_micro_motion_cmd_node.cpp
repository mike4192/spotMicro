// Node file to create object and initialising the ROS node
#include "spot_micro_motion_cmd.hpp" 
#include "command.h"
#include <iostream>
#include "spot_micro_fsm.h"


int main(int argc, char** argv)
{
    /* initialising the ROS node creating node handle
    for regestring it to the master and then private node handle to
    handle the parameters */
    ros::init(argc, argv, "spot_micro_motion_cmd"); 
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~"); 
    
    SpotMicroMotionCmd node(nh,pnh); // Creating the object
    
    smfsm::Command cmd;
    std::cout << "Got here!!!" << std::endl;
    
    smfsm::SpotMicroFsm fsm;

    ros::Rate rate(1.0); // Defing the looping rate

    /* Looking for any interupt else it will continue looping */
    while (ros::ok())
    {   
        node.runOnce();
        ros::spinOnce();
        fsm.handleInputCommands(cmd);
        rate.sleep();
    }
    return 0;
}
