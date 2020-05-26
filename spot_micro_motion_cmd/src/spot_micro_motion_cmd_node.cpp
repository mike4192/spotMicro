// Node file to create object and initialising the ROS node
#include "spot_micro_motion_cmd.h" 
#include <iostream>


int main(int argc, char** argv) {
  /* initialising the ROS node creating node handle
  for regestring it to the master and then private node handle to
  handle the parameters */
  ros::init(argc, argv, "spot_micro_motion_cmd"); 
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~"); 
 
  SpotMicroMotionCmd node(nh,pnh); // Creating the object

  ros::Rate rate(1.0/node.getNodeConfig().dt); // Defing the looping rate


  // Only proceed if servo configuration publishing succeeds
   if (node.publishServoConfiguration()) {
  
  //node.publishServoConfiguration();
    
    /* Looking for any interupt else it will continue looping */
    // Main loop runs indefinitely unless there is an interupt call
    while (ros::ok())
    {   
        node.runOnce();
        ros::spinOnce();
        rate.sleep();
    }
    // On exit, command idle mode event to safely lower robot and disable servo
    // position hold
    // Command idle mode
    node.commandIdle();

    //std::cout << node.getCurrentStateName() << std::endl;
    //Continue loop until idle mode is entered
    while (node.getCurrentStateName().compare("Idle") != 0) {
      std::cout << node.getCurrentStateName() << std::endl;
      node.runOnce();
      ros::spinOnce();
      rate.sleep(); 
    }
    node.runOnce();


  }
  return 0;
}
