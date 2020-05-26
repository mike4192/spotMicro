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
    // TODO: Command idle mode event
  }
  return 0;
}
