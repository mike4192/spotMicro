// Declaration file 

#pragma once //designed to include the current source file only once in a single compilation.
#ifndef SPOT_MICRO_MOTION_CMD //usd for conditional compiling.
#define SPOT_MICRO_MOTION_CMD
#include <ros/ros.h>


/* defining the class */
class SpotMicroMotionCmd
{
    public:
        SpotMicroMotionCmd(ros::NodeHandle &nh, ros::NodeHandle &pnh); //constructor method
        ~SpotMicroMotionCmd(); // distructor method
        void runOnce(); // runOnce method to control the flow of program
    private:
        ros::NodeHandle nh_; // Defining the ros NodeHandle variable for registrating the same with the master
};
#endif  
