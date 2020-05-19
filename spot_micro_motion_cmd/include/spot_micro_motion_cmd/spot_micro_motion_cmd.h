// Declaration file 

#pragma once //designed to include the current source file only once in a single compilation.
#ifndef SPOT_MICRO_MOTION_CMD //usd for conditional compiling.
#define SPOT_MICRO_MOTION_CMD
#include <ros/ros.h>
#include "std_msgs/Bool.h"
#include "command.h"

#include "spot_micro_kinematics/spot_micro_kinematics.h"
#include "spot_micro_state.h"

/* defining the class */
class SpotMicroMotionCmd
{
    public:
        SpotMicroMotionCmd(ros::NodeHandle &nh, ros::NodeHandle &pnh); //constructor method
        ~SpotMicroMotionCmd(); // distructor method
        void runOnce(); // runOnce method to control the flow of program
    private:
        // Declare SpotMicroState a friend so it can access and modify private
        // members of this class
        friend class SpotMicroState;

        // Pointer to state object
        std::unique_ptr<SpotMicroState> state_;

        // Command object
        Command cmd_; // Command object, encapsulate commands
        

        // Spot Micro Kinematics object. Holds state of robot, and holds
        // kinematics operations for setting position/orientation of the robot
        smk::SpotMicroKinematics sm_; 

        // Ros publisher and subscriber handles
        ros::NodeHandle nh_; // Defining the ros NodeHandle variable for registrating the same with the master
        ros::Subscriber stand_sub_; // ros subscriber handle for stand_cmd topic
        ros::Subscriber idle_sub_; // ros subscriber handle for idle_cmd topic
        ros::Publisher servos_absolute_pub_; 

        // Callback method for stand command
        void standCommandCallback(const std_msgs::Bool::ConstPtr& msg);

        // Callback method for idle command
        void idleCommandCallback(const std_msgs::Bool::ConstPtr& msg);

        void resetEventCommands();

        // Handle input commands, delegate to state machine
        void handleInputCommands();

        // Changes state of the state machine
        void changeState(std::unique_ptr<SpotMicroState> sms);

        // Publish a servo configuration message
        

        // Publishes a servo proportional command message
        

        // Publishes a servo absolute command message
        

        // Set servo proprotional message data
        

        // Set sero absolute message data
};
#endif  
