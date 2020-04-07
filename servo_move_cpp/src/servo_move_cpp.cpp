
#include "ros/ros.h"
#include "std_msgs/String.h"

#include "i2cpwm_board/Servo.h"
#include "i2cpwm_board/ServoArray.h"

#include <sstream>
#include <string>
#include <iostream>


int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */

    ros::init(argc, argv, "servo_move_cpp");


  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */

    ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */

    ros::Publisher servo_move_pub = n.advertise<i2cpwm_board::ServoArray>("servos_absolute", 500);

    ros::Rate loop_rate(0.5);

    // Instantiate servo message, and servo array message objects
    i2cpwm_board::ServoArray srvArray;
    i2cpwm_board::Servo srv;

    int count = 0;

    srv.servo = 1;

    srv.value = 100;

    std::vector<i2cpwm_board::Servo> tempCopy;

    tempCopy.push_back(srv);

    // Add servo to servo array?
    //srvArray.servos.push_back(srv);

    while (ros::ok())
    {
        if (tempCopy[0].value == 400)
        {
            tempCopy[0].value = 100;
        }
        else 
        {
            tempCopy[0].value = 400;
        }

        ROS_INFO("Servo value from tempCopy: %f",tempCopy[0].value);
        
        // Copy data to srvArray message
        srvArray.servos = tempCopy;
        servo_move_pub.publish(srvArray);

        ROS_INFO("Length of servo array: %i",srvArray.servos.size());

        loop_rate.sleep();
        ++count;
        ROS_INFO("Here: %i",count);
    }


    return 0;
}
// %EndTag(FULLTEXT)%