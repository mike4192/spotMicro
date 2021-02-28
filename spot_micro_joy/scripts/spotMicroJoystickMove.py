#!/usr/bin/python

import rospy
from std_msgs.msg import Float32, Bool
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
from math import pi


class SpotMicroJoystickControl():
    BUTTON_GREEN = 0
    BUTTON_RED = 1
    BUTTON_YELLOW = 2
    BUTTON_BLUE = 3

    AXES_ROLL = 0
    AXES_HEIGHT = 1
    AXES_YAW = 2
    AXES_PITCH = 3

    WALK_AXES_FORWARD = 1
    WALK_AXES_STRAFE = 0
    WALK_AXES_YAW = 2

    MODE_IDLE = 0
    MODE_STAND = 1
    MODE_ANGLE = 2
    MODE_WALK = 3

    def __init__(self):

        self._angle_cmd_msg = Vector3()
        self._angle_cmd_msg.x = 0
        self._angle_cmd_msg.y = 0
        self._angle_cmd_msg.z = 0

        self._speed_cmd_msg = Vector3()
        self._speed_cmd_msg.x = 0
        self._speed_cmd_msg.y = 0
        self._speed_cmd_msg.z = 0

        self._vel_cmd_msg = Twist()
        self._vel_cmd_msg.linear.x = 0
        self._vel_cmd_msg.linear.y = 0
        self._vel_cmd_msg.linear.z = 0
        self._vel_cmd_msg.angular.x = 0
        self._vel_cmd_msg.angular.y = 0
        self._vel_cmd_msg.angular.z = 0

        self._walk_event_cmd_msg = Bool()
        self._walk_event_cmd_msg.data = True  # Mostly acts as an event driven action on receipt of a true message

        self._stand_event_cmd_msg = Bool()
        self._stand_event_cmd_msg.data = True

        self._idle_event_cmd_msg = Bool()
        self._idle_event_cmd_msg.data = True

        rospy.loginfo("Setting Up the Spot Micro Joystick Control Node...")

        # Set up and title the ros node for this code
        rospy.init_node('spot_micro_joystick_control')

        # Create publishers for commanding velocity, angle, and robot states
        self._ros_pub_angle_cmd = rospy.Publisher('/angle_cmd', Vector3, queue_size=1)
        self._ros_pub_vel_cmd = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self._ros_pub_walk_cmd = rospy.Publisher('/walk_cmd', Bool, queue_size=1)
        self._ros_pub_stand_cmd = rospy.Publisher('/stand_cmd', Bool, queue_size=1)
        self._ros_pub_idle_cmd = rospy.Publisher('/idle_cmd', Bool, queue_size=1)

        rospy.loginfo("Joystick control node publishers corrrectly initialized")

    def reset_all_motion_commands_to_zero(self):
        '''Reset body motion cmd states to zero and publish zero value body motion commands'''

        self._vel_cmd_msg.linear.x = 0
        self._vel_cmd_msg.linear.y = 0
        self._vel_cmd_msg.linear.z = 0
        self._vel_cmd_msg.angular.x = 0
        self._vel_cmd_msg.angular.y = 0
        self._vel_cmd_msg.angular.z = 0

        self._ros_pub_vel_cmd.publish(self._vel_cmd_msg)

    def reset_all_angle_commands_to_zero(self):
        '''Reset angle cmd states to zero and publish them'''

        self._angle_cmd_msg.x = 0
        self._angle_cmd_msg.y = 0
        self._angle_cmd_msg.z = 0

        self._ros_pub_angle_cmd.publish(self._angle_cmd_msg)

    def on_joy(self, msg):
        if msg.buttons[self.BUTTON_GREEN] == 1:
            self._ros_pub_idle_cmd.publish(self._idle_event_cmd_msg)
            rospy.loginfo('Idle command issued from joystick.')
            self.mode = self.MODE_IDLE

        if msg.buttons[self.BUTTON_YELLOW] == 1:
            self._ros_pub_stand_cmd.publish(self._stand_event_cmd_msg)
            rospy.loginfo('Stand command issued from joystick.')
            self.mode = self.MODE_STAND

        if msg.buttons[self.BUTTON_BLUE] == 1:
            self.reset_all_angle_commands_to_zero()
            rospy.loginfo('Entering joystick angle command mode.')
            self.mode = self.MODE_ANGLE

        if msg.buttons[self.BUTTON_RED] == 1:
            self.reset_all_angle_commands_to_zero()
            self._ros_pub_walk_cmd.publish(self._walk_event_cmd_msg)
            rospy.loginfo('Entering joystick walk command mode.')
            self.mode = self.MODE_WALK

        if self.mode == self.MODE_ANGLE:
            print('Cmd Values: phi: %1.3f deg, theta: %1.3f deg, psi: %1.3f deg ' \
                  % (
                  self._angle_cmd_msg.x * 180 / pi, self._angle_cmd_msg.y * 180 / pi, self._angle_cmd_msg.z * 180 / pi))

            self._angle_cmd_msg.x = pi / 180 * msg.axes[self.AXES_ROLL] * 45 * -1
            self._angle_cmd_msg.y = pi / 180 * msg.axes[self.AXES_PITCH] * 45 * -1
            self._angle_cmd_msg.z = pi / 180 * msg.axes[self.AXES_YAW] * 45
            self._ros_pub_angle_cmd.publish(self._angle_cmd_msg)

        if self.mode == self.MODE_WALK:
            self._vel_cmd_msg.linear.x = msg.axes[self.WALK_AXES_FORWARD] * 0.05
            self._vel_cmd_msg.linear.y = msg.axes[self.WALK_AXES_STRAFE] * 0.05
            self._vel_cmd_msg.angular.z = pi / 180 * msg.axes[self.AXES_YAW] * 15

            self._ros_pub_vel_cmd.publish(self._vel_cmd_msg)
            print('Cmd Values: x speed: %1.3f m/s, y speed: %1.3f m/s, yaw rate: %1.3f deg/s ' \
                  % (self._vel_cmd_msg.linear.x, self._vel_cmd_msg.linear.y, self._vel_cmd_msg.angular.z * 180 / pi))

    def run(self):
        print("green = idle")
        print("yellow = stand")
        print("blue = angle")
        print("red = walk")

        # Publish all body motion commands to 0
        self.reset_all_motion_commands_to_zero()
        rospy.Subscriber("/joy", Joy, self.on_joy)
        rospy.spin()


if __name__ == "__main__":
    smjc = SpotMicroJoystickControl()
    smjc.run()
