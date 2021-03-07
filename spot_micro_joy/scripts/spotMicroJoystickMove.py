#!/usr/bin/python

import rospy
from std_msgs.msg import Float32, Bool
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
from math import pi


class SpotMicroJoystickControl():
    BUTTON_IDLE = 0
    BUTTON_WALK = 1
    BUTTON_STAND = 2
    BUTTON_ANGLE = 3

    ANGLE_AXES_ROLL = 0
    ANGLE_AXES_HEIGHT = 1
    ANGLE_AXES_YAW = 2
    ANGLE_AXES_PITCH = 3

    WALK_AXES_FORWARD = 1
    WALK_AXES_STRAFE = 0
    WALK_AXES_YAW = 2

    MODE_IDLE = 0
    MODE_STAND = 1
    MODE_ANGLE = 2
    MODE_WALK = 3

    MAX_ROLL_DEG = 45
    MAX_YAW_DEG = 45
    MAX_PATCH_DEG = 45

    MAX_FORWARD_SPEED = 0.05
    MAX_STRAFE_SPEED = 0.05
    MAX_YAW_SPEED_DEG = 15

    def __init__(self):

        self._angle_cmd_msg = Vector3()
        self._angle_cmd_msg.x = 0
        self._angle_cmd_msg.y = 0
        self._angle_cmd_msg.z = 0

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
        self.on_joy_buttons(msg.buttons)
        self.on_joy_axes(msg.axes)

    def on_joy_buttons(self, buttons):
        if buttons[self.BUTTON_IDLE] == 1:
            self._ros_pub_idle_cmd.publish(self._idle_event_cmd_msg)
            rospy.loginfo('Idle command issued from joystick.')
            self.mode = self.MODE_IDLE
        elif buttons[self.BUTTON_STAND] == 1:
            self._ros_pub_stand_cmd.publish(self._stand_event_cmd_msg)
            rospy.loginfo('Stand command issued from joystick.')
            self.mode = self.MODE_STAND
        elif buttons[self.BUTTON_ANGLE] == 1:
            self.reset_all_angle_commands_to_zero()
            rospy.loginfo('Entering joystick angle command mode.')
            self.mode = self.MODE_ANGLE
        elif buttons[self.BUTTON_WALK] == 1:
            self.reset_all_angle_commands_to_zero()
            self._ros_pub_walk_cmd.publish(self._walk_event_cmd_msg)
            rospy.loginfo('Entering joystick walk command mode.')
            self.mode = self.MODE_WALK

    def on_joy_axes(self, axes):
        if self.mode == self.MODE_ANGLE:
            self.on_joy_angle_mode(axes)
        elif self.mode == self.MODE_WALK:
            self.on_joy_walk_mode(axes)

    def on_joy_walk_mode(self, axes):
        self._vel_cmd_msg.linear.x = axes[self.WALK_AXES_FORWARD] * self.MAX_FORWARD_SPEED
        self._vel_cmd_msg.linear.y = axes[self.WALK_AXES_STRAFE] * self.MAX_STRAFE_SPEED
        self._vel_cmd_msg.angular.z = pi / 180 * axes[self.WALK_AXES_YAW] * self.MAX_YAW_SPEED_DEG
        print('Cmd Values: x speed: %1.3f m/s, y speed: %1.3f m/s, yaw rate: %1.3f deg/s ' \
              % (self._vel_cmd_msg.linear.x, self._vel_cmd_msg.linear.y, self._vel_cmd_msg.angular.z * 180 / pi))
        self._ros_pub_vel_cmd.publish(self._vel_cmd_msg)

    def on_joy_angle_mode(self, axes):
        self._angle_cmd_msg.x = pi / 180 * axes[self.ANGLE_AXES_ROLL] * self.MAX_ROLL_DEG * -1
        self._angle_cmd_msg.y = pi / 180 * axes[self.ANGLE_AXES_PITCH] * self.MAX_PATCH_DEG * -1
        self._angle_cmd_msg.z = pi / 180 * axes[self.ANGLE_AXES_YAW] * self.MAX_YAW_DEG
        print('Cmd Values: phi: %1.3f deg, theta: %1.3f deg, psi: %1.3f deg ' \
              % (
                  self._angle_cmd_msg.x * 180 / pi, self._angle_cmd_msg.y * 180 / pi,
                  self._angle_cmd_msg.z * 180 / pi))
        self._ros_pub_angle_cmd.publish(self._angle_cmd_msg)

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
