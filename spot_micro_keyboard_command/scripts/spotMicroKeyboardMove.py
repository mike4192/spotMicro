#!/usr/bin/env python3

"""
Class for sending keyboard commands to spot micro walk node, control velocity, yaw rate, and walk event
"""
import rclpy
from rclpy.node import Node
import sys, select, termios, tty
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
from math import pi

msg = """
Spot Micro Walk Command

Enter one of the following options:
-----------------------------
quit: stop and quit the program
walk: Start walk mode and keyboard motion control
stand: Stand robot up

Keyboard commands for body motion
---------------------------
   q   w   e            u
   a   s   d


  u: Quit body motion command mode and go back to rest mode
  w: Increment forward speed command / decrease pitch angle
  a: Increment left speed command / left roll angle
  s: Increment backward speed command / increase pitch angle
  d: Increment right speed command / right roll angle
  q: Increment body yaw rate command / left yaw angle (negative left, positive right)
  e: Increment body yaw rate command / right yaw angle (negative left, positive right)
  f: In walk mode, zero out all rate commands.

  anything else : Prompt again for command


CTRL-C to quit
"""
valid_cmds = ('quit','Quit','walk','stand','idle', 'angle_cmd')

# Global body motion increment values
speed_inc = 0.02
yaw_rate_inc = 3*pi/180
angle_inc = 2.5*pi/180

class SpotMicroKeyboardControl(Node):
    def __init__(self):
        super().__init__('spot_micro_keyboard_control')

        self._angle_cmd_msg = Vector3()
        self._angle_cmd_msg.x = 0.0
        self._angle_cmd_msg.y = 0.0
        self._angle_cmd_msg.z = 0.0

        self._speed_cmd_msg = Vector3()
        self._speed_cmd_msg.x = 0.0
        self._speed_cmd_msg.y = 0.0
        self._speed_cmd_msg.z = 0.0

        self._vel_cmd_msg = Twist()
        self._vel_cmd_msg.linear.x = 0.0
        self._vel_cmd_msg.linear.y = 0.0
        self._vel_cmd_msg.linear.z = 0.0
        self._vel_cmd_msg.angular.x = 0.0
        self._vel_cmd_msg.angular.y = 0.0
        self._vel_cmd_msg.angular.z = 0.0

        self._walk_event_cmd_msg = Bool()
        self._walk_event_cmd_msg.data = True

        self._stand_event_cmd_msg = Bool()
        self._stand_event_cmd_msg.data = True

        self._idle_event_cmd_msg = Bool()
        self._idle_event_cmd_msg.data = True

        self.get_logger().info("Setting Up the Spot Micro Keyboard Control Node...")

        self._ros_pub_angle_cmd = self.create_publisher(Vector3, '/angle_cmd', 1)
        self._ros_pub_vel_cmd = self.create_publisher(Twist, '/cmd_vel', 1)
        self._ros_pub_walk_cmd = self.create_publisher(Bool, '/walk_cmd', 1)
        self._ros_pub_stand_cmd = self.create_publisher(Bool, '/stand_cmd', 1)
        self._ros_pub_idle_cmd = self.create_publisher(Bool, '/idle_cmd', 1)

        self.get_logger().info("Keyboard control node publishers correctly initialized")

        self.settings = termios.tcgetattr(sys.stdin)

        self.get_logger().info("Keyboard control node initialization complete")

    def reset_all_motion_commands_to_zero(self):
        self._vel_cmd_msg.linear.x = 0.0
        self._vel_cmd_msg.linear.y = 0.0
        self._vel_cmd_msg.linear.z = 0.0
        self._vel_cmd_msg.angular.x = 0.0
        self._vel_cmd_msg.angular.y = 0.0
        self._vel_cmd_msg.angular.z = 0.0

        self._ros_pub_vel_cmd.publish(self._vel_cmd_msg)

    def reset_all_angle_commands_to_zero(self):
        self._angle_cmd_msg.x = 0.0
        self._angle_cmd_msg.y = 0.0
        self._angle_cmd_msg.z = 0.0

        self._ros_pub_angle_cmd.publish(self._angle_cmd_msg)

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        self.reset_all_motion_commands_to_zero()
        self.get_logger().info('Main keyboard control loop started.')

        try:
            while rclpy.ok():
                print(msg)
                userInput = input("Command?: ")

                if userInput not in valid_cmds:
                    self.get_logger().warn('Invalid keyboard command entered: %s' % userInput)
                else:
                    if userInput == 'quit':
                        self.get_logger().info("Exiting keyboard control node...")
                        break

                    elif userInput == 'stand':
                        self._ros_pub_stand_cmd.publish(self._stand_event_cmd_msg)
                        self.get_logger().info('Stand command issued from keyboard.')

                    elif userInput == 'idle':
                        self._ros_pub_idle_cmd.publish(self._idle_event_cmd_msg)
                        self.get_logger().info('Idle command issued from keyboard.')

                    elif userInput == 'angle_cmd':
                        self.reset_all_angle_commands_to_zero()
                        self.get_logger().info('Entering keyboard angle command mode.')
                        print('Enter command, u to go back to command select: ')

                        while rclpy.ok():
                            print('Cmd Values: phi: %1.3f deg, theta: %1.3f deg, psi: %1.3f deg ' \
                                  % (self._angle_cmd_msg.x*180/pi, self._angle_cmd_msg.y*180/pi, self._angle_cmd_msg.z*180/pi))

                            userInput = self.getKey()

                            if userInput == 'u':
                                break

                            elif userInput not in ('w','a','s','d','q','e','u'):
                                self.get_logger().warn('Invalid keyboard command issued in angle command mode: %s' % userInput)
                            else:
                                if userInput == 'w':
                                    self._angle_cmd_msg.y = self._angle_cmd_msg.y - angle_inc
                                    self._ros_pub_angle_cmd.publish(self._angle_cmd_msg)

                                elif userInput == 's':
                                    self._angle_cmd_msg.y = self._angle_cmd_msg.y + angle_inc
                                    self._ros_pub_angle_cmd.publish(self._angle_cmd_msg)

                                elif userInput == 'q':
                                    self._angle_cmd_msg.z = self._angle_cmd_msg.z + angle_inc
                                    self._ros_pub_angle_cmd.publish(self._angle_cmd_msg)

                                elif userInput == 'e':
                                    self._angle_cmd_msg.z = self._angle_cmd_msg.z - angle_inc
                                    self._ros_pub_angle_cmd.publish(self._angle_cmd_msg)

                                elif userInput == 'a':
                                    self._angle_cmd_msg.x = self._angle_cmd_msg.x - angle_inc
                                    self._ros_pub_angle_cmd.publish(self._angle_cmd_msg)

                                elif userInput == 'd':
                                    self._angle_cmd_msg.x = self._angle_cmd_msg.x + angle_inc
                                    self._ros_pub_angle_cmd.publish(self._angle_cmd_msg)

                    elif userInput == 'walk':
                        self.reset_all_motion_commands_to_zero()
                        self._ros_pub_walk_cmd.publish(self._walk_event_cmd_msg)
                        self.get_logger().info('Walk command issued from keyboard.')

                        print('Enter command, u to go back to stand mode: ')

                        while rclpy.ok():
                            print('Cmd Values: x speed: %1.3f m/s, y speed: %1.3f m/s, yaw rate: %1.3f deg/s ' \
                                  % (self._vel_cmd_msg.linear.x, self._vel_cmd_msg.linear.y, self._vel_cmd_msg.angular.z*180/pi))

                            userInput = self.getKey()

                            if userInput == 'u':
                                self._ros_pub_stand_cmd.publish(self._stand_event_cmd_msg)
                                self.get_logger().info('Stand command issued from keyboard.')
                                break

                            elif userInput not in ('w','a','s','d','q','e','u','f'):
                                print('Key not in valid key commands, try again')
                                self.get_logger().warn('Invalid keyboard command issued in walk mode: %s' % userInput)
                            else:
                                if userInput == 'w':
                                    self._vel_cmd_msg.linear.x = self._vel_cmd_msg.linear.x + speed_inc
                                    self._ros_pub_vel_cmd.publish(self._vel_cmd_msg)

                                elif userInput == 's':
                                    self._vel_cmd_msg.linear.x = self._vel_cmd_msg.linear.x - speed_inc
                                    self._ros_pub_vel_cmd.publish(self._vel_cmd_msg)

                                elif userInput == 'a':
                                    self._vel_cmd_msg.linear.y = self._vel_cmd_msg.linear.y - speed_inc
                                    self._ros_pub_vel_cmd.publish(self._vel_cmd_msg)

                                elif userInput == 'd':
                                    self._vel_cmd_msg.linear.y = self._vel_cmd_msg.linear.y + speed_inc
                                    self._ros_pub_vel_cmd.publish(self._vel_cmd_msg)

                                elif userInput == 'q':
                                    self._vel_cmd_msg.angular.z = self._vel_cmd_msg.angular.z - yaw_rate_inc
                                    self._ros_pub_vel_cmd.publish(self._vel_cmd_msg)

                                elif userInput == 'e':
                                    self._vel_cmd_msg.angular.z = self._vel_cmd_msg.angular.z + yaw_rate_inc
                                    self._ros_pub_vel_cmd.publish(self._vel_cmd_msg)

                                elif userInput == 'f':
                                    self._vel_cmd_msg.linear.x = 0.0
                                    self._vel_cmd_msg.linear.y = 0.0
                                    self._vel_cmd_msg.angular.z = 0.0

                                    self._ros_pub_vel_cmd.publish(self._vel_cmd_msg)
                                    self.get_logger().info('Command issued to zero all rate commands.')
        except KeyboardInterrupt:
            pass
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

if __name__ == "__main__":
    rclpy.init()
    smkc = SpotMicroKeyboardControl()
    smkc.run()
    rclpy.shutdown()
