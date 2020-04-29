#!/usr/bin/python

"""
Class for sending keyboard commands to spot micro walk node, control velocity, yaw rate, and trot event 
"""
import rospy
import sys, select, termios, tty # For terminal keyboard key press reading
from std_msgs.msg import Float32, Bool 
from math import pi

msg = """
Spot Micro Walk Command

Enter one of the following options:
-----------------------------
quit: stop and quit the program
trot: Start trot mode and keyboard motion control

Keyboard commands for body motion 
---------------------------
   q   w   e         u
   a   s   d    
            

  u: Quit body motion command mode and go back to rest mode
  w: Increment forward speed command by 0.02 m/s
  a: Increment left speed command by 0.02 m/s
  s: Increment backward speed command by 0.02 m/s
  d: Increment right speed command by 0.02 m/s
  q: Increment body yaw rate command by -2 deg/s (negative left, positive right) 
  e: Increment body yaw rate command by +2 deg/s (negative left, positive right) 

  anything else : Prompt again for command


CTRL-C to quit
"""
valid_cmds = ('quit','Quit','trot')

# Global body motion increment values
speed_inc = 0.0025
yaw_rate_inc = 2*pi/180


class SpotMicroKeyboardControl():
    def __init__(self):

        # Create messages for body motion commands, and initialize to zero 
        self._x_speed_cmd_msg = Float32()
        self._x_speed_cmd_msg.data = 0

        self._y_speed_cmd_msg = Float32()
        self._y_speed_cmd_msg.data = 0

        self._yaw_rate_cmd_msg = Float32()
        self._yaw_rate_cmd_msg.data = 0

        self._trot_event_cmd_msg = Bool()
        self._trot_event_cmd_msg.data = True # Mostly acts as an event driven action on receipt of a true message

        rospy.loginfo("Setting Up the Spot Micro Keyboard Control Node...")

        # Set up and title the ros node for this code
        rospy.init_node('spot_micro_keyboard_control')

        # Create publishers for x,y speed command, body rate command, and state command
        self.ros_pub_x_speed_cmd    = rospy.Publisher('x_speed_cmd',Float32,queue_size=1)
        self.ros_pub_y_speed_cmd    = rospy.Publisher('/y_speed_cmd',Float32,queue_size=1)
        self.ros_pub_yaw_rate_cmd   = rospy.Publisher('/yaw_rate_cmd',Float32,queue_size=1)
        self.ros_pub_state_cmd      = rospy.Publisher('/state_cmd',Bool,queue_size=1)
        rospy.loginfo("> Publishers corrrectly initialized")

        rospy.loginfo("Initialization complete")

        # Setup terminal input reading, taken from teleop_twist_keyboard
        self.settings = termios.tcgetattr(sys.stdin)

    def reset_all_motion_commands_to_zero(self):
        '''Reset body motion cmd states to zero and publish zero value body motion commands'''
        self._x_speed_cmd_msg.data = 0
        self._y_speed_cmd_msg.data = 0
        self._yaw_rate_cmd_msg.data = 0

        self.ros_pub_x_speed_cmd.publish(self._x_speed_cmd_msg)
        self.ros_pub_y_speed_cmd.publish(self._y_speed_cmd_msg)
        self.ros_pub_yaw_rate_cmd.publish(self._yaw_rate_cmd_msg)

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        # Publish all body motion commands to 0
        self.reset_all_motion_commands_to_zero()

        # Prompt user with keyboard command information
        
        while not rospy.is_shutdown():
            print(msg)
            userInput = raw_input("Command?: ")

            if userInput not in valid_cmds:
                print('Valid command not entered, try again...')
            else:
                if userInput == 'quit':
                    print("Ending program...")
                    break

                elif userInput == 'trot':
                    # Reset all body motion commands to zero
                    self.reset_all_motion_commands_to_zero()

                    # Publish trot event
                    self.ros_pub_state_cmd.publish(self._trot_event_cmd_msg)

                    # Enter loop to act on user command

                    print('Enter command, q to go back to rest mode: ')

                    while (1):
                        print('Cmd Values: x speed: %1.2f m/s, y speed: %1.2f m/s, yaw rate: %1.2f deg/s '\
                                %(self._x_speed_cmd_msg.data,self._y_speed_cmd_msg.data,self._yaw_rate_cmd_msg.data*180/pi))
                       
                        userInput = self.getKey()

                        if userInput == 'u':
                            # Send trot event message, this will take robot back to rest mode
                            self.ros_pub_state_cmd.publish(self._trot_event_cmd_msg)
                            break

                        elif userInput not in ('w','a','s','d','q','e','u'):
                            print('Key not in valid key commands, try again')
                        else:
                            if userInput == 'w':
                                self._x_speed_cmd_msg.data = self._x_speed_cmd_msg.data + speed_inc
                                self.ros_pub_x_speed_cmd.publish(self._x_speed_cmd_msg)
                            
                            elif userInput == 's':
                                self._x_speed_cmd_msg.data = self._x_speed_cmd_msg.data - speed_inc
                                self.ros_pub_x_speed_cmd.publish(self._x_speed_cmd_msg)

                            elif userInput == 'a':
                                self._y_speed_cmd_msg.data = self._y_speed_cmd_msg.data + speed_inc
                                self.ros_pub_y_speed_cmd.publish(self._y_speed_cmd_msg)
                            
                            elif userInput == 'd':
                                self._y_speed_cmd_msg.data = self._y_speed_cmd_msg.data - speed_inc
                                self.ros_pub_y_speed_cmd.publish(self._y_speed_cmd_msg)

                            elif userInput == 'q':
                                self._yaw_rate_cmd_msg.data = self._yaw_rate_cmd_msg.data + yaw_rate_inc
                                self.ros_pub_yaw_rate_cmd.publish(self._yaw_rate_cmd_msg)

                            elif userInput == 'e':
                                self._yaw_rate_cmd_msg.data = self._yaw_rate_cmd_msg.data - yaw_rate_inc
                                self.ros_pub_yaw_rate_cmd.publish(self._yaw_rate_cmd_msg)


if __name__ == "__main__":
    smkc     = SpotMicroKeyboardControl()
    smkc.run()
