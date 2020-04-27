#!/usr/bin/python

"""
Class for sending keyboard commands to spot micro walk node, control velocity, yaw rate, and trot event 
"""
import rospy
import sys, select, termios, tty # For terminal keyboard key press reading
from std_msgs.msg import Float32, Bool 

msg = """
Spot Micro Walk Command

Enter one of the following options:
-----------------------------
quit: stop and quit the program
oneServo: Move one servo manually, all others will be commanded to their center position
allServos: Move all servo's manually together

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

class SpotMicroKeyboardControl():
    def __init__(self):
        rospy.loginfo("Setting Up the Spot Micro Keyboard Control Node...")

        # Set up and title the ros node for this code
        rospy.init_node('spot_micro_keyboard_control')

        # Create publishers for x,y speed command, body rate command, and state command
        self.ros_pub_x_speed_cmd    = rospy.Publisher('/x_speed_cmd',Float32,queue_size=1)
        self.ros_pub_y_speed_cmd    = rospy.Publisher('/y_speed_cmd',Float32,queue_size=1)
        self.ros_pub_yaw_rate_cmd   = rospy.Publisher('/yaw_rate_cmd',Float32,queue_size=1)
        self.ros_pub_state_cmd      = rospy.Publisher('/state_cmd',Bool,queue_size=1)
        rospy.loginfo("> Publishers corrrectly initialized")

        rospy.loginfo("Initialization complete")

        # Setup terminal input reading, taken from teleop_twist_keyboard
        self.settings = termios.tcgetattr(sys.stdin)

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):

        # Prompt user with keyboard command information
        # Ability to control individual servo to find limits and center values
        # and ability to control all servos together
        
        while not rospy.is_shutdown():
            print(msg)
            userInput = raw_input("Command?: ")

            if userInput not in validCmds:
                print('Valid command not entered, try again...')
            else:
                if userInput == 'quit':
                    print("Ending program...")
                    print('Final Servo Values')
                    print('--------------------')
                    for i in range(numServos):
                        print('Servo %2i:   Min: %4i,   Center: %4i,   Max: %4i'%(i,self.servos[i]._min,self.servos[i]._center,self.servos[i]._max))                    
                    break

                elif userInput == 'oneServo':
                    # Reset all servos to center value, and send command
                    self.reset_all_servos_off()
                    self.send_servo_msg()

                    # First get servo number to command
                    nSrv = -1
                    while (1):
                        userInput = input('Which servo to control? Enter a number 1 through 12: ')
                        
                        if userInput not in range(1,numServos+1):
                            print("Invalid servo number entered, try again")
                        else:
                            nSrv = userInput - 1
                            break

                    # Loop and act on user command
                    print('Enter command, q to go back to option select: ')
                    while (1):
                       
                        userInput = self.getKey()

                        if userInput == 'q':
                            break
                        elif userInput not in keyDict:
                            print('Key not in valid key commands, try again')
                        else:
                            keyDict[userInput](self.servos[nSrv])
                            print('Servo %2i cmd: %4i'%(nSrv,self.servos[nSrv].value))
                            self.send_servo_msg()

                elif userInput == 'allServos':
                    # Reset all servos to center value, and send command
                    self.reset_all_servos_center()
                    self.send_servo_msg()

                    print('Enter command, q to go back to option select: ')                   
                    while (1):

                        userInput = self.getKey()

                        if userInput == 'q':
                            break
                        elif userInput not in keyDict:
                            print('Key not in valid key commands, try again')
                        elif userInput in ('b','n','m'):
                            print('Saving values not supported in all servo control mode')
                        else:
                            for s in self.servos.values():
                                keyDict[userInput](s)
                            print('All Servos Commanded')
                            self.send_servo_msg()
                                






            # Set the control rate in Hz
            rate = rospy.Rate(10)
            rate.sleep()

if __name__ == "__main__":
    smkc     = SpotMicroKeyboardControl()
    smkc.run()
