

'''
Main package file for running and issuing simple body pose commands to spot micro robot
'''

import rospy
from i2cpwm_board.msg import Servo, ServoArray, ServoConfig, ServoConfigArray
from i2cpwm_board.srv import ServosConfig
from spot_micro_simple_command.spot_micro_kinematics.spot_micro_stick_figure import SpotMicroStickFigure
from spot_micro_simple_command.first_order_filter.fof import FirstOrderFilter 
from math import pi
import numpy as np


#########################################################################
########################## Global Variables #############################
#########################################################################
num_servos = 12

r2d = 180/pi
d2r = pi/180

# Dictionary to encapsulate servo configuration
#   num: Port that servo is plugged into servo board, starting from index 1
#   center: Center PWM value to be set
#   range: Total PWM range to be set for servo, max_pwm - min_pwm
#   direction: Direction of servo to robot angle
#   center_angle_deg: Angle of robot limb at center PWM val

servo_max_angle_deg = 70
servo_dict = {
                'RF_3': {'num':1, 'center':306,'range':367,'direction': 1,'center_angle_deg': 57.2},
                'RF_2': {'num':2, 'center':306,'range':328,'direction': 1,'center_angle_deg':-25.6},
                'RF_1': {'num':3, 'center':320,'range':352,'direction':-1,'center_angle_deg':  0.0},
                'RB_3': {'num':4, 'center':306,'range':331,'direction': 1,'center_angle_deg': 54.8},
                'RB_2': {'num':5, 'center':306,'range':344,'direction': 1,'center_angle_deg':-17.1},
                'RB_1': {'num':6, 'center':306,'range':345,'direction': 1,'center_angle_deg':  0.0},
                'LB_3': {'num':7, 'center':306,'range':347,'direction':1,'center_angle_deg': -52.6},
                'LB_2': {'num':8, 'center':306,'range':333,'direction':1,'center_angle_deg':  30.3},
                'LB_1': {'num':9, 'center':316,'range':343,'direction':-1,'center_angle_deg':  0.0},
                'LF_3': {'num':10,'center':306,'range':333,'direction':1,'center_angle_deg': -55.1},
                'LF_2': {'num':11,'center':306,'range':347,'direction':1,'center_angle_deg':  26.6},
                'LF_1': {'num':12,'center':282,'range':342,'direction': 1,'center_angle_deg':  0.0},
            }

class SpotMicroSimpleCommand():
    '''Class to encapsulate simple command of spot micro robot'''

    def __init__(self):
        '''Constructor'''

        # Create and publish servo config message
        # Initialize servo_config_msg
        self._servo_config_msg = ServoConfigArray()
        for s in servo_dict.values():
            temp_servo = ServoConfig()
            temp_servo.center = s['center']
            temp_servo.range = s['range']
            temp_servo.servo = s['num']
            temp_servo.direction = s['direction']

            # Append servo to servo config message
            self._servo_config_msg.servos.append(temp_servo)

        # Publish servo configuration
        rospy.loginfo("> Waiting for config_servos service...")
        rospy.wait_for_service('config_servos')
        rospy.loginfo("> Config_servos service found!")
        try:
            servoConfigService = rospy.ServiceProxy('config_servos',ServosConfig)
            resp = servoConfigService(self._servo_config_msg.servos)
            print("Config servos done!!, returned value: %i"%resp.error)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e



        # Set up and initialize ros node
        rospy.loginfo("Setting Up the Spot Micro Simple Command Node...")

        # Set up and title the ros node for this code
        rospy.init_node('spot_micro_simple_command') 

        # Create a servo command dictionary in the same order as angles are received back from
        # SpotMicroStickFigure.get_leg_angles
        self.servo_cmds_rad = {'RB_1':0,'RB_2':0,'RB_3':0,
                               'RF_1':0,'RF_2':0,'RF_3':0,
                               'LF_1':0,'LF_2':0,'LF_3':0,
                               'LB_1':0,'LB_2':0,'LB_3':0}
        
        # Create empty ServoArray message with n number of Servos in its array
        self._servo_msg       = ServoArray()
        for _ in range(len(servo_dict)): 
            self._servo_msg.servos.append(Servo())

        # Create the servo array publisher
        self.ros_pub_servo_array    = rospy.Publisher("/servos_proportional", ServoArray, queue_size=1)
        rospy.loginfo("> Publisher corrrectly initialized")

        rospy.loginfo("Initialization complete")

        # Create a spot micro stick figure object to encapsulate robot state
        self.sm = SpotMicroStickFigure(y=0.18)

        # Set absolute foot positions
        l = self.sm.body_length
        w = self.sm.body_width
        l1 = self.sm.hip_length
        desired_p4_points = np.array([ [-l/2,   0,  w/2 + l1],
                                    [ l/2 ,  0,  w/2 + l1],
                                    [ l/2 ,  0, -w/2 - l1],
                                    [-l/2 ,  0, -w/2 - l1] ])

        self.sm.set_absolute_foot_coordinates(desired_p4_points)



    def send_servo_cmd_msg(self):
        # Loop through servo_cmds_rad, calculate pwm value to send, 
        # populate _servo_cmd_msg, and publish
        for s in self.servo_cmds_rad:
            
            servo_num = servo_dict[s]['num']
            cmd_ang_rad = self.servo_cmds_rad[s]
            center_ang_rad = servo_dict[s]['center_angle_deg']*d2r
            servo_proportional_cmd = (cmd_ang_rad - center_ang_rad) / (servo_max_angle_deg*d2r)

            if servo_proportional_cmd > 1.0:
                print('WARNING: Proportional command above +1.0 was calculated, clipped to 1.0')
                print('Joint: %s, Angle: %1.2f'%(s,cmd_ang_rad*r2d))
                servo_proportional_cmd = 1.0
            elif servo_proportional_cmd < -1.0:
                print('WARNING: Proportional command above -1.0 was calculated, clipped to -1.0')
                print('Joint: %s, Angle: %1.2f'%(s,cmd_ang_rad*r2d))
                servo_proportional_cmd = -1.0

            self._servo_msg.servos[servo_num-1].servo = servo_num
            self._servo_msg.servos[servo_num-1].value = servo_proportional_cmd 

        
        # Publish message
        self.ros_pub_servo_array.publish(self._servo_msg)

    def run(self):
        
        # Send initial 0 command to all servos
        self.send_servo_cmd_msg()

        # Define the loop rate in Hz
        rate = rospy.Rate(50)

        fof = FirstOrderFilter(0.02,tau=1,x0=0)


        cmd_ang_1 = 20*d2r
        cmd_ang_2 = -20*d2r
        cmd = cmd_ang_1
        time = 0

        fof.set_command(cmd_ang_1)

        while not rospy.is_shutdown():
            time += 0.02

            # Switch command every x seconds
            if time > 4:
                time = 0
                if cmd == cmd_ang_2:
                    cmd = cmd_ang_1
                else:
                    cmd = cmd_ang_2
                fof.set_command(cmd)
            
            filtered_cmd_ang = fof.run_timestep_and_get_output()


            # Command body pose
            self.sm.set_body_angles(phi=filtered_cmd_ang)

            # Get leg angles
            leg_angs = self.sm.get_leg_angles()


            # Set leg angles
            self.servo_cmds_rad['RB_1'] = leg_angs[0][0]
            self.servo_cmds_rad['RB_2'] = leg_angs[0][1]
            self.servo_cmds_rad['RB_3'] = leg_angs[0][2]
            self.servo_cmds_rad['RF_1'] = leg_angs[1][0]
            self.servo_cmds_rad['RF_2'] = leg_angs[1][1]
            self.servo_cmds_rad['RF_3'] = leg_angs[1][2]
            self.servo_cmds_rad['LF_1'] = leg_angs[2][0]
            self.servo_cmds_rad['LF_2'] = leg_angs[2][1]
            self.servo_cmds_rad['LF_3'] = leg_angs[2][2]
            self.servo_cmds_rad['LB_1'] = leg_angs[3][0]
            self.servo_cmds_rad['LB_2'] = leg_angs[3][1]
            self.servo_cmds_rad['LB_3'] = leg_angs[3][2]


            self.send_servo_cmd_msg()

            
            # print(filtered_cmd_ang*r2d)
            
            rate.sleep()


    

if __name__ == '__main__':
    print('Hello World from main 1')


def main():
    print('Hello world from def main')
    z = SpotMicroSimpleCommand()
    print('Got here!!!')
    z.run()
