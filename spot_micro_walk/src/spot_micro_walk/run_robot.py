import numpy as np
import time
from pupper_src.State import State
from pupper_src.Command import Command
from pupper_src.Controller import Controller
from pupper.Config import Configuration
#from pupper.Kinematics import four_legs_inverse_kinematics

import rospy
from i2cpwm_board.msg import Servo, ServoArray, ServoConfig, ServoConfigArray
from i2cpwm_board.srv import ServosConfig
from spot_micro_simple_command.spot_micro_kinematics.spot_micro_stick_figure import SpotMicroStickFigure
from spot_micro_simple_command.first_order_filter.fof import FirstOrderFilter 
from math import pi
from std_msgs.msg import Float32

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
    '''Class to encapsulate walk command of spot micro robot from external commands'''

    def __init__(self):
        '''Constructor'''
        # Create speed and body rate command data variables
        self.x_speed_cmd_mps = 0
        self.y_speed_cmd_mps = 0
        self.yaw_rate_cmd_rps = 0 

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
        rospy.init_node('spot_micro_walk') 

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

        # Create subsribers for speed and body rate command topics, both using vector3
        rospy.Subscriber('/x_speed_cmd',Float32,self.update_x_speed_cmd)
        rospy.Subscriber('/y_speed_cmd',Float32,self.update_y_speed_cmd)
        rospy.Subscriber('/yaw_rate_cmd',Float32,self.update_yaw_rate_cmd)


        rospy.loginfo("Initialization complete")

        # Create a spot micro stick figure object to encapsulate robot state
        default_height = 0.18
        self.sm = SpotMicroStickFigure(y=default_height)

        # Set absolute foot positions for default stance,
        # foot order: RB, RF, LF, LB
        l = self.sm.body_length
        w = self.sm.body_width
        l1 = self.sm.hip_length
        desired_p4_points = np.array([ [-l/2,   0,  w/2 + l1],
                                    [ l/2 ,  0,  w/2 + l1],
                                    [ l/2 ,  0, -w/2 - l1],
                                    [-l/2 ,  0, -w/2 - l1] ])

        self.sm.set_absolute_foot_coordinates(desired_p4_points)

        # Create configuration object and update values to reflect spot micro configuration
        self.config = Configuration()
        self.config.delta_x = l/2
        self.config.delta_y = w/2 + l1
        self.default_z_ref = -default_height
        
        # Create controller object
        self.controller = Controller(self.config)

        # Create state object
        self.state = State()
        self.state.foot_locations = (self.config.default_stance + np.array([0,0,-default_height])[:, np.newaxis])
        
        # Create Command object
        self.command = Command()
        self.command.height = -default_height
   


    def update_x_speed_cmd(self,msg):
        '''Updates x speed command from received message'''
        self.x_speed_cmd_mps = msg.data

    def update_y_speed_cmd(self,msg):
        '''Updates y speed command from received message'''
        self.x_speed_cmd_mps = msg.data

    def update_yaw_rate_cmd(self,msg):
        '''Updates yaw rate command from received message'''
        self.yaw_rate_cmd_rp = msg.data

    def set_leg_angles_servo_msg(self,leg_angs):
        '''Sets servo_cmds_rad to the set of 12 leg angles received from get_leg_angles'''
        
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

        print(self.config.default_stance)
        state = State()
        state.foot_locations = (
                    self.config.default_stance + np.array([0, 0, -0.16])[:, np.newaxis]
                )
        # Define the loop rate in Hz
        rate = rospy.Rate(50)

        # Instantiate a first order filter object
        fof = FirstOrderFilter(0.02,tau=1,x0=0)

        # Command neutral body pose
        self.sm.set_body_angles(theta=0,phi=0,psi=0)

        # Get leg angles
        leg_angs = self.sm.get_leg_angles()
        
        # Send command to all servos for initial stance
        self.send_servo_cmd_msg()
         
        while not rospy.is_shutdown():

            #Call gait or stand controller
            foot_positions = self.controller.run(self.state, self.command)

            # Command Servos
            self.send_servo_cmd_msg()


            # Sleep till next loop
            rate.sleep()
    

def main():
    print('Hello world from def main')
    smsc_obj = SpotMicroSimpleCommand()
    print('Got here!!!')
    smsc_obj.run()

def main(use_imu=False):
    """Main program
    """

    # Create config
    config = Configuration()

    # Create controller and user input handles
    controller = Controller(config)


    print(config.default_stance)
    state = State()
    state.foot_locations = (
                config.default_stance + np.array([0, 0, -0.16])[:, np.newaxis]
            )
    # print("Creating joystick listener...")
    # joystick_interface = JoystickInterface(config)
    # print("Done.")

    last_loop = time.time()

    print("Summary of gait parameters:")
    # print("overlap time: ", config.overlap_time)
    # print("swing time: ", config.swing_time)
    # print("z clearance: ", config.z_clearance)
    # print("x shift: ", config.x_shift)


    command = Command()
    command.horizontal_velocity = np.array([0.2, 0])

    foot_position_data = []
    # Wait until the activate button has been pressed
    # while True:
        # print("Waiting for L1 to activate robot.")
        # while True:
        #     command = joystick_interface.get_command(state)
        #     joystick_interface.set_color(config.ps4_deactivated_color)
        #     if command.activate_event == 1:
        #         break
        #     time.sleep(0.1)
        # print("Robot activated.")
        # joystick_interface.set_color(config.ps4_color)
    count = -1
    
    while count<101:
        
        now = time.time()
        if now - last_loop < config.dt:
            continue
        else:
            count += 1
            last_loop = time.time()

            # # Parse the udp joystick commands and then update the robot controller's parameters
            # command = joystick_interface.get_command(state)
            # if command.activate_event == 1:
            #     print("Deactivating Robot")
            #     break

            # Read imu data. Orientation will be None if no data was available
            # quat_orientation = (
            #     imu.read_orientation() if use_imu else np.array([1, 0, 0, 0])
            # )
            # state.quat_orientation = quat_orientation


            # Step the controller forward by dt
            
            test = controller.run(state, command)
            # print(test)
            if count>50:
                foot_position_data.append(test)





        # # Update the pwm widths going to the servos
        # hardware_interface.set_actuator_postions(state.joint_angles)


    fig, ax = plt.subplots(2,2)

    # Reorganize data
    f1xpos = []
    f1zpos = []

    f2xpos = []
    f2zpos = []

    f3xpos = []
    f3zpos = []

    f4xpos = []
    f4zpos = []

    for d in foot_position_data:
        f1xpos.append(d[0][0])
        f1zpos.append(d[2][0])

        f2xpos.append(d[0][1])
        f2zpos.append(d[2][1])

        f3xpos.append(d[0][2])
        f3zpos.append(d[2][2])
        
        f4xpos.append(d[0][3])
        f4zpos.append(d[2][3])


    COLOR='blue'
    MAP='winter'
    NPOINTS = len(f1xpos)

    cm = plt.get_cmap(MAP)
    colors = plt.cm.winter(np.linspace(0,1,NPOINTS))
    ax[0,1].set_prop_cycle('color',colors)
    for i in range(NPOINTS-1):
        ax[0,1].plot(f1xpos[i:i+2],f1zpos[i:i+2])

    ax[0,0].set_prop_cycle('color',colors)
    for i in range(NPOINTS-1):
        ax[0,0].plot(f2xpos[i:i+2],f2zpos[i:i+2])
    
    ax[1,1].set_prop_cycle('color',colors)
    for i in range(NPOINTS-1):
        ax[1,1].plot(f3xpos[i:i+2],f3zpos[i:i+2])


    ax[1,0].set_prop_cycle('color',colors)
    for i in range(NPOINTS-1):
        ax[1,0].plot(f4xpos[i:i+2],f4zpos[i:i+2])

    # Plot foot default stance points as red dots
    ax[0,1].plot(config.default_stance[0,0],config.default_z_ref,'ro')
    ax[0,0].plot(config.default_stance[0,1],config.default_z_ref,'ro')
    ax[1,1].plot(config.default_stance[0,2],config.default_z_ref,'ro')
    ax[1,0].plot(config.default_stance[0,3],config.default_z_ref,'ro')

    # Set axis labels
    for i in range(2):
        for j in range(2):
            ax[i,j].set_ylabel('z position')
            ax[i,j].set_xlabel('x position')
    
    # Set titles
    ax[0,1].set_title('Front Right Foot')
    ax[0,0].set_title('Front Left Foot')
    ax[1,0].set_title('Back Left Foot')
    ax[1,1].set_title('Back Right Foot')

    ax[0,0].axis('scaled')
    ax[0,1].axis('scaled')
    ax[1,0].axis('scaled')
    ax[1,1].axis('scaled')
    
    # Set title
    fig.suptitle('Pupper Trot Foot Positions, Forward Speed 0.2 m/s, alpha = 0.8',fontsize=16)
    plt.show()
    

main()
