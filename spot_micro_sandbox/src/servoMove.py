#!/usr/bin/python

"""
Class for testing cotnrol of 12 servos. It assumes ros-12cpwmboard has been
installed
"""
import rospy
from i2cpwm_board.msg import Servo, ServoArray
#from geometry_msgs.msg import Twist
import time


class ServoConvert():
    '''
    ServoConvert Class encapsulates a servo
    Servo has a center value, and range, and is commanded by a 1 to -1 value
    '''
    def __init__(self, id=1, center_value=333, range=500, direction=1):
        self.value      = 0.0
        self.value_out  = center_value
        self._center    = center_value
        self._range     = range
        self._half_range= 0.5*range
        self._dir       = direction
        self.id         = id

        #--- Convert its range in [-1, 1]
        self._sf        = 1.0/self._half_range

    def get_value_out(self, value_in):
        '''
        Input: Value between 1 and -1
        Output: integer servo command value
        '''
        self.value      = value_in
        self.value_out  = int(self._dir*value_in*self._half_range + self._center)
        print self.id, self.value_out
        return(self.value_out)

class SpotMicroServoControl():
    def __init__(self):
        rospy.loginfo("Setting Up the Spot Micro Servo Control Node...")

        rospy.init_node('spot_micro_servo_control')

        self.servos = {}
        self.servos[1]  = ServoConvert(id=1)
        rospy.loginfo("> Servos corrrectly initialized")

        self._servo_msg       = ServoArray()
        for i in range(1): self._servo_msg.servos.append(Servo())

        #--- Create the servo array publisher
        self.ros_pub_servo_array    = rospy.Publisher("/servos_absolute", ServoArray, queue_size=1)
        rospy.loginfo("> Publisher corrrectly initialized")

        # #--- Create the Subscriber to Twist commands
        # self.ros_sub_twist          = rospy.Subscriber("/cmd_vel", Twist, self.set_actuators_from_cmdvel)
        # rospy.loginfo("> Subscriber corrrectly initialized")

        rospy.loginfo("Initialization complete")

    # def set_actuators_from_cmdvel(self, message):
    #     """
    #     Get a message from cmd_vel, assuming a maximum input of 1
    #     """
    #     #-- Save the time
    #     self._last_time_cmd_rcv = time.time()

    #     #-- Convert vel into servo values
    #     self.actuators['throttle'].get_value_out(message.linear.x)
    #     self.actuators['steering'].get_value_out(message.angular.z)
    #     rospy.loginfo("Got a command v = %2.1f  s = %2.1f"%(message.linear.x, message.angular.z))
    #     self.send_servo_msg()

    def send_servo_msg(self):
        for servo_name, servo_obj in self.servos.iteritems():
            self._servo_msg.servos[servo_obj.id-1].servo = servo_obj.id
            self._servo_msg.servos[servo_obj.id-1].value = servo_obj.value_out
            rospy.loginfo("Sending to %s command %d"%(servo_name, servo_obj.value_out))

        self.ros_pub_servo_array.publish(self._servo_msg)

    def run(self):

        #--- Set the control rate in Hz
        rate = rospy.Rate(100)

        # Logic to make servo go back and forth indefinitely
        pos = 0
        dir = 1
        changeRate = 0.01
        while not rospy.is_shutdown():
            if dir == 1:
                pos += changeRate
                if pos >= 1:
                    pos = 1
                    dir = -1
            else:
                pos -= changeRate
                if pos <= -1:
                    pos = -1
                    dir = 1
            

            self.servos[1].get_value_out(pos)
            self.send_servo_msg()
            # print self._last_time_cmd_rcv, self.is_controller_connected
            # if not self.is_controller_connected:
            #     self.set_actuators_idle()
           

            rate.sleep()

if __name__ == "__main__":
    smsc     = SpotMicroServoControl()
    smsc.run()