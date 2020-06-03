
from lcd_monitor import I2C_LCD_driver
import datetime
import time
import rospy
from geometry_msgs.msg import Vector3
from std_msgs.msg import String

class SpotMicroLcd():
	''' Class to encapsulate lcd driver for spot micro robot '''

	def __init__(self):
		'''Constructor'''
		self._mylcd = I2C_LCD_driver.lcd()

		self._state_str = 'None'

		self._fwd_speed_cmd = 0.0
		self._side_speed_cmd = 0.0
		self._yaw_rate_cmd = 0.0

		self._phi_cmd = 0.0
		self._theta_cmd = 0.0
		self._psi_cmd = 0.0

		rospy.init_node('lcd_monitor_node')

		rospy.Subscriber('sm_speed_cmd',Vector3,self.update_speed_cmd)
		rospy.Subscriber('sm_angle_cmd',Vector3,self.update_angle_cmd)
		rospy.Subscriber('sm_state',String,self.update_state_string)

	def update_speed_cmd(self, msg):
		''' Updates speed command attributes'''
		self._fwd_speed_cmd = msg.x
		self._side_speed_cmd = msg.z
		self._yaw_rate_cmd = msg.y

	def update_angle_cmd(self, msg):
		''' Updates angle command attributes'''
		self._phi_cmd = msg.x
		self._theta_cmd = msg.y
		self._psi_cmd = msg.z

	def update_state_string(self, msg):
		''' Updates angle command attributes'''
		self._state_str = msg.data

	def run(self):
		''' Runs the lcd driver and prints data'''

		# Define the loop rate in Hz
		rate = rospy.Rate(1)

		while not rospy.is_shutdown():

			self._mylcd.lcd_display_string('State: %s'%(self._state_str),1)
			# mylcd.lcd_display_string("%2i:%2i:%2i"%(hour,minute,second), 2)

			# Sleep till next loop
            rate.sleep()







def main():
	sm_lcd_obj = SpotMicroLcd()
	sm_lcd_obj.run()
