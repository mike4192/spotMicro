
from lcd_monitor import I2C_LCD_driver
import datetime
import time
import rclpy
from rclpy.node import Node
from math import pi
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class SpotMicroLcd(Node):
	''' Class to encapsulate lcd driver for spot micro robot '''

	def __init__(self):
		super().__init__('lcd_monitor_node')

		self._mylcd = I2C_LCD_driver.lcd()

		self._state_str = 'None'
		self._padded_state_str = 'None'

		self._fwd_speed_cmd = 0.0
		self._side_speed_cmd = 0.0
		self._yaw_rate_cmd = 0.0

		self._phi_cmd = 0.0
		self._theta_cmd = 0.0
		self._psi_cmd = 0.0

		self.create_subscription(Twist, 'lcd_vel_cmd', self.update_speed_cmd, 1)
		self.create_subscription(Vector3, 'lcd_angle_cmd', self.update_angle_cmd, 1)
		self.create_subscription(String, 'lcd_state', self.update_state_string, 1)

		self._timer = self.create_timer(1.0/3.0, self.run_lcd)

	def update_speed_cmd(self, msg):
		''' Updates speed command attributes'''
		self._fwd_speed_cmd = msg.linear.x*100.0
		self._side_speed_cmd = msg.linear.y*100.0
		self._yaw_rate_cmd = msg.angular.z*180.0/pi

	def update_angle_cmd(self, msg):
		''' Updates angle command attributes'''
		self._phi_cmd = msg.x * 180.0/pi
		self._theta_cmd = msg.y * 180.0/pi
		self._psi_cmd = msg.z * 180.0/pi

	def update_state_string(self, msg):
		''' Updates angle command attributes'''
		self._state_str = msg.data

		if self._state_str == "Transit Stand":
			self._padded_state_str = "To Stand"
		elif self._state_str == "Transit Idle":
			self._padded_state_str = "To Idle"
		else:
			self._padded_state_str = self._state_str

		self._padded_state_str = self._padded_state_str.ljust(16,' ')

	def run_lcd(self):
		''' Runs the lcd driver and prints data'''
		self._mylcd.lcd_display_string('State: %s'%(self._padded_state_str),1)

		if self._state_str == "Stand":
			self._mylcd.lcd_display_string('x%3.0f y%3.0f z%3.0f'%(self._phi_cmd, self._theta_cmd, self._psi_cmd),2)
		elif self._state_str == "Walk":
			self._mylcd.lcd_display_string('x%3.0f y%3.0f z%3.0f'%(self._fwd_speed_cmd, self._side_speed_cmd, self._yaw_rate_cmd),2)
		else:
			self._mylcd.lcd_display_string('                ',2)


def main(args=None):
	rclpy.init(args=args)
	sm_lcd_obj = SpotMicroLcd()
	rclpy.spin(sm_lcd_obj)
	sm_lcd_obj.destroy_node()
	rclpy.shutdown()
