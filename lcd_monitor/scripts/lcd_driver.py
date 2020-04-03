#!/usr/bin/python


from lcd_monitor import I2C_LCD_driver
import datetime
import time
import rospy






if __name__ == "__main__":
	mylcd = I2C_LCD_driver.lcd()

	
	rospy.init_node('lcd_monitor_node')
	rate = rospy.Rate(1)	

	while not rospy.is_shutdown():
		hour = datetime.datetime.now().hour
		minute = datetime.datetime.now().minute
		second = datetime.datetime.now().second
		#mylcd.lcd_clear()
		mylcd.lcd_display_string('Hi Maggie!! :)',1)
		mylcd.lcd_display_string("%2i:%2i:%2i"%(hour,minute,second), 2)
		rate.sleep()


