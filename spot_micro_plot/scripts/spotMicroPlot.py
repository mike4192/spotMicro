#!/usr/bin/python

import numpy as np
import time

import rospy
from spot_micro_kinematics_python.spot_micro_stick_figure import SpotMicroStickFigure
from math import pi
from std_msgs.msg import Float32, Bool 
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation


#########################################################################
########################## Global Variables #############################
#########################################################################
num_servos = 12

r2d = 180/pi
d2r = pi/180


# Attaching 3D axis to the figure
fig = plt.figure()
ax = p3.Axes3D(fig)
ax.set_proj_type('ortho')

ax.set_xlabel('X')
ax.set_ylabel('Z')
ax.set_zlabel('Y')

ax.set_xlim3d([-0.2, 0.2])
ax.set_zlim3d([0, 0.4])
ax.set_ylim3d([-0.2,0.2])

x = 0

# def update_x_speed_cmd(msg):
#         '''Updates x speed command from received message'''
#         x = msg.data
#         print('here')

def update_lines(num, x, lines):
    for l in lines:
        msg = rospy.wait_for_message("/joint_angles", Float32MultiArray, timeout=None)
        print(len(joint_angles))
        print(msg.data)
        l.set_data([0, num/100.0], [0, num/100.0])
        print(num/100.0)
        l.set_3d_properties([0, num/100.0])

    return lines

# Set up and title the ros node for this code
rospy.init_node('spot_micro_plot') 

# rospy.Subscriber('x_speed_cmd',Float32,update_x_speed_cmd)

# Define the loop rate in Hz
# rate = rospy.Rate(2)

# cnt = 0
# while not rospy.is_shutdown():


lines = []
lines.append(ax.plot([0, 0.01], [0, 0.01], [0, 0.01])[0])




lines_ani = animation.FuncAnimation(fig, update_lines, frames=1000, fargs=(x,lines), interval=1000)

plt.show()





