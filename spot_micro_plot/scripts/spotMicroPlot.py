#!/usr/bin/python

import numpy as np
import time

import rospy
from spot_micro_kinematics_python.spot_micro_stick_figure import SpotMicroStickFigure
from spot_micro_kinematics_python.utilities import transformations
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
ax.set_facecolor('black')

ax.set_xlabel('X')
ax.set_ylabel('Z')
ax.set_zlabel('Y')

ax.set_xlim3d([-0.2, 0.2])
ax.set_zlim3d([0, 0.3])
ax.set_ylim3d([0.2,-0.2])

x = 0

# def update_x_speed_cmd(msg):
#         '''Updates x speed command from received message'''
#         x = msg.data
#         print('here')

def update_lines(num, x, lines):
    msg = rospy.wait_for_message("/body_state", Float32MultiArray, timeout=None)

    foot_data = np.array([ [msg.data[0], msg.data[1], msg.data[2]],
                           [msg.data[3], msg.data[4], msg.data[5]],
                           [msg.data[6], msg.data[7], msg.data[8]],
                           [msg.data[9], msg.data[10], msg.data[11]] ])
    
    xpos = msg.data[12]
    ypos = msg.data[13]
    zpos = msg.data[14]

    phi = msg.data[15]
    theta = msg.data[16]
    psi = msg.data[17]

    # print(foot_data)
    sm.set_absolute_foot_coordinates(foot_data)
    temp_rot = transformations.rotxyz(phi, psi, theta)
    temp_pose = np.identity(4)
    temp_pose[0:3, 0:3] = temp_rot
    temp_pose[0,3] = xpos
    temp_pose[1,3] = ypos
    temp_pose[2,3] = zpos

    sm.set_absolute_body_pose(temp_pose)

    # print(len(msg.data))
    # print(msg.data[0])
    # l.set_data([0, num/100.0], [0, num/100.0])
    # print(num/100.0)
    # l.set_3d_properties([0, num/100.0])

    # Get leg coordinates and append to data list
    coord_data = sm.get_leg_coordinates()

    line_to_leg__and_link_dict =   {4:(0,0),
                                    5:(0,1),
                                    6:(0,2),
                                    7:(1,0),
                                    8:(1,1),
                                    9:(1,2),
                                    10:(2,0),
                                    11:(2,1),
                                    12:(2,2),
                                    13:(3,0),
                                    14:(3,1),
                                    15:(3,2)}

    for line, i in zip(lines, range(len(lines))):

        line.set_linewidth(4)
        if i < 4:
            # First four lines are the square body
            if i == 3:
                ind = -1
            else:
                ind = i
            x_vals = [coord_data[ind][0][0], coord_data[ind+1][0][0]]
            y_vals = [coord_data[ind][0][1], coord_data[ind+1][0][1]]
            z_vals = [coord_data[ind][0][2], coord_data[ind+1][0][2]]
            # NOTE: there is no .set_data() for 3 dim data...
            line.set_data(x_vals,z_vals)
            line.set_3d_properties(y_vals)

    # Next 12 lines are legs
    # Leg 1, link 1, link 2, link 3
    # Leg 2, link 1, link 2, link 3...
        else:
            leg_num = line_to_leg__and_link_dict[i][0]
            link_num = line_to_leg__and_link_dict[i][1]
            x_vals = [coord_data[leg_num][link_num][0], coord_data[leg_num][link_num+1][0]]
            y_vals = [coord_data[leg_num][link_num][1], coord_data[leg_num][link_num+1][1]]
            z_vals = [coord_data[leg_num][link_num][2], coord_data[leg_num][link_num+1][2]]
            
            line.set_data(x_vals,z_vals)
            line.set_3d_properties(y_vals)
    return lines

# Set up and title the ros node for this code
rospy.init_node('spot_micro_plot') 

# Instantiate spot micro stick figure obeject
sm = SpotMicroStickFigure(x=0,y=0.093,z=0)

coords = sm.get_leg_coordinates()

# Initialize empty list top hold line objects
lines = []

# Construct the body of 4 lines from the first point of each leg (the four corners of the body)
for i in range(4):
    # For last leg, connect back to first leg point
    if i == 3:
        ind = -1
    else:
        ind = i

    # Due to mplot3d rotation and view limitations, swap y and z to make the stick figure
    # appear oriented better
    x_vals = [coords[ind][0][0], coords[ind+1][0][0]]
    y_vals = [coords[ind][0][1], coords[ind+1][0][1]]
    z_vals = [coords[ind][0][2], coords[ind+1][0][2]]
    lines.append(ax.plot(x_vals,z_vals,y_vals,color='k')[0])


# Plot color order for leg links: (hip, upper leg, lower leg)
plt_colors = ['r','c','b']
for leg in coords:
    for i in range(3):
        
        # Due to mplot3d rotation and view limitations, swap y and z to make the stick figure
        # appear oriented better
        x_vals = [leg[i][0], leg[i+1][0]]
        y_vals = [leg[i][1], leg[i+1][1]]
        z_vals = [leg[i][2], leg[i+1][2]]
        lines.append(ax.plot(x_vals,z_vals,y_vals,color=plt_colors[i])[0])




lines_ani = animation.FuncAnimation(fig, update_lines, frames=1000, fargs=(x,lines), interval=100)

plt.show()





