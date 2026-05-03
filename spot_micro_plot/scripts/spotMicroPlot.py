#!/usr/bin/env python3

import numpy as np
import time
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation

# Local kinematics module (pure Python, no ROS deps)
from spot_micro_kinematics_python.spot_micro_stick_figure import SpotMicroStickFigure
from spot_micro_kinematics_python.utilities import transformations
from math import pi

num_servos = 12
r2d = 180/pi
d2r = pi/180

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

latest_msg = None
msg_lock = threading.Lock()

class BodyStateSubscriber(Node):
    def __init__(self):
        super().__init__('spot_micro_plot')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/body_state',
            self.listener_callback,
            1)

    def listener_callback(self, msg):
        global latest_msg
        with msg_lock:
            latest_msg = msg


def update_lines(num, x, lines):
    global latest_msg
    with msg_lock:
        msg = latest_msg

    if msg is None:
        return lines

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

    sm.set_absolute_foot_coordinates(foot_data)
    temp_rot = transformations.rotxyz(phi, psi, theta)
    temp_pose = np.identity(4)
    temp_pose[0:3, 0:3] = temp_rot
    temp_pose[0,3] = xpos
    temp_pose[1,3] = ypos
    temp_pose[2,3] = zpos

    sm.set_absolute_body_pose(temp_pose)

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
            if i == 3:
                ind = -1
            else:
                ind = i
            x_vals = [coord_data[ind][0][0], coord_data[ind+1][0][0]]
            y_vals = [coord_data[ind][0][1], coord_data[ind+1][0][1]]
            z_vals = [coord_data[ind][0][2], coord_data[ind+1][0][2]]
            line.set_data(x_vals,z_vals)
            line.set_3d_properties(y_vals)

        else:
            leg_num = line_to_leg__and_link_dict[i][0]
            link_num = line_to_leg__and_link_dict[i][1]
            x_vals = [coord_data[leg_num][link_num][0], coord_data[leg_num][link_num+1][0]]
            y_vals = [coord_data[leg_num][link_num][1], coord_data[leg_num][link_num+1][1]]
            z_vals = [coord_data[leg_num][link_num][2], coord_data[leg_num][link_num+1][2]]

            line.set_data(x_vals,z_vals)
            line.set_3d_properties(y_vals)
    return lines


rclpy.init()
node = BodyStateSubscriber()

# Spin the node in a background thread so matplotlib can run on the main thread
spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
spin_thread.start()

sm = SpotMicroStickFigure(x=0,y=0.093,z=0)

coords = sm.get_leg_coordinates()

lines = []

for i in range(4):
    if i == 3:
        ind = -1
    else:
        ind = i

    x_vals = [coords[ind][0][0], coords[ind+1][0][0]]
    y_vals = [coords[ind][0][1], coords[ind+1][0][1]]
    z_vals = [coords[ind][0][2], coords[ind+1][0][2]]
    lines.append(ax.plot(x_vals,z_vals,y_vals,color='k')[0])

plt_colors = ['r','c','b']
for leg in coords:
    for i in range(3):
        x_vals = [leg[i][0], leg[i+1][0]]
        y_vals = [leg[i][1], leg[i+1][1]]
        z_vals = [leg[i][2], leg[i+1][2]]
        lines.append(ax.plot(x_vals,z_vals,y_vals,color=plt_colors[i])[0])

lines_ani = animation.FuncAnimation(fig, update_lines, frames=1000, fargs=(x,lines), interval=100)

plt.show()

node.destroy_node()
rclpy.shutdown()
