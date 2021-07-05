#!/usr/bin/env python3

import time
import math
import numpy as np
import rospy
import matplotlib.pyplot as plt
import rosbag
import rospkg
import bagpy
from bagpy import bagreader
from geometry_msgs.msg import Vector3
import pandas as pd
import seaborn as sea

rospack = rospkg.RosPack()
path = rospack.get_path('map2_robot')

name = '2021-07-05-11-37-48'


b = bagreader(path + '/bagfiles/' + str(name) + '.bag')

print(b.topic_table)

csvfile = []
for t in b.topics:
	data = b.message_by_topic(t)
	csvfile.append(data)
	
leg_position_csv = pd.read_csv(path + '/bagfiles/' + str(name) +'/leg_position.csv')
leg_velocity_csv = pd.read_csv(path + '/bagfiles/' + str(name) +'/leg_velocity.csv')
heading_angle_csv = pd.read_csv(path + '/bagfiles/' + str(name) +'/leg_heading_angle.csv')
wheel_torque_R_csv = pd.read_csv(path + '/bagfiles/' + str(name) +'/leg_torque_R.csv')
wheel_torque_L_csv = pd.read_csv(path + '/bagfiles/' + str(name) +'/leg_torque_L.csv')


print(heading_angle_csv)

# PLOTTING

desired_x = [0.0, 0.5, 0.5]
desired_y = [0.0, 0.0, 0.5]

desired_pos_x_list = [1.0, 1.0, 0.0, 0.0] #[0.5, 0.5, 0.0, 0.0]
desired_pos_y_list = [0.0, 0.7, 0.7, 0.0] #[0.0, 0.5, 0.5, 0.0]

plt.plot(leg_position_csv['x'], leg_position_csv['y'], label = 'robot trajectory')
#plt.plot(desired_x, desired_y, marker = 'x', label = 'desired position')
plt.plot(desired_pos_x_list, desired_pos_y_list, marker = 'x', label = 'desired position')
plt.title("Robot trajectory based on the wheel encoders ")
plt.xlabel("x [m]")
plt.ylabel("y [m]")
#plt.axis("square")
plt.legend()
plt.show()



fig, ax = bagpy.create_fig(7)

ax[0].scatter(x = 'Time', y = 'x', data = leg_position_csv, s = 1, label = 'position x [m]')
ax[1].scatter(x = 'Time', y = 'y', data = leg_position_csv, s = 1, label = 'position y [m]')
ax[2].scatter(x = 'Time', y = 'x', data = leg_velocity_csv, s = 1, label = 'velocity x [m/s]')
ax[3].scatter(x = 'Time', y = 'y', data = leg_velocity_csv, s = 1, label = 'velocity y [m/s]')
ax[4].scatter(x = 'Time', y = 'data', data = heading_angle_csv, s = 1, label = 'heading angle in randian')
ax[5].scatter(x = 'Time', y = 'data', data = wheel_torque_L_csv, s = 1, label = 'left wheel torque [Nm]')
ax[6].scatter(x = 'Time', y = 'data', data = wheel_torque_R_csv, s = 1, label = 'right_wheel_troque [Nm]')

for axis in ax:
	axis.legend()
axis.set_xlabel('Time [s]')

plt.show()





