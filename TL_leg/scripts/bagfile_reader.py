#!/usr/bin/env python3

import time
import math
import numpy as np
import rospy
from std_msgs.msg import Bool,Int32, Float32, Float64MultiArray 
import matplotlib.pyplot as plt
import rosbag
import rospkg
import bagpy
from bagpy import bagreader
from geometry_msgs.msg import Vector3


rospack = rospkg.RosPack()
path = rospack.get_path('map2_robot')

name = '2021-07-05-09-33-18'

bag = rosbag.Bag(path + '/bagfiles/' + str(name) + '.bag')

print(path + '/bagfiles/' + str(name) + '.bag')

leg_x = []
leg_y = []

print('hello')
for topic, msg, t in bag.read_messages(topics=['/leg_position']):
	leg_x.append(msg.x)
	leg_y.append(msg.y)
	
leg_vel_x = []
leg_vel_y = []

print('hello')
for topic, msg, t in bag.read_messages(topics=['/leg_position']):
	leg_x.append(msg.x)
	leg_y.append(msg.y)
	
	
print('after')
	
desired_x = [0.0, 0.5, 0.5]
desired_y = [0.0, 0.0, 0.5]

desired_pos_x_list = [1.0, 1.0, 0.0, 0.0] #[0.5, 0.5, 0.0, 0.0]
desired_pos_y_list = [0.0, 0.7, 0.7, 0.0] #[0.0, 0.5, 0.5, 0.0]

plt.plot(leg_x, leg_y, label = 'robot trajectory')
#plt.plot(desired_x, desired_y, marker = 'x', label = 'desired position')
plt.plot(desired_pos_x_list, desired_pos_y_list, marker = 'x', label = 'desired position')
plt.title("Robot trajectory based on the wheel encoders ")
plt.xlabel("x [m]")
plt.ylabel("y [m]")
#plt.axis("square")
plt.legend()
plt.show()
