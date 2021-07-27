#!/usr/bin/env python3


import time
import math
import numpy as np
import rospy
from std_msgs.msg import Bool,Int32, Float32, Float64MultiArray
import matplotlib.pyplot as plt
import rosbag
from geometry_msgs.msg import Vector3


def desired_angle_callback(msg):
		global desired_angle
		global new_angle
		desired_h_angle = msg.data
		new_angle = True
		print("inside the angle callback")

def desired_position_callback(msg):
		global desired_position
		global new_position
		desired_position.x = msg.x
		desired_position.y = msg.y
		desired_position.z = msg.z
		print(desired_position)
		new_position = True
		print("inside the position callback")


# desired position and angle
desired_position = Vector3()
desired_position.x = 0
desired_position.y = 0
desired_position.z = 0

desired_h_angle = 0

# new data indicators
new_angle = False
new_position = False

# subscribers
rospy.Subscriber('desired_angle', Float32, desired_angle_callback)
rospy.Subscriber('desired_position', Vector3, desired_position_callback)

if __name__ == '__main__':

	rospy.init_node('test_node')
	
	print("running")
	
	rate = rospy.Rate(1000)
	
	while not rospy.is_shutdown():
		
		
		if new_angle:
			new_angle = False
			
			print("I'm here!!!!!!!!!")
			
			print(desired_h_angle)
			
		if new_position:
			new_position = False
			
			print(desired_position.x, desired_position.y)
			
		rate.sleep()
