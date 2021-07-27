#!/usr/bin/env python3

import odrive
from odrive.enums import *
import time
import math
import numpy as np
import rospy
from std_msgs.msg import Bool,Int32, Float32, Float64MultiArray
from Odrive_class import Odrive_class 
import matplotlib.pyplot as plt
import rosbag
from geometry_msgs.msg import Vector3


if __name__ == '__main__':

	rospy.init_node('TL_leg_node')
	

	my_odrive = Odrive_class()
	
	print("Odrive is connected!")

	while not rospy.is_shutdown():
		
		if my_odrive.new_angle:
			my_odrive.new_angle = False
			
			print("I'm here!!!!!!!!!")
			
			my_odrive.heading_angle_control(my_odrive.desired_h_angle)
			
		if my_odrive.new_position:
			my_odrive.new_position = False
			
			my_odrive.diff_drive_control(my_odrive.desired_position.x, my_odrive.desired_position.y)
	
	
	my_odrive.reboot()


