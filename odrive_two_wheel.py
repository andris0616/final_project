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


if __name__ == '__main__':

	rospy.init_node('odrive_node')

	my_odrive = Odrive_class()
	
	print("Odrive is connected!")

	desired_pos = 0.5

	#my_odrive.straight_position_control(desired_pos)
	
	
	desired_x = 0.2
	desired_y = 0.0
	
	desired_pos_x_list = [1.0, 1.0, 0.0, 0.0] #[0.25, 0.5, 0.50, 0.5, 0,25, 0.0, 0.00, 0.0]
	desired_pos_y_list = [0.0, 0.7, 0.7, 0.0] #[0.00, 0.0, 0.25, 0.5, 0,50, 0.5, 0.25, 0.0]
	
	for i in range(len(desired_pos_x_list)):
		my_odrive.diff_drive_control(desired_pos_x_list[i], desired_pos_y_list[i])
		time.sleep(2)
	
	#my_odrive.diff_drive_control(desired_x, desired_y)
	
	########
	#my_odrive.pos_control(1.0)
	#my_odrive.angular_pos_control(90)
	#my_odrive.pos_control(1.0)
	#my_odrive.angular_pos_control(90)
	#my_odrive.pos_control(1.0)
	#my_odrive.angular_pos_control(90)
	#my_odrive.pos_control(1.0)
	
	
	#######
	
	#my_odrive.pos_control(desired_pos)

	desired_angle = 360
	
	#my_odrive.angular_pos_control(desired_angle)

	print(my_odrive.getEncoder0())
	print(my_odrive.getEncoder1())
	
	#while not rospy.is_shutdown():
		#print(my_odrive.heading_angle)
	
	# writing to bag file:
	"""bag = rosbag.Bag('robot_data.bag', 'w')
	
	# NEED TO CONVERT THE LIST TO FLOAT64MultiArray
	bag.write('position_x', my_odrive.pos_x_list)
	bag.write('position_y', my_odrive.pos_y_list)
	
	bag.close()"""		
	
	file1 = open("pos_data_x.txt", "w")
	for x in  my_odrive.pos_x_list:
		file1.write(str(x) + "\n")
	file1.close
		
	file2 = open("pos_data_y.txt", "w")
	for y in  my_odrive.pos_y_list:
		file2.write(str(y) + "\n")
		
	file2.close
	
	
	my_odrive.reboot()


