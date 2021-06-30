#!/usr/bin/env python3

import odrive
from odrive.enums import *
import time
import math
import numpy as np
import rospy
from std_msgs.msg import Bool,Int32, Float32
from Odrive_class import Odrive_class 


if __name__ == '__main__':

	rospy.init_node('odrive_node')

	my_odrive = Odrive_class()
	
	print("Odrive is connected!")

	desired_pos = 10

	#my_odrive.straight_position_control(desired_pos)
	
	
	desired_x = 5
	desired_y = 0.5
	my_odrive.diff_drive_control(desired_x, desired_y)
	
	#my_odrive.pos_control(desired_pos)

	desired_angle = 300
	#my_odrive.angular_pos_control(desired_angle)

	print(my_odrive.getEncoder0())
	print(my_odrive.getEncoder1())
	
	my_odrive.reboot()


