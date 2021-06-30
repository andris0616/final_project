#!/usr/bin/env python3

import odrive
from odrive.enums import *
import time
import math
import numpy as np
import rospy
from std_msgs.msg import Bool,Int32, Float32
#import matplotlib.pyplot as plt
#import numpy.core.mutiarry

pos_list =[]
time_list = []
vel_list = []
current_list = []
torque_list = []



if __name__ == '__main__':

	rospy.init_node('odrive_node')
	pub = rospy.Publisher('encoder_state', Float32, queue_size = 10)

	# connecting to odrive
	print("connecting to odrive")
	my_odrive = odrive.find_any()

	print(my_odrive.vbus_voltage)

	print("CLOSED LOOP CONTROL")
	my_odrive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

	my_odrive.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
	my_odrive.axis0.controller.input_vel = 1
	
	time = 0.0
	

	rate = rospy.Rate(10)
	
	forward = True
	backward = True
	
	

	while forward and not rospy.is_shutdown():
		
		enc = my_odrive.axis0.encoder.pos_estimate
		print(enc)
		pub.publish(enc)
		
		if enc > 5.0:
			my_odrive.axis0.controller.input_vel = 0
			forward = False
		
		time_list.append(time)
		pos_list.append(enc)
		vel_list.append(my_odrive.axis0.encoder.vel_estimate)
		current_list.append(my_odrive.axis0.motor.current_control.Iq_measured)
		#current_list.append(my_odrive.axis0.motor.current)
		torque_list.append(my_odrive.axis0.motor.current_control.Iq_measured * 8.27)
		#torque_list.append(my_odrive.axis0.motor.current * 8.27)
		time += 0.1
		
		rate.sleep()
		
	while backward and not rospy.is_shutdown():
		my_odrive.axis0.controller.input_vel = -1
		
		enc = my_odrive.axis0.encoder.pos_estimate
		print(enc)
		pub.publish(enc)
		
		pos_list.append(enc)
		time_list.append(time)
		vel_list.append(my_odrive.axis0.encoder.vel_estimate)
		current_list.append(my_odrive.axis0.motor.current_control.Iq_measured)
		#current_list.append(my_odrive.axis0.motor.current)
		torque_list.append(my_odrive.axis0.motor.current_control.Iq_measured * 8.27)
		#torque_list.append(my_odrive.axis0.motor.current * 8.27)
		time += 0.1
		
		if enc < 0:
			my_odrive.axis0.controller.input_vel = 0
			backward = False
		
		rate.sleep()
		
		
			

	my_odrive.axis0.controller.input_vel = 0
	
	my_odrive.axis0.requested_state = AXIS_STATE_IDLE
	
	#plt.plot(time_list, pos_list)
	#plt.show()
	print(time_list)
	print(pos_list)
	print(vel_list)
	print(torque_list)
	print(len(time_list))
	print(len(pos_list))
	
	rospy.is_shutdown()
	my_odrive = None
