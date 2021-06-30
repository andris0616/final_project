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

def velocity_demo():
	my_odrive.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
	my_odrive.axis0.controller.input_vel = 1
	
	forward = True
	backward = True
	
	

	while forward:
		
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
		torque_list.append(my_odrive.axis0.motor.current_control.Iq_measured * 8.27)
		time += hz
		
		rate.sleep()
		
	while backward:
		my_odrive.axis0.controller.input_vel = -1
		
		enc = my_odrive.axis0.encoder.pos_estimate
		print(enc)
		pub.publish(enc)
		
		pos_list.append(enc)
		time_list.append(time)
		vel_list.append(my_odrive.axis0.encoder.vel_estimate)
		current_list.append(my_odrive.axis0.motor.current_control.Iq_measured)
		torque_list.append(my_odrive.axis0.motor.current_control.Iq_measured * 8.27)
		time += hz
		
		if enc < 0:
			my_odrive.axis0.controller.input_vel = 0
			backward = False
		
		rate.sleep()		

	my_odrive.axis0.controller.input_vel = 0
	
	#plt.plot(time_list, pos_list)
	#plt.show()
	print(time_list)
	print(pos_list)
	print(len(time_list))
	print(len(pos_list))
	
	
	
def straight_position_control(desired_pos):
		
		starting_pos = my_odrive.axis0.encoder.pos_estimate
		my_odrive.axis0.controller.input_pos = starting_pos
	
		my_odrive.axis0.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
	
		circumference = 2 * 0.1 * np.pi
		
		#convert desired pos to number of rotations
		rotations = desired_pos / circumference
		
		my_odrive.axis0.controller.pos_setpoint = rotations
		
		while my_odrive.axis0.encoder.vel_estimate > 0.01:
		#while my_odrive.axis0.encoder.pos_estimate < rotations - 0.01:
			pos_list.append(my_odrive.axis0.encoder.pos_estimate - starting_pos)
			time_list.append(time)
			vel_list.append(my_odrive.axis0.encoder.vel_estimate)
			current_list.append(my_odrive.axis0.motor.current_control.Iq_measured)
			torque_list.append(my_odrive.axis0.motor.current_control.Iq_measured * 8.27)
			time += hz
			
	
	

if __name__ == '__main__':

	rospy.init_node('odrive_node')
	pub = rospy.Publisher('encoder_state', Float32, queue_size = 10)

	# connecting to odrive
	print("connecting to odrive")
	my_odrive = odrive.find_any()

	print(my_odrive.vbus_voltage)

	print("CLOSED LOOP CONTROL")
	my_odrive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

	time = 0.0
	
	hz = 100

	rate = rospy.Rate(hz)
	
	
	
	
	
	if rospy.is_shutdown():
		break
