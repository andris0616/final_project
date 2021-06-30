#!/usr/bin/env python3

"""
Odrive class
"""

from __future__ import print_function

import odrive
from odrive.enums import *
import time
import math
import numpy as np
import rospy


class Odrive_class(object):
	
	
	
	def __init__(self):
		
		# radius of the wheels
		self.radius = 0.1
		# distnace between the two wheels
		self.wheel_dist = 0.39
		# wheel circumference
		self.circumference = 2 * self.radius * np.pi
		# hall sensor ticks per revolution
		self.ticks = 90
		# current x position
		self.current_x = 0
		self.current_y = 0

		
		# connecting to odrive
		print("connecting to odrive")
		self.my_odrive = odrive.find_any()
		print("Connected to odrive")
		
		# Calibrating the Odrive
		#self.calibrate()
		
		print("odrive is ready to use")
		
		
	def calibrate(self):
		
		# calibrating the motors
		print("Motor calibration...")
		
		# motor0 calibration
		#elf.my_odrive.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
		
		time.sleep(3)
		
		while self.my_odrive.axis0.current_state != AXIS_STATE_IDLE:
			time.sleep(0.1)
			
		print("motor0 is calibrated")
		
		self.my_odrive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
		
		# motor1 calibration
		self.my_odrive.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
		
		time.sleep(3)
		
		while self.my_odrive.axis1.current_state != AXIS_STATE_IDLE:
			time.sleep(0.1)
			
		print("motor1 is calibrated")
		
		self.my_odrive.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
		
	def reboot(self):
		self.my_odrive.reboot()


	def getEncoder0(self):
		enc0 = self.my_odrive.axis0.encoder.pos_estimate
		return enc0
		
		
	def getEncoder1(self):
		enc1 = -self.my_odrive.axis1.encoder.pos_estimate
		return enc1
		

	def setVelocity_control_mode(self):
		self.my_odrive.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
		self.my_odrive.axis1.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
		
	def setPosition_control_mode(self):
		starting_pos0 = self.my_odrive.axis0.encoder.pos_estimate
		self.my_odrive.axis0.controller.input_pos = starting_pos0
		starting_pos1 = self.my_odrive.axis1.encoder.pos_estimate
		self.my_odrive.axis1.controller.input_pos = starting_pos1
		
		self.my_odrive.axis0.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
		self.my_odrive.axis1.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL


	def setVel0(self, vel):
		""" Input vel is in turn/second"""
		self.my_odrive.axis0.controller.input_vel = vel
		

	def setVel1(self, vel):
		""" Input vel is in turn/second"""
		self.my_odrive.axis1.controller.input_vel = -vel
			
			
	def straight_position_control(self, desired_pos):
		
		self.setPosition_control_mode()
		
		#convert desired pos to number of rotations
		rotations = desired_pos / self.circumference
		
		self.my_odrive.axis0.controller.input_pos = rotations
		self.my_odrive.axis1.controller.input_pos = rotations
		
		
	def straight_position_control_test(self, desired_pos):
		
		setPosition_control_mode()
		
		#convert desired pos to number of rotations
		rotations = desired_pos / self.circumference
		
		my_odrive.axis0.controller.pos_setpoint = rotations
		my_odrive.axis1.controller.pos_setpoint = rotations
		
		
	def angular_pos_control(self, desired_angle):
		# set control mode to velocity control
		self.setVelocity_control_mode()
		
		#calculate the arc the wheels need to rotate to get to the desired angle
		ds = desired_angle / 360 * self.wheel_dist *np.pi
		
		#convert desired pos to number of rotations
		rotations = ds / self.circumference
		
		
		# PID params
		Kp = 2
		Ki = 0.0
		Kd = 0.0
		
		epsilon = 0.01
		
		e_prev = 0
		ei = 0
		dt = 0.01
		
		
		while not rospy.is_shutdown() and abs(self.getEncoder0() - rotations) > epsilon:
			
			print(self.getEncoder0())
			# error
			e = rotations - self.getEncoder0()
			ed = (e - e_prev) / dt
			ei = ei + e*dt
			
			
			# PID equation
			vel = Kp*e + Ki* ei + Kd * ed
			
			
			# limit the input velocities
			vel = max(min(vel,0.3),-0.3)
			print(vel)
			
			# set the new input velocities
			self.setVel0(vel)
			self.setVel1(-vel)
			
			e_prev = e
			
			time.sleep(0.01)
			
		# STOP
		self.setVel0(0)
		self.setVel1(0)	
		print("Finished")
		print(self.getEncoder0())
		print(self.getEncoder1())
		print("Desired pos: {}".format(rotations))
		
		
		
		
	
	
		
	def pos_control(self, desired_pos):
		
		# set control mode to velocity control
		self.setVelocity_control_mode()
		
		#convert desired pos to number of rotations
		rotations = desired_pos / self.circumference
		
		# PID params
		Kp = 2
		Ki = 0.0
		Kd = 0.0
		
		epsilon = 0.01
		
		e_prev = 0
		ei = 0
		dt = 0.1
		
		# START
		self.setVel0(0.5)
		self.setVel1(0.5)	
		
		while not rospy.is_shutdown() and abs(self.getEncoder0() - rotations) > epsilon:
			
			print(self.getEncoder0())
			# error
			e = rotations - self.getEncoder0()
			ed = (e - e_prev) / dt
			ei = ei + e*dt
			
				
			
			# PID equation
			vel = Kp*e + Ki* ei + Kd * ed
			print(vel)
			
			
			# limit the input velocities
			vel = max(min(vel,1),-1)
			
			
			# set the new input velocities
			self.setVel0(vel)
			self.setVel1(vel)
			
			e_prev = e
			
			time.sleep(0.1)
			
		# STOP
		self.setVel0(0)
		self.setVel1(0)	
		print("Finished")
		print(self.getEncoder0())
		print(self.getEncoder1())
		print("Desired pos: {}".format(rotations))
			
		
		

	def diff_drive_control(self, desired_x, desired_y):
		
		# set control mode to velocity control
		self.setVelocity_control_mode()
		
		# PID params
		Kp = 4
		Ki = 0.0
		Kd = 4
		
		epsilon = 0.1
		
		current_x = 0
		current_y = 0
		fi = 0
		
		enc0_prev = 0
		enc1_prev = 0
		
		e_prev = 0
		ei = 0
		dt = 0.1
		
		# starting speed
		vel = 0.5
		
		self.setVel0(vel)
		self.setVel1(vel)
		
		dx = desired_x - current_x
		dy = desired_y - current_y
		
		while epsilon < np.sqrt((dx)**2 + (dy)**2) and not rospy.is_shutdown():
			
			enc0 = self.getEncoder0()
			enc1 = self.getEncoder1()
			# distance the left wheel travelled
			dL = (enc0 - enc0_prev) * self.circumference #/ self.ticks
			# distance the right wheel travelled
			dR = (enc1 - enc1_prev) * self.circumference #/ self.ticks
			
			# angle of the leg from the hall sensors (this should be changed later to the leg angle necoder data I guess)
			fi += 1/self.wheel_dist * (dR - dL)
			
			print(fi)
			
			
			# current position calculated from the hall data 
			current_x = current_x + 1/2 * (dR + dL) * np.cos(fi)
			current_y += 1/2 * (dR + dL) * np.sin(fi)
			
			# current position difference
			dx = desired_x - current_x
			dy = desired_y - current_y
			
			
			print("dx: {} m".format(dx))
			print("dy: {} m".format(dy))
			
			# error
			e = math.atan2(dy, dx)
			ed = (e - e_prev) / dt
			ei = ei + e*dt
			
			# PID equation
			fi_dot = Kp*e + Ki* ei + Kd * ed
			
			vel0 = vel - 1/2 * self.wheel_dist * fi_dot
			vel1 = vel + 1/2 * self.wheel_dist * fi_dot
			
			if np.sqrt((dx)**2 + (dy)**2) < 0.05:
				vel0 = vel0 * 0.1
				vel1 = vel1 * 0.1
				
			elif np.sqrt((dx)**2 + (dy)**2) < 0.1:
				vel0 = vel0 * 0.5
				vel1 = vel1 * 0.5
				
			
			# limit the input velocities
			vel0 = max(min(vel0,1),-1)
			vel1 = max(min(vel1,1),-1)
			
			print("right wheel: {}".format(vel0))
			print("left wheel: {}".format(vel1))
			
			# set the new input velocities
			self.setVel0(vel0)
			self.setVel1(vel1)
			
			#Update
			e_prev = e
			enc0_prev = enc0
			enc1_prev = enc1
  			
			time.sleep(0.1)
			
		
		# STOP
		self.setVel0(0)
		self.setVel1(0)	
				
		
			
		
		
		
		
