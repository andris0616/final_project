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
from std_msgs.msg import Bool,Int32, Float32
from nav_msgs.msg import Odometry
#from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Vector3



class Odrive_class(object):
	
	
	
	def __init__(self):
		
		# radius of the wheels [m]
		self.radius = 0.1
		# distnace between the two wheels [m]
		self.wheel_dist = 0.39
		# wheel circumference
		self.circumference = 0.68 # 2 * self.radius * np.pi #0.68 #
		# hall sensor ticks per revolution
		self.ticks = 90
		# torque constant
		self.torque_constant = 8.27
		# current position in m
		self.current_x = 0
		self.current_y = 0
		#current heading orientation in degrees
		self.heading_angle = 0
		self.fi = 0
		
		# all odometry data in one variable
		leg_odometry = Odometry()
		
		# position vector
		self.current_pos = Vector3()
		self.current_pos.x = 0
		self.current_pos.y = 0
		self.current_pos.z = 0
		# velocity vector
		self.current_vel = Vector3()
		self.current_vel.x = 0
		self.current_vel.y = 0
		self.current_vel.z = 0
		
		# leg rotation
		self.leg_rotation = Vector3()
		self.leg_rotation.x = 0
		self.leg_rotation.y = 0
		self.leg_rotation.z = 0
		
		# leg acceleration
		self.leg_acceleration = Vector3()
		self.leg_acceleration.x = 0
		self.leg_acceleration.y = 0
		self.leg_acceleration.z = 0
		
		# leg gyro
		self.leg_gyro = Vector3()
		self.leg_gyro.x = 0
		self.leg_gyro.y = 0
		self.leg_gyro.z = 0
		
		#leg velocities
		self.vel_x = 0
		self.vel_y = 0
		
		# wheel torques
		self.torque_L = 0
		self.torque_R = 0
		
		self.pos_x_list = [0.0]
		self.pos_y_list = [0.0]
		
		# desired position and angle
		self.desired_position = Vector3()
		self.desired_position.x = 0
		self.desired_position.y = 0
		self.desired_position.z = 0
		
		self.desired_h_angle = 0
		
		# new data indicators
		self.new_angle = False
		self.new_position = False
		
		# subscribers
		rospy.Subscriber('heading_angle', Float32, self.heading_angle_callback)
		rospy.Subscriber('desired_angle', Float32, self.desired_angle_callback)
		rospy.Subscriber('desired_position', Vector3, self.desired_position_callback)
		rospy.Subscriber('TL_leg_rotation', Vector3, self.leg_rotation_callback)
		rospy.Subscriber('TL_leg_acceleration', Vector3, self.leg_acceleration_callback)
		rospy.Subscriber('TL_leg_gyro', Vector3, self.leg_gyro_callback)
		
		
		#  publishers 
		#pub_odom = rospy.Publisher('leg_odometry', Odometry, queue_size = 10)
		self.pub_pos = rospy.Publisher('TL_leg_position', Vector3, queue_size = 10)
		self.pub_vel = rospy.Publisher('TL_leg_velocity', Vector3, queue_size = 10)
		#self.pub_ha = rospy.Publisher('TL_leg_heading_angle', Float32, queue_size = 10)
		self.pub_ha = rospy.Publisher('TL_enc_ha', Float32, queue_size = 10)
		self.pub_des_ha = rospy.Publisher('TL_des_enc_ha', Float32, queue_size = 10)
		self.pub_torque_L = rospy.Publisher('TL_leg_torque_L', Float32, queue_size = 10)
		self.pub_torque_R = rospy.Publisher('TL_leg_torque_R', Float32, queue_size = 10)
		self.pub_current_set_L = rospy.Publisher('TL_leg_current_set_L', Float32, queue_size = 10)
		self.pub_current_set_R = rospy.Publisher('TL_leg_current_set_R', Float32, queue_size = 10)
		self.pub_vel_L = rospy.Publisher('TL_leg_vel_L', Float32, queue_size = 10)
		self.pub_vel_R = rospy.Publisher('TL_leg_vel_R', Float32, queue_size = 10)
		self.pub_acc_L = rospy.Publisher('TL_leg_acc_L', Float32, queue_size = 10)
		self.pub_acc_R = rospy.Publisher('TL_leg_acc_R', Float32, queue_size = 10)
		self.pub_R_enc = rospy.Publisher('TL_leg_R_enc', Float32, queue_size = 10)
		self.pub_L_enc = rospy.Publisher('TL_leg_L_enc', Float32, queue_size = 10)
		self.pub_des_vel_L = rospy.Publisher('TL_leg_des_vel_L', Float32, queue_size = 10)
		self.pub_des_vel_R = rospy.Publisher('TL_leg_des_vel_R', Float32, queue_size = 10)
		self.cont_mode_L = rospy.Publisher('TL_leg_cont_mode_L', Float32, queue_size = 10)
		self.cont_mode_R = rospy.Publisher('TL_leg_cont_mode_R', Float32, queue_size = 10)
		
		
		

		
		# connecting to odrive
		print("connecting to odrive")
		self.my_odrive = odrive.find_any()
		print("Connected to odrive")
		
		# controller gains
		self.vel_gain = self.my_odrive.axis0.controller.config.vel_gain
		self.vel_integrator_gain = self.my_odrive.axis0.controller.config.vel_integrator_gain
		
		self.current_lim = self.my_odrive.axis0.motor.effective_current_lim
		self.torque_lim = self.current_lim * self.torque_constant
		
		# Calibrating the Odrive
		#self.calibrate()

		#self.my_odrive.axis0.requested_state = AXIS_STATE_IDLE
		#self.my_odrive.axis1.requested_state = AXIS_STATE_IDLE
		
		#self.my_odrive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
		#self.my_odrive.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL


		
		print("odrive is ready to use")
		
	
	""" Put odrive to free rotate """
	def setIdle(self):
		self.my_odrive.axis0.requested_state = AXIS_STATE_IDLE
		self.my_odrive.axis1.requested_state = AXIS_STATE_IDLE
		
	"""To calibrate the motors"""
	def calibrate(self):
		
		# calibrating the motors
		print("Motor calibration...")
		
		# motor0 calibration
		self.my_odrive.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
		
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
	
	"""This function reboots the odrive"""	
	def reboot(self):
		self.my_odrive.reboot()
		
	"""Callback function for storing the heading angle"""
	def heading_angle_callback(self, msg):
		self.heading_angle = msg.data
		
	"""Callback function for storing the heading angle"""
	def desired_angle_callback(self, msg):
		self.desired_h_angle = msg.data
		self.new_angle = True

	def desired_position_callback(self, msg):
		self.desired_position.x = msg.x
		self.desired_position.y = msg.y
		self.desired_position.z = msg.z
		self.new_position = True
		
	def leg_rotation_callback(self, msg):
		self.leg_rotation.x = msg.x
		self.leg_rotation.y = msg.y
		self.leg_rotation.z = msg.z
		
	def leg_acceleration_callback(self, msg):
		self.leg_acceleration.x = msg.x
		self.leg_acceleration.y = msg.y
		self.leg_acceleration.z = msg.z
		
	def leg_gyro_callback(self, msg):
		self.leg_gyro.x = msg.x
		self.leg_gyro.y = msg.y
		self.leg_gyro.z = msg.z

	"""Gives back the current number of rotations of the right wheel"""
	def getEncoder0(self):
		enc0 = self.my_odrive.axis0.encoder.pos_estimate
		return enc0
		
	"""Gives back the current number of rotations of the left wheel"""	
	def getEncoder1(self):
		enc1 = -self.my_odrive.axis1.encoder.pos_estimate
		return enc1
		
	
	"""Gives back the current velocity of the right wheel"""
	def getVel0(self):
		vel0 = self.my_odrive.axis0.encoder.vel_estimate
		return vel0
		
	"""Gives back the current velocity of the left wheel"""	
	def getVel1(self):
		vel1 = -self.my_odrive.axis1.encoder.vel_estimate
		return vel1
		
	"""Gives back the current torque of the left wheel """
	def getTorque0(self):
		torque0 = self.my_odrive.axis0.motor.current_control.Iq_measured * self.torque_constant
		return torque0
		
	"""Gives back the current torque of the right wheel """
	def getTorque1(self):
		torque1 = -self.my_odrive.axis1.motor.current_control.Iq_measured * self.torque_constant
		return torque1
		
	"""Gives back the current setpoint current of the left wheel """
	def getCurrentSetP0(self):
		current0 = self.my_odrive.axis0.motor.current_control.Iq_setpoint 
		return current0
		
	"""Gives back the current setpoint current of the right wheel """
	def getCurrentSetP1(self):
		current1 = -self.my_odrive.axis1.motor.current_control.Iq_setpoint 
		return current1
		
	"""This function sets the odrive into velocity control mode"""
	def setVelocity_control_mode(self):
		self.my_odrive.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
		self.my_odrive.axis1.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
		
	"""This function sets the left wheel into velocity control mode"""
	def setVelocity_control_mode1(self):
		self.my_odrive.axis1.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
		
	"""This function sets the right wheel into velocity control mode"""
	def setVelocity_control_mode0(self):
		self.my_odrive.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
		
	"""This function sets the odrive into torque control mode"""
	def setTorque_control_mode(self):
		self.my_odrive.axis0.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL
		self.my_odrive.axis1.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL
		
	"""This function sets the right into torque control mode"""
	def setTorque_control_mode0(self):
		self.my_odrive.axis0.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL
		
	"""This function sets the left wheel into torque control mode"""
	def setTorque_control_mode1(self):
		self.my_odrive.axis1.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL
		
	"""This function sets the odrive into position control mode"""	
	def setPosition_control_mode(self):
		starting_pos0 = self.my_odrive.axis0.encoder.pos_estimate
		self.my_odrive.axis0.controller.input_pos = starting_pos0
		starting_pos1 = self.my_odrive.axis1.encoder.pos_estimate
		self.my_odrive.axis1.controller.input_pos = starting_pos1
		
		self.my_odrive.axis0.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
		self.my_odrive.axis1.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL

	"""
	This function set the input velocity of the right wheel
	
	@param:
		vel: input velocity in turn/second
	
	"""
	def setVel0(self, vel):
		""" Input vel is in turn/second"""
		self.my_odrive.axis0.controller.input_vel = vel
		
	"""
	This function set the input velocity of the left wheel
	
	@param:
		vel: input velocity in turn/second
	
	"""
	def setVel1(self, vel):
		""" Input vel is in turn/second"""
		self.my_odrive.axis1.controller.input_vel = -vel
		
	"""
	This function set the input torque of the right wheel
	
	@param:
		torque: input torque is in Nm
	
	"""
	def setTorque0(self, torque):
		self.my_odrive.axis0.controller.input_torque = torque
		
	"""
	This function set the input torque of the left wheel
	
	@param:
		torque: input torque is in Nm
	
	"""
	def setTorque1(self, torque):
		self.my_odrive.axis1.controller.input_torque = -torque
			
	"""
	Sraight line position control based on the position control mode of the odrive
	
	@param:
		desired_pos: desired position in [m]
	"""		
	def straight_position_control(self, desired_pos):
		
		self.setPosition_control_mode()
		
		#convert desired pos to number of rotations
		rotations = desired_pos / self.circumference
		
		self.my_odrive.axis0.controller.input_pos = rotations
		self.my_odrive.axis1.controller.input_pos = rotations
			
	"""
	Turns the leg with a desired angle
	
	@params:
		desired_angle: desired turning angle in degrees
	"""	
	def angular_pos_control(self, desired_angle):
		# set control mode to velocity control
		self.setVelocity_control_mode()
		
		#calculate the arc the wheels need to rotate to get to the desired angle
		ds = desired_angle / 360 * self.wheel_dist *np.pi
		
		#convert desired pos to number of rotations
		rotations = (ds / self.circumference) + self.getEncoder0()
		
		
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
			
			print("heading angle:")
			print(self.heading_angle)
			
			
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
		
		
	"""
	Turns the leg to a desired heading angle
	
	@params:
		desired_angle: desired angle in radians
	"""	
	def heading_angle_control(self, desired_angle):
		# set control mode to velocity control
		self.setVelocity_control_mode()
		
		# PID params
		Kp = 20
		Ki = 1.0
		Kd = 15
		
		epsilon = 0.01
		
		e_prev_angle = 0
		ei_angle = 0
		dt = 0.01
		
		# for velocity approximation
		x_prev = self.current_pos.x
		y_prev = self.current_pos.y
		
		enc0_prev = self.getEncoder0()
		enc1_prev = self.getEncoder1()
		
		
		while not rospy.is_shutdown() and abs(self.fi - desired_angle) > epsilon:
			
			enc0 = self.getEncoder0()
			enc1 = self.getEncoder1()
			# distance the left wheel travelled
			dR = (enc0 - enc0_prev) * self.circumference #/ self.ticks
			# distance the right wheel travelled
			dL = (enc1 - enc1_prev) * self.circumference #/ self.ticks
			
			# angle of the leg from the hall sensors (this should be changed later to the leg angle ecoder data I guess)
			self.fi += 1/self.wheel_dist * (dR - dL)
			
			fi_deg = self.fi/np.pi*180

			print("calculated heading angles: {}".format(fi_deg))
			print("heading angle:")
			print(self.heading_angle)
			
			
			
			# current position calculated from the hall data 
			self.current_pos.x = self.current_pos.x + 1/2 * (dR + dL) * np.cos(self.fi)
			self.current_pos.y = self.current_pos.y + 1/2 * (dR + dL) * np.sin(self.fi)
			
			#store pos
			self.pos_x_list.append(self.current_pos.x)
			self.pos_y_list.append(self.current_pos.y)
			
			#print("current x: {} m".format(self.current_pos.x))
			#print("current y: {} m".format(self.current_pos.y))
			
			#distance travelled ion this time step
			distx = self.current_pos.x - x_prev
			disty = self.current_pos.y - y_prev
			
			# leg velocity calculation
			self.current_vel.x = distx / dt
			self.current_vel.y = disty / dt
			
			# -180/180 orientation fixing
			if abs(desired_angle + 2 * np.pi - self.fi) < abs(desired_angle - self.fi):
				desired_angle += 2 * np.pi

			# error
			e_angle = desired_angle - self.fi
			ed_angle = (e_angle - e_prev_angle) / dt
			ei_angle = ei_angle + e_angle*dt
			
			#print("current angle {}".format(self.fi))
			
			# PID equation
			fi_dot = Kp * e_angle + Ki * ei_angle + Kd * ed_angle
			
			
			# next input velocity calculation
			vel0 = + 1/2 * self.wheel_dist * fi_dot
			vel1 = - 1/2 * self.wheel_dist * fi_dot
                        

			# limit the input velocities
			vel0 = max(min(vel0, 0.4), -0.4)
			vel1 = max(min(vel1, 0.4), -0.4)


			"""if abs(desired_angle - self.fi) < 0.3:
				vel0 = vel0 * 0.3  # abs(desired_angle - self.fi)
				vel1 = vel1 * 0.3  # abs(desired_angle - self.fi)
				
			elif abs(desired_angle - self.fi) < 0.5:
				vel0 = vel0 * abs(desired_angle - self.fi) #0.5
				vel1 = vel1 * abs(desired_angle - self.fi)  #0.5"""
				
			# set the new input velocities
			self.setVel0(vel0)
			self.setVel1(vel1)
			
			#Update
			e_prev_angle = e_angle
			enc0_prev = enc0
			enc1_prev = enc1
			x_prev = self.current_pos.x
			y_prev = self.current_pos.y
			
			
			# publish data
			self.pub_pos.publish(self.current_pos)
			self.pub_vel.publish(self.current_vel)
			self.pub_ha.publish(self.fi)
			self.pub_torque_R.publish(self.my_odrive.axis0.motor.current_control.Iq_measured * self.torque_constant)
			self.pub_torque_L.publish(-self.my_odrive.axis1.motor.current_control.Iq_measured * self.torque_constant)
			self.pub_vel_R.publish(self.getVel0())
			self.pub_vel_L.publish(self.getVel1())
  			
			time.sleep(dt)
				
		# STOP
		self.setVel0(0)
		self.setVel1(0)
		
		print("FINISHED")
				
			
			
		
		
	"""
	Straight line position control based on the velocity control mode of the odrive
	
	@params:
		desired_pos: desired_position in [m]
	"""	
	def pos_control(self, desired_pos):
		
		# set control mode to velocity control
		self.setVelocity_control_mode()
		
		#convert desired pos to number of rotations
		rotations = (desired_pos / self.circumference) + self.getEncoder0()
		

		# PID params
		Kp = 2
		Ki = 0.0
		Kd = 2.0
		
		epsilon = 0.01
		
		e_prev = 0
		ei = 0
		dt = 0.01
		
		# START
		self.setVel0(0.4)
		self.setVel1(0.4)	
		
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
			vel = max(min(vel,0.4),-0.4)
			
			
			# set the new input velocities
			self.setVel0(vel)
			self.setVel1(vel)
			
			e_prev = e
			
			time.sleep(dt)
			
		# STOP
		self.setVel0(0)
		self.setVel1(0)	
		print("Finished")
		print(self.getEncoder0())
		print(self.getEncoder1())
		print("Desired pos: {}".format(rotations))
			
		
		
	"""
	Differential drive 2D position control
	
	@params:
		desired_x: desired x position in [m]
		desired_y: desired y position in [m]
	"""
	def diff_drive_control(self, desired_x, desired_y):
		
		# set control mode to velocity control
		self.setVelocity_control_mode()
		
		# PID params for the heading angle control
		Kp_a = 7 #15 #7 10
		Ki_a = 1   #1
		Kd_a = 5  #10 #5.0 7
		
		epsilon = 0.01
		
		
		enc0_prev = self.getEncoder0()
		enc1_prev = self.getEncoder1()
		
		e_prev_angle = 0
		ei_angle = 0
		dt = 0.01
		
		# for velocity approximation
		x_prev = self.current_pos.x
		y_prev = self.current_pos.y
		
		
		# starting speed
		vel = 0.4
		
		self.setVel0(vel)
		self.setVel1(vel)
		
		# initial distance
		dx = desired_x - self.current_pos.x
		dy = desired_y - self.current_pos.y
		
		time_prev = rospy.get_time()
		control_mode_L = 1
		control_mode_R = 1
		
		
		while epsilon < np.sqrt((dx)**2 + (dy)**2) and not rospy.is_shutdown():
			
			enc0 = self.getEncoder0()
			enc1 = self.getEncoder1()
			# distance the left wheel travelled
			dR = (enc0 - enc0_prev) * self.circumference #/ self.ticks
			# distance the right wheel travelled
			dL = (enc1 - enc1_prev) * self.circumference #/ self.ticks
			
			# angle of the leg from the hall sensors (this should be changed later to the leg angle ecoder data I guess)
			self.fi += 1/self.wheel_dist * (dR - dL)
			
			fi_deg = self.fi/np.pi*180

			#print("calculated heading angles: {}".format(fi_deg))
			
			
			# current position calculated from the hall data 
			self.current_pos.x = self.current_pos.x + 1/2 * (dR + dL) * np.cos(self.fi)
			self.current_pos.y = self.current_pos.y + 1/2 * (dR + dL) * np.sin(self.fi)
			
			#store pos
			self.pos_x_list.append(self.current_pos.x)
			self.pos_y_list.append(self.current_pos.y)
			
			#print("current x: {} m".format(self.current_pos.x))
			#print("current y: {} m".format(self.current_pos.y))
			
			# current position difference
			dx = desired_x - self.current_pos.x
			dy = desired_y - self.current_pos.y
			
			#distance travelled ion this time step
			distx = self.current_pos.x - x_prev
			disty = self.current_pos.y - y_prev
			
			time_now = rospy.get_time()
			d_time = time_now - time_prev
			time_prev = time_now
			
			# leg velocity calculation
			self.current_vel.x = distx / d_time
			self.current_vel.y = disty / d_time
			
			
			#print("dx: {} m".format(dx))
			#print("dy: {} m".format(dy))
			print("distance: {}".format(np.sqrt(dx**2+dy**2)))

			# -180/180 orientation fixing
			desired_angle = math.atan2(dy, dx)
			if abs(desired_angle + 2 * np.pi - self.fi) < abs(desired_angle - self.fi):
				desired_angle += 2 * np.pi

			# error
			e_angle = desired_angle - self.fi
			ed_angle = (e_angle - e_prev_angle) / d_time
			ei_angle = ei_angle + e_angle*dt
			
			#print("desired angle {}".format(math.atan2(dy, dx)))
			#print("current angle {}".format(self.fi))
			
			# PID equation
			fi_dot = Kp_a*e_angle + Ki_a* ei_angle + Kd_a * ed_angle
			
			
			# next input velocity calculation
			vel0 = vel + 1/2 * self.wheel_dist * fi_dot
			vel1 = vel - 1/2 * self.wheel_dist * fi_dot
                        
			#print("right wheel velocity: {}".format(vel0))
			#print("left wheel velocity: {}".format(vel1))

			# limit the input velocities
			vel0 = max(min(vel0, 0.5), -0.5)
			vel1 = max(min(vel1, 0.5), -0.5)


			if np.sqrt((dx)**2 + (dy)**2) < 0.3:
				vel0 = vel0 * 0.5
				vel1 = vel1 * 0.5
				#vel0 = max(min(vel0, 0.3), -0.3)
				#vel1 = max(min(vel1, 0.3), -0.3)
				
			"""elif np.sqrt((dx)**2 + (dy)**2) < 0.5:
				vel0 = vel0 * np.sqrt((dx)**2 + (dy)**2)
				vel1 = vel1 * np.sqrt((dx)**2 + (dy)**2)"""
				
			
			# limit the input velocities
			#vel0 = max(min(vel0,0.4),-0.4)
			#vel1 = max(min(vel1,0.4),-0.4)
			
			#print("right wheel velocity: {}".format(vel0))
			#print("left wheel velocity: {}".format(vel1))
			
			# set the new input velocities
			self.setVel0(vel0)
			self.setVel1(vel1)
			
			#Update
			e_prev_angle = e_angle
			#e_prev_pos = e_pos
			enc0_prev = enc0
			enc1_prev = enc1
			x_prev = self.current_pos.x
			y_prev = self.current_pos.y
			
			# publish data
			self.pub_pos.publish(self.current_pos)
			self.pub_vel.publish(self.current_vel)
			self.pub_ha.publish(self.fi)
			self.pub_des_ha.publish(math.atan2(dy, dx))
			self.pub_torque_R.publish(self.getTorque0())
			self.pub_torque_L.publish(self.getTorque1())
			self.pub_vel_R.publish(self.getVel0())
			self.pub_vel_L.publish(self.getVel1())
			self.pub_des_vel_L.publish(vel1)
			self.pub_des_vel_R.publish(vel0)
			self.pub_current_set_L.publish(self.getCurrentSetP1())
			self.pub_current_set_R.publish(self.getCurrentSetP0())
			self.cont_mode_L.publish(control_mode_L)
			self.cont_mode_R.publish(control_mode_R)
  			
			
		
		# STOP
		self.setVel0(0)
		self.setVel1(0)
		
		print("FINISHED")
		#self.angular_pos_control(0)

		#self.my_odrive.axis0.requested_state = AXIS_STATE_IDLE
		#self.my_odrive.axis1.requested_state = AXIS_STATE_IDLE
		

		#print("pos from enc: {}".format(enc0 * self.circumference))
		
	
	#########################################
	"""
	Rough terrain 2D position control without rosTime
	
	@params:
		desired_x: desired x position in [m]
		desired_y: desired y position in [m]
	"""
	def rough_terrain_control(self, desired_x, desired_y):
		
		# set control mode to velocity control
		self.setVelocity_control_mode()
		
		# PID params for the heading angle control
		Kp_a = 7 #15 #7 10
		Ki_a = 1   #1
		Kd_a = 5  #10 #5.0 7
		
		epsilon = 0.01
		
		
		enc0_prev = self.getEncoder0()
		enc1_prev = self.getEncoder1()
		
		e_prev_angle = 0
		ei_angle = 0
		dt = 0.01
		
		# for velocity approximation
		x_prev = self.current_pos.x
		y_prev = self.current_pos.y
		
		
		# starting speed
		vel = 0.5
		max_vel = 0.5
		
		self.setVel0(vel)
		self.setVel1(vel)
		
		# initial distance
		dx = desired_x - self.current_pos.x
		dy = desired_y - self.current_pos.y
		
		bumper_L = False
		bumper_R = False
		wheelR_vel_prev = 0
		wheelL_vel_prev = 0
		torque0 = 50
		torque1 = 50
		breakingR = False
		breakingL = False
		
		finish = False
		
		while epsilon < np.sqrt((dx)**2 + (dy)**2) and not rospy.is_shutdown() and not finish:
			
			enc0 = self.getEncoder0()
			enc1 = self.getEncoder1()
			# distance the left wheel travelled
			dR = (enc0 - enc0_prev) * self.circumference #/ self.ticks
			# distance the right wheel travelled
			dL = (enc1 - enc1_prev) * self.circumference #/ self.ticks
			
			# angle of the leg from the hall sensors (this should be changed later to the leg angle ecoder data I guess)
			self.fi += 1/self.wheel_dist * (dR - dL)
			
			fi_deg = self.fi/np.pi*180

			#print("calculated heading angles: {}".format(fi_deg))
			
			
			# current position calculated from the hall data 
			self.current_pos.x = self.current_pos.x + 1/2 * (dR + dL) * np.cos(self.fi)
			self.current_pos.y = self.current_pos.y + 1/2 * (dR + dL) * np.sin(self.fi)
			
			#store pos
			self.pos_x_list.append(self.current_pos.x)
			self.pos_y_list.append(self.current_pos.y)
			
			#print("current x: {} m".format(self.current_pos.x))
			#print("current y: {} m".format(self.current_pos.y))
			
			# current position difference
			dx = desired_x - self.current_pos.x
			dy = desired_y - self.current_pos.y
			
			#distance travelled ion this time step
			distx = self.current_pos.x - x_prev
			disty = self.current_pos.y - y_prev
			
			# leg velocity calculation
			self.current_vel.x = distx / dt
			self.current_vel.y = disty / dt
			
			
			#print("dx: {} m".format(dx))
			#print("dy: {} m".format(dy))
			print("distance: {}".format(np.sqrt(dx**2+dy**2)))

			# -180/180 orientation fixing
			desired_angle = math.atan2(dy, dx)
			if abs(desired_angle + 2 * np.pi - self.fi) < abs(desired_angle - self.fi):
				desired_angle += 2 * np.pi

			# error
			e_angle = desired_angle - self.fi
			ed_angle = (e_angle - e_prev_angle) / dt
			ei_angle = ei_angle + e_angle*dt
			
			#print("desired angle {}".format(math.atan2(dy, dx)))
			#print("current angle {}".format(self.fi))
			
			# PID equation
			fi_dot = Kp_a*e_angle + Ki_a* ei_angle + Kd_a * ed_angle
			
			
			# next input velocity calculation
			vel0 = vel + 1/2 * self.wheel_dist * fi_dot
			vel1 = vel - 1/2 * self.wheel_dist * fi_dot
			
			print("velocity")
			print(vel0)
			print(vel1)      
                  
			# limit the input velocities
			vel0 = max(min(vel0, max_vel), -max_vel)
			vel1 = max(min(vel1, max_vel), -max_vel)


			if np.sqrt((dx)**2 + (dy)**2) < 0.3:
				vel0 = vel0 * 0.5
				vel1 = vel1 * 0.5
			
			wheelR_vel = self.getVel0()	
			wheelR_acc = (wheelR_vel - wheelR_vel_prev) / dt
			wheelR_vel_prev = wheelR_vel
			torqueR = self.getTorque0()
			
			wheelL_vel = self.getVel1()	
			wheelL_acc = (wheelL_vel - wheelL_vel_prev) / dt
			wheelL_vel_prev = wheelL_vel
			torqueL = self.getTorque1()
				
			
			# check whether there is a bumper or not
			if abs(torqueL) > 60 and dL == 0.0:
				bumper_L = True
				
			if abs(torqueR) > 60 and dR == 0.0:
				bumper_R = True
				
			if bumper_L == False and bumper_R == False:
				# set the new input velocities
				self.setVel0(vel0)
				self.setVel1(vel1)
				control_mode_R = 1
				control_mode_L = 1
				
			else:
				if bumper_R == True and bumper_L == True:
					print("TORQUE CONTROL R")
					# if right wheel gets over the bumper then stop
					if (abs(wheelR_acc) > 4.0 and abs(self.getVel0()) > 0.5) or breakingR:
						self.my_odrive.axis0.controller.config.vel_gain = self.vel_gain
						self.my_odrive.axis0.controller.config.vel_integrator_gain = self.vel_integrator_gain
						breakingR = True
						
						self.setTorque_control_mode0()
						self.setTorque0(-self.getTorque0())
						print("BREAK")
						control_mode_R = 3
						control_mode_L = 2
						
						if self.getVel0() < 0.1:	
							breakingR = False
							self.setVelocity_control_mode0()
							vel0 = 0
							vel1 = 0
							self.setVel0(vel0)
							bumper_R = False
							
					# if lefty wheel gets over the bumper then stop
					if (abs(wheelL_acc) > 4.0 and abs(self.getVel1()) > 0.5) or breakingL:
						self.my_odrive.axis1.controller.config.vel_gain = self.vel_gain
						self.my_odrive.axis1.controller.config.vel_integrator_gain = self.vel_integrator_gain
						breakingL = True
						
						self.setTorque_control_mode1()
						self.setTorque1(-self.getTorque1())
						print("BREAK")
						control_mode_R = 2
						control_mode_L = 3
						
						if self.getVel1() < 0.1:	
							breakingL = False
							self.setVelocity_control_mode1()
							vel0 = 0
							vel1 = 0
							self.setVel0(vel0)
							self.setVel1(vel1)
							bumper_L = False
							
					else:	
						self.my_odrive.axis0.controller.config.vel_gain += 10
						self.my_odrive.axis1.controller.config.vel_gain += 10
						print("vel_gain")
						print(self.my_odrive.axis0.controller.config.vel_gain)
						self.my_odrive.axis0.controller.config.vel_integrator_gain = 0
						self.my_odrive.axis1.controller.config.vel_integrator_gain = 0
						
						# set the new input velocities
						vel0 = max_vel
						vel1 = max_vel
						
						self.setVel0(vel0) # vel0
						self.setVel1(vel1) #vel1
						control_mode_R = 2
						control_mode_L = 2
					
				#-----------------------------------------------------------------------
				
				elif bumper_R == True:
					print("bumper R")
					# if right wheel gets over the bumper then stop
					if (abs(wheelR_acc) > 4.0 and abs(self.getVel0()) > 0.5) or breakingR:
						self.my_odrive.axis0.controller.config.vel_gain = self.vel_gain
						self.my_odrive.axis0.controller.config.vel_integrator_gain = self.vel_integrator_gain
						breakingR = True
						
						self.setTorque_control_mode0()
						self.setTorque0(-self.getTorque0())
						print("BREAK")
						control_mode_R = 3
						control_mode_L = 1
						
						if self.getVel0() < 0.1:	
							breakingR = False
							self.setVelocity_control_mode0()
							vel0 = 0
							vel1 = 0
							self.setVel0(vel0)
							self.setVel1(vel1)
							bumper_R = False
						
					else:	
						self.my_odrive.axis0.controller.config.vel_gain += 10
						print("vel_gain")
						print(self.my_odrive.axis0.controller.config.vel_gain)
						self.my_odrive.axis0.controller.config.vel_integrator_gain = 0
						
						# set the new input velocities
						vel0 = max_vel
						vel1 = 0
						
						self.setVel0(vel0) # vel0
						self.setVel1(vel1) #vel1
						control_mode_R = 2
						control_mode_L = 1
							
				#------------------------------------------------------------------	

				elif bumper_L == True:
					print("bumper L")
					# if right wheel gets over the bumper then stop
					if (abs(wheelL_acc) > 4.0 and abs(self.getVel1()) > 0.5) or breakingL:
						self.my_odrive.axis1.controller.config.vel_gain = self.vel_gain
						self.my_odrive.axis1.controller.config.vel_integrator_gain = self.vel_integrator_gain
						breakingL = True
						
						self.setTorque_control_mode1()
						self.setTorque1(-self.getTorque1())
						print("BREAK")
						control_mode_R = 1
						control_mode_L = 3
						
						if self.getVel1() < 0.1:	
							breakingL = False
							self.setVelocity_control_mode1()
							vel0 = 0
							vel1 = 0
							self.setVel0(vel0)
							self.setVel1(vel1)
							bumper_L = False
							#finish = True
					else:	
						self.my_odrive.axis1.controller.config.vel_gain += 10
						print("vel_gain")
						print(self.my_odrive.axis1.controller.config.vel_gain)
						self.my_odrive.axis1.controller.config.vel_integrator_gain = 0
						
						# set the new input velocities
						vel1 = max_vel
						vel0 = 0
						
						self.setVel0(vel0) # vel0
						self.setVel1(vel1) #vel1
						control_mode_R = 1
						control_mode_L = 2
				
			
			#############################################3
				
			
			
			# set the new input velocities
			#self.setVel0(vel0)
			#self.setVel1(vel1)
			
			#Update
			e_prev_angle = e_angle
			#e_prev_pos = e_pos
			enc0_prev = enc0
			enc1_prev = enc1
			x_prev = self.current_pos.x
			y_prev = self.current_pos.y
			
			# publish data
			self.pub_pos.publish(self.current_pos)
			self.pub_vel.publish(self.current_vel)
			self.pub_ha.publish(self.fi)
			self.pub_des_ha.publish(math.atan2(dy, dx))
			self.pub_torque_R.publish(self.getTorque0()/self.torque_constant)
			self.pub_torque_L.publish(self.getTorque1()/self.torque_constant)
			self.pub_vel_R.publish(self.getVel0())
			self.pub_vel_L.publish(self.getVel1())
			self.pub_acc_R.publish(wheelR_acc)
			self.pub_acc_L.publish(wheelL_acc)
			self.pub_R_enc.publish(enc0)
			self.pub_L_enc.publish(enc1)
			self.pub_des_vel_L.publish(vel1)
			self.pub_des_vel_R.publish(vel0)
			self.pub_current_set_L.publish(self.getCurrentSetP1())
			self.pub_current_set_R.publish(self.getCurrentSetP0())
			self.cont_mode_L.publish(control_mode_L)
			self.cont_mode_R.publish(control_mode_R)
  			
			time.sleep(dt)
			
		
		# STOP
		self.setVel0(0)
		self.setVel1(0)
		
		print("FINISHED")
		print("current x pos: {}".format(self.current_pos.x))
		print("current y pos: {}".format(self.current_pos.y))
	
	#####################################
	
		
	"""
	Rough terrain 2D position control
	
	@params:
		desired_x: desired x position in [m]
		desired_y: desired y position in [m]
	"""
	def rough_terrain_control_2(self, desired_x, desired_y):
		
		# set control mode to velocity control
		self.setVelocity_control_mode()
		
		# PID params for the heading angle control
		Kp_a = 7 #15 #7 10
		Ki_a = 1   #1
		Kd_a = 5  #10 #5.0 7
		
		epsilon = 0.01
		
		
		enc0_prev = self.getEncoder0()
		enc1_prev = self.getEncoder1()
		
		e_prev_angle = 0
		ei_angle = 0
		dt = 0.01
		
		# for velocity approximation
		x_prev = self.current_pos.x
		y_prev = self.current_pos.y
		
		
		# starting speed
		avg_vel = 0.3
		max_vel = 0.4
		vel_thresh = 0.5
		# acc threshold for bumper stuff
		acc_treshold = 4.0
		travel_threshold = 0.15
		
		self.setVel0(avg_vel)
		self.setVel1(avg_vel)
		
		# initial distance
		dx = desired_x - self.current_pos.x
		dy = desired_y - self.current_pos.y
		
		bumper_L = False
		bumper_R = False
		wheelR_vel_prev = 0
		wheelL_vel_prev = 0
		torque0 = 50
		torque1 = 50
		breakingR = False
		breakingL = False
		
		# control mode of the wheels
		control_mode_R = 1
		control_mode_L = 1
		
		# distance travelled by the wheels since the bumper is detected
		d_trav_L = 0
		d_trav_R = 0
		
		finish = False
		
		time_prev = rospy.get_time()
		
		while epsilon < np.sqrt((dx)**2 + (dy)**2) and not rospy.is_shutdown() and not finish:
			
			enc0 = self.getEncoder0()
			enc1 = self.getEncoder1()
			# distance the left wheel travelled
			dR = (enc0 - enc0_prev) * self.circumference #/ self.ticks
			# distance the right wheel travelled
			dL = (enc1 - enc1_prev) * self.circumference #/ self.ticks
			
			# angle of the leg from the hall sensors (this should be changed later to the leg angle ecoder data I guess)
			self.fi += 1/self.wheel_dist * (dR - dL)
			
			fi_deg = self.fi/np.pi*180

			#print("calculated heading angles: {}".format(fi_deg))
			
			
			# current position calculated from the hall data 
			self.current_pos.x = self.current_pos.x + 1/2 * (dR + dL) * np.cos(self.fi)
			self.current_pos.y = self.current_pos.y + 1/2 * (dR + dL) * np.sin(self.fi)
			
			#store pos
			self.pos_x_list.append(self.current_pos.x)
			self.pos_y_list.append(self.current_pos.y)
			
			#print("current x: {} m".format(self.current_pos.x))
			#print("current y: {} m".format(self.current_pos.y))
			
			# current position difference
			dx = desired_x - self.current_pos.x
			dy = desired_y - self.current_pos.y
			
			#distance travelled ion this time step
			distx = self.current_pos.x - x_prev
			disty = self.current_pos.y - y_prev
			
			time_now = rospy.get_time()
			d_time = time_now - time_prev
			time_prev = time_now
			# leg velocity calculation
			self.current_vel.x = distx / d_time #dt
			self.current_vel.y = disty / d_time #dt
			
			
			#print("dx: {} m".format(dx))
			#print("dy: {} m".format(dy))
			print("distance: {}".format(np.sqrt(dx**2+dy**2)))

			# -180/180 orientation fixing
			desired_angle = math.atan2(dy, dx)
			if abs(desired_angle + 2 * np.pi - self.fi) < abs(desired_angle - self.fi):
				desired_angle += 2 * np.pi

			# error
			e_angle = desired_angle - self.fi
			ed_angle = (e_angle - e_prev_angle) / d_time #dt
			ei_angle = ei_angle + e_angle*dt
			
			#print("desired angle {}".format(math.atan2(dy, dx)))
			#print("current angle {}".format(self.fi))
			
			# PID equation
			fi_dot = Kp_a*e_angle + Ki_a* ei_angle + Kd_a * ed_angle
			
			
			# next input velocity calculation
			vel0 = avg_vel + 1/2 * self.wheel_dist * fi_dot
			vel1 = avg_vel - 1/2 * self.wheel_dist * fi_dot
			     
                  
			# limit the input velocities
			vel0 = max(min(vel0, max_vel), -max_vel)
			vel1 = max(min(vel1, max_vel), -max_vel)


			if np.sqrt((dx)**2 + (dy)**2) < 0.3:
				vel0 = vel0 * 0.5
				vel1 = vel1 * 0.5
			
			wheelR_vel = self.getVel0()	
			wheelR_acc = (wheelR_vel - wheelR_vel_prev) / d_time #dt
			wheelR_vel_prev = wheelR_vel
			torqueR = self.getTorque0()
			
			wheelL_vel = self.getVel1()	
			wheelL_acc = (wheelL_vel - wheelL_vel_prev) / d_time #dt
			wheelL_vel_prev = wheelL_vel
			torqueL = self.getTorque1()
				
			#print("R encoder change: {}".format(dR))
			#print("R wheel velocity: {}".format(wheelR_vel))
			#print("R wheel torque: {}".format(torqueL))
			#print("R wheel acceleration: {}".format(wheelR_acc))
			
			"""print("L encoder change: {}".format(dL))
			print("L wheel velocity: {}".format(wheelL_vel))
			print("L wheel torque: {}".format(torqueL))
			print("L wheel acceleration: {}".format(wheelL_acc))"""

			
			# limit the input velocities
			#vel0 = max(min(vel0,0.4),-0.4)
			#vel1 = max(min(vel1,0.4),-0.4)
			
			#print("right wheel velocity: {}".format(vel0))
			#print("left wheel velocity: {}".format(vel1))
			
			#if np.sqrt((dx)**2 + (dy)**2) > 0.05 and 
			
			
			# check whether there is a bumper or not
			"""if abs(torqueL) > 60 and dL == 0.0:
				bumper_L = True
				
			if abs(torqueR) > 60 and dR == 0.0:
				bumper_R = True
			
			if bumper_R == True and bumper_L == True:
				# if right wheel gets over the bumper then stop
				if abs(wheelR_acc) > 4.0:
					self.setTorque0(-torque0)
					self.setVelocity_control_mode0()
					self.setVel0(vel0)
					self.setVel1(vel1)
					bumper_R = False
					
				# if left wheel gets over the bumper then stop
				if abs(wheelL_acc) > 4.0:
					self.setTorque1(-torque1)
					self.setVelocity_control_mode1()
					self.setVel1(vel1)
					self.setVel0(vel0)
					bumper_L = False
					
					
				if 	abs(wheelL_acc) < 4.0 and abs(wheelR_acc) < 4.0:
					self.setTorque_control_mode()
					print("TORQUE CONTROL BOTH")
					
					torque0 = torque1 + 5
					self.setTorque0(torque0)
					
					torque1 = torque1 + 5
					self.setTorque1(torque1)
				
			elif bumper_R == True:
				# if right wheel gets over the bumper then stop
				if abs(wheelR_acc) > 4.0:
					self.setTorque0(-torque0)
					self.setVelocity_control_mode0()
					self.setVel0(vel0)
					self.setVel1(vel1)
					bumper_R = False
				else:
				
					self.setTorque_control_mode0()
					print("TORQUE CONTROL R")
					
					torque0 = torque0 + 5
					self.setTorque0(torque0)
					self.setVel1(vel1)
				
			elif bumper_L == True:
				# if left wheel gets over the bumper then stop
				if abs(wheelL_acc) > 4.0:
					self.setTorque1(-torque1)
					self.setVelocity_control_mode1()
					self.setVel1(vel1)
					self.setVel0(vel0)
					bumper_L = False
				else:
				
					self.setTorque_control_mode1()
					print("TORQUE CONTROL L")
					
					torque1 = torque1 + 5
					self.setTorque1(torque1)
					self.setVel0(vel0)
				
			else:
				# set the new input velocities
				self.setVel0(vel0)
				self.setVel1(vel1)"""
				
			####################################
			# check whether there is a bumper or not
			"""if abs(torqueL) > 60 and dL == 0.0:
				bumper_L = True
				
			if abs(torqueR) > 60 and dR == 0.0:
				bumper_R = True
			
				
			if bumper_R == True:
				print("TORQUE CONTROL R")
				# if right wheel gets over the bumper then stop
				if abs(wheelR_acc) > 4.0:
					self.my_odrive.axis0.controller.config.vel_gain = self.vel_gain
					self.my_odrive.axis0.controller.config.vel_integrator_gain = self.vel_integrator_gain
					
					self.setVel0(0)
					self.setVel1(0)
					bumper_R = False
					#finish = True
				else:	
					self.my_odrive.axis0.controller.config.vel_gain += 10
					print("vel_gain")
					print(self.my_odrive.axis0.controller.config.vel_gain)
					self.my_odrive.axis0.controller.config.vel_integrator_gain = 0
					
					# set the new input velocities
					
					self.setVel0(max_vel) # vel0
					self.setVel1(0) #vel1

			if bumper_L == True:
				# THIS IN NOT IMPLEMENTED YET
				print("TORQUE CONTROL L")
				# if left wheel gets over the bumper then stop
				if abs(wheelL_acc) > 4.0:
					self.setVel1(-vel1*10)
					self.setVel0(vel0)
					bumper_L = False
				else:
				
					# set the new input velocities
					self.setVel0(vel0)
					self.setVel1(vel1)
				
			if bumper_L == False and bumper_R == False:
				# set the new input velocities
				self.setVel0(vel0)
				self.setVel1(vel1)"""
			#############################################3
			
			#######################################
			# TORQUE CONTROL
			# check whether there is a bumper or not
			"""if abs(torqueL) > 60 and dL == 0.0:
				bumper_L = True
				self.setTorque_control_mode1()
				#self.my_odrive.axis1.controller.config.vel_limit = 0.5
				
			if abs(torqueR) > 60 and dR == 0.0:
				bumper_R = True
				self.setTorque_control_mode0()
				#self.my_odrive.axis0.controller.config.vel_limit = 0.5
			
				
			if bumper_R == True:
				print("TORQUE CONTROL R")
				# if right wheel gets over the bumper then stop
				if abs(wheelR_acc) > 4.0:
					#self.my_odrive.axis0.controller.config.vel_gain = self.vel_gain
					#self.my_odrive.axis0.controller.config.vel_integrator_gain = self.vel_integrator_gain
					self.setTorque0(-torque0)
					self.setVelocity_control_mode0()
					self.setTorque0(0)
					
					self.setVel0(vel0)
					self.setVel1(vel1)
					bumper_R = False
					#finish = True
				else:	
					torque0 = torque0 + 10
					print("torqueR: {}".format(torque0))
					self.setTorque0(torque0)
					self.setVel1(0)
					

			if bumper_L == True:
				# THIS IN NOT IMPLEMENTED YET
				print("TORQUE CONTROL L")
				# if left wheel gets over the bumper then stop
				if abs(wheelL_acc) > 4.0:
					self.setVel1(-vel1*10)
					self.setVel0(vel0)
					bumper_L = False
				else:
				
					# set the new input velocities
					self.setVel0(vel0)
					self.setVel1(vel1)
				
			if bumper_L == False and bumper_R == False:
				# set the new input velocities
				self.setVel0(vel0)
				self.setVel1(vel1)"""
				
			#####################################
			
			####################################
			# check whether there is a bumper or not
			if abs(torqueL) > 60 and dL == 0.0:
				bumper_L = True
				enc_start_L = self.getEncoder1()
				
			if abs(torqueR) > 60 and dR == 0.0:
				bumper_R = True
				enc_start_R = self.getEncoder0()
				
			if bumper_L == False and bumper_R == False:
				# set the new input velocities
				self.setVel0(vel0)
				self.setVel1(vel1)
				control_mode_R = 1
				control_mode_L = 1
			else:
				if bumper_R == True and bumper_L == True:
					#print("TORQUE CONTROL R")
					# calculate travelled distance by the wheel
					d_trav_R = (self.getEncoder0() - enc_start_R) * self.circumference
					# if right wheel gets over the bumper then stop
					#if (abs(wheelR_acc) > acc_treshold and abs(self.getVel0()) > vel_thresh) or breakingR or abs(d_trav_R) > travel_threshold :
					if breakingR or abs(d_trav_R) > travel_threshold:
						breakingR = True
						
						self.setTorque_control_mode0()
						self.setTorque0(-self.getTorque0())
						self.my_odrive.axis0.controller.config.vel_gain = self.vel_gain
						self.my_odrive.axis0.controller.config.vel_integrator_gain = self.vel_integrator_gain
						#print("BREAK")
						control_mode_R = 3
						control_mode_L = 2
						
						if self.getVel0() < 0.3:	
							breakingR = False
							self.setVelocity_control_mode0()
							vel0 = 0
							vel1 = 0
							self.setVel0(vel0)
							bumper_R = False
					
					# calculate travelled distance by the wheel
					d_trav_L = (self.getEncoder1() - enc_start_L) * self.circumference	
					# if lefty wheel gets over the bumper then stop
					#if (abs(wheelL_acc) > acc_treshold and abs(self.getVel1()) > vel_thresh) or breakingL or abs(d_trav_R) > travel_threshold:
					if breakingL or abs(d_trav_R) > travel_threshold:
						
						breakingL = True
						
						self.setTorque_control_mode1()
						self.setTorque1(-self.getTorque1())
						self.my_odrive.axis1.controller.config.vel_gain = self.vel_gain
						self.my_odrive.axis1.controller.config.vel_integrator_gain = self.vel_integrator_gain
						#print("BREAK")
						control_mode_R = 2
						control_mode_L = 3
						
						if self.getVel1() < 0.3:	
							breakingL = False
							self.setVelocity_control_mode1()
							vel0 = 0
							vel1 = 0
							self.setVel0(vel0)
							self.setVel1(vel1)
							bumper_L = False
							
					else:	
						self.my_odrive.axis0.controller.config.vel_gain += 10
						self.my_odrive.axis1.controller.config.vel_gain += 10
						#print("vel_gain")
						#print(self.my_odrive.axis0.controller.config.vel_gain)
						self.my_odrive.axis0.controller.config.vel_integrator_gain = 0
						self.my_odrive.axis1.controller.config.vel_integrator_gain = 0
						
						# set the new input velocities
						vel0 = max_vel
						vel1 = max_vel
						
						self.setVel0(vel0) # vel0
						self.setVel1(vel1) #vel1
						control_mode_R = 2
						control_mode_L = 2
					
				#-----------------------------------------------------------------------
				
				elif bumper_R == True:
					#print("bumper R")
					# calculate travelled distance by the wheel
					d_trav_R = (self.getEncoder0() - enc_start_R) * self.circumference
					# if right wheel gets over the bumper then stop
					
					#if (abs(wheelR_acc) > acc_treshold and abs(self.getVel0()) > vel_thresh) or breakingR or abs(d_trav_R) > travel_threshold:
					if abs(d_trav_R) > travel_threshold:
						
						breakingR = True
						
						self.setTorque_control_mode0()
						self.setTorque0(-self.getTorque0())
						
						self.my_odrive.axis0.controller.config.vel_gain = self.vel_gain
						self.my_odrive.axis0.controller.config.vel_integrator_gain = self.vel_integrator_gain
						#print("BREAK")
						control_mode_R = 3
						control_mode_L = 1
						
						if self.getVel0() < 0.3:	
							breakingR = False
							self.setVelocity_control_mode0()
							vel0 = 0
							vel1 = 0
							self.setVel0(vel0)
							self.setVel1(vel1)
							bumper_R = False
						
					else:	
						self.my_odrive.axis0.controller.config.vel_gain += 10
						#print("vel_gain")
						#print(self.my_odrive.axis0.controller.config.vel_gain)
						self.my_odrive.axis0.controller.config.vel_integrator_gain = 0
						
						# set the new input velocities
						vel0 = max_vel
						#vel1 = 0
						
						self.setVel0(vel0) # vel0
						self.setVel1(vel1) #vel1
						control_mode_R = 2
						control_mode_L = 1
							
				#------------------------------------------------------------------	

				elif bumper_L == True:
					#print("bumper L")
					# calculate travelled distance by the wheel
					d_trav_L = (self.getEncoder1() - enc_start_L) * self.circumference	
					# if right wheel gets over the bumper then stop
					#if (abs(wheelL_acc) > acc_treshold and abs(self.getVel1()) > vel_thresh) or breakingL or abs(d_trav_L) > travel_threshold:
					if breakingL or abs(d_trav_L) > travel_threshold:
						breakingL = True
						
						self.setTorque_control_mode1()
						self.setTorque1(-self.getTorque1())
						self.my_odrive.axis1.controller.config.vel_gain = self.vel_gain
						self.my_odrive.axis1.controller.config.vel_integrator_gain = self.vel_integrator_gain
						#print("BREAK")
						control_mode_R = 1
						control_mode_L = 3
						
						if self.getVel1() < 0.3:	
							breakingL = False
							self.setVelocity_control_mode1()
							vel0 = 0
							vel1 = 0
							self.setVel0(vel0)
							self.setVel1(vel1)
							bumper_L = False
							#finish = True
					else:	
						self.my_odrive.axis1.controller.config.vel_gain += 10
						#print("vel_gain")
						#print(self.my_odrive.axis1.controller.config.vel_gain)
						self.my_odrive.axis1.controller.config.vel_integrator_gain = 0
						
						# set the new input velocities
						vel1 = max_vel
						#vel0 = 0
						
						self.setVel0(vel0) # vel0
						self.setVel1(vel1) #vel1
						control_mode_L = 2
						control_mode_R = 1
				
			
			#############################################3
				
			
			
			# set the new input velocities
			#self.setVel0(vel0)
			#self.setVel1(vel1)
			
			#Update
			e_prev_angle = e_angle
			#e_prev_pos = e_pos
			enc0_prev = enc0
			enc1_prev = enc1
			x_prev = self.current_pos.x
			y_prev = self.current_pos.y
			
			# publish data
			self.pub_pos.publish(self.current_pos)
			self.pub_vel.publish(self.current_vel)
			self.pub_ha.publish(self.fi)
			self.pub_des_ha.publish(math.atan2(dy, dx))
			self.pub_torque_R.publish(self.getTorque0()/self.torque_constant)
			self.pub_torque_L.publish(self.getTorque1()/self.torque_constant)
			self.pub_vel_R.publish(self.getVel0())
			self.pub_vel_L.publish(self.getVel1())
			self.pub_acc_R.publish(wheelR_acc)
			self.pub_acc_L.publish(wheelL_acc)
			self.pub_R_enc.publish(enc0)
			self.pub_L_enc.publish(enc1)
			self.pub_des_vel_L.publish(vel1)
			self.pub_des_vel_R.publish(vel0)
			self.pub_current_set_L.publish(self.getCurrentSetP1())
			self.pub_current_set_R.publish(self.getCurrentSetP0())
			self.cont_mode_L.publish(control_mode_L)
			self.cont_mode_R.publish(control_mode_R)
  			
			#time.sleep(dt)
			
		
		# STOP
		self.setVel0(0)
		self.setVel1(0)
		
		print("FINISHED")
		print("current x pos: {}".format(self.current_pos.x))
		print("current y pos: {}".format(self.current_pos.y))
		
		
		
	"""
	Differential drive 2D torque position control
	
	@params:
		desired_x: desired x position in [m]
		desired_y: desired y position in [m]
	"""
	def diff_drive_torque_control(self, desired_x, desired_y):
		
		# set control mode to torque control
		self.setTorque_control_mode()
		
		# PID params for the heading angle control
		Kp_a = 164 #15 #7 10 108
		Ki_a = 24   #1	16
		Kd_a = 60  #10 #5.0 7 40
		
		# PID params for the heading angle control
		Kp_v = self.vel_gain #7 #15 #7 10
		Ki_v = self.vel_integrator_gain #5   #1
		
		
		PI_Iref_R = 0.0 #self.getTorque0() 
		prev_sum_R = 0
		
		PI_Iref_L = 0.0 #self.getTorque1() / self.torque_constant
		prev_sum_L = 0
		
		error_sum_L = 0.0
		error_sum_R = 0.0
		
		
		vel0_prev = self.getVel0()
		vel1_prev = self.getVel1()
		
		epsilon = 0.01
		
		
		enc0_prev = self.getEncoder0()
		enc1_prev = self.getEncoder1()
		
		e_prev_angle = 0
		ei_angle = 0
		dt = 0.01
		
		# for velocity approximation
		x_prev = self.current_pos.x
		y_prev = self.current_pos.y
		
		wheelR_vel_prev = 0
		wheelL_vel_prev = 0
		
		
		# starting speed
		avg_torque = 50
		
		
		# initial distance
		dx = desired_x - self.current_pos.x
		dy = desired_y - self.current_pos.y
		
		while epsilon < np.sqrt((dx)**2 + (dy)**2) and not rospy.is_shutdown():
			
			enc0 = self.getEncoder0()
			enc1 = self.getEncoder1()
			# distance the left wheel travelled
			dR = (enc0 - enc0_prev) * self.circumference #/ self.ticks
			# distance the right wheel travelled
			dL = (enc1 - enc1_prev) * self.circumference #/ self.ticks
			
			# angle of the leg from the hall sensors (this should be changed later to the leg angle ecoder data I guess)
			self.fi += 1/self.wheel_dist * (dR - dL)
			
			fi_deg = self.fi/np.pi*180

			#print("calculated heading angles: {}".format(fi_deg))
			
			
			# current position calculated from the hall data 
			self.current_pos.x = self.current_pos.x + 1/2 * (dR + dL) * np.cos(self.fi)
			self.current_pos.y = self.current_pos.y + 1/2 * (dR + dL) * np.sin(self.fi)
			
			#store pos
			self.pos_x_list.append(self.current_pos.x)
			self.pos_y_list.append(self.current_pos.y)
			
			#print("current x: {} m".format(self.current_pos.x))
			#print("current y: {} m".format(self.current_pos.y))
			
			# current position difference
			dx = desired_x - self.current_pos.x
			dy = desired_y - self.current_pos.y
			
			#distance travelled ion this time step
			distx = self.current_pos.x - x_prev
			disty = self.current_pos.y - y_prev
			
			# leg velocity calculation
			self.current_vel.x = distx / dt
			self.current_vel.y = disty / dt
			
			
			print("dx: {} m".format(dx))
			print("dy: {} m".format(dy))
			print("distance: {}".format(np.sqrt(dx**2+dy**2)))

			# -180/180 orientation fixing
			desired_angle = math.atan2(dy, dx)
			if abs(desired_angle + 2 * np.pi - self.fi) < abs(desired_angle - self.fi):
				desired_angle += 2 * np.pi

			# error
			e_angle = desired_angle - self.fi
			ed_angle = (e_angle - e_prev_angle) / dt
			ei_angle = ei_angle + e_angle*dt
			
			print("desired angle {}".format(math.atan2(dy, dx)))
			print("current angle {}".format(self.fi))
			
			# PID equation
			fi_dot = Kp_a*e_angle + Ki_a* ei_angle + Kd_a * ed_angle
			
			
			# next input velocity calculation
			torque0 = avg_torque + 1/2 * self.wheel_dist * fi_dot
			torque1 = avg_torque - 1/2 * self.wheel_dist * fi_dot
                        

			# limit the input velocities
			torque0 = max(min(torque0, self.torque_lim), -self.torque_lim)
			torque1 = max(min(torque1, self.torque_lim), -self.torque_lim)


			#if np.sqrt((dx)**2 + (dy)**2) < 0.3:
			#	torque0 = torque0 * 0.5
			#	torque0 = torque1 * 0.5
			
			print("right wheel torque: {}".format(torque0))
			print("left wheel torque: {}".format(torque1))
			
			wheelR_vel = self.getVel0()	
			wheelR_acc = (wheelR_vel - wheelR_vel_prev) / dt
			wheelR_vel_prev = wheelR_vel
			torqueR = self.getTorque0()
			
			wheelL_vel = self.getVel1()	
			wheelL_acc = (wheelL_vel - wheelL_vel_prev) / dt
			wheelL_vel_prev = wheelL_vel
			torqueL = self.getTorque1()
			
			# Torque control with foxboro
			"""v_error_R = vel0 - self.getVel0()
			error_sum_R += v_error_R
			PI_Iref_R = Ki_v * error_sum_R + Kp_v * v_error_R
			
			v_error_L = vel1 - self.getVel1()
			error_sum_L += v_error_L
			PI_Iref_L = -1 * (Ki_v * error_sum_L + Kp_v * v_error_L)"""
			
			"""v_error_R = vel0 - self.getVel0()
			foxboro_feedback_R = prev_sum_R
			prev_sum_R = (PI_Iref_R - foxboro_feedback_R) * Ki_v + foxboro_feedback_R
			PI_Iref_R = foxboro_feedback_R + Kp_v * v_error_R
			
			v_error_L = vel1 - self.getVel1()
			foxboro_feedback_L = prev_sum_L
			prev_sum_L = (PI_Iref_L - foxboro_feedback_L) * Ki_v + foxboro_feedback_L
			PI_Iref_L = -1* (foxboro_feedback_L + Kp_v * v_error_L)
			
			print(PI_Iref_L)
			print(PI_Iref_R)
			
			# limit the input velocities
			PI_Iref_R = max(min(PI_Iref_R, self.torque_lim), -self.torque_lim)
			PI_Iref_L = max(min(PI_Iref_L, self.torque_lim), -self.torque_lim)"""
			
			if abs(self.current_vel.x) > 0.5 or abs(self.current_vel.y) > 0.5:
				self.setTorque0(torque0*0.5)
				self.setTorque1(torque1*0.5)
			else:
				self.setTorque0(torque0)
				self.setTorque1(torque1)
			
			#self.setTorque0(torque0)
			#self.setTorque1(torque1)
			
			
			#Update
			e_prev_angle = e_angle
			#e_prev_pos = e_pos
			enc0_prev = enc0
			enc1_prev = enc1
			x_prev = self.current_pos.x
			y_prev = self.current_pos.y
			
			# publish data
			self.pub_pos.publish(self.current_pos)
			self.pub_vel.publish(self.current_vel)
			self.pub_ha.publish(self.fi)
			self.pub_des_ha.publish(math.atan2(dy, dx))
			self.pub_torque_R.publish(self.getTorque0()/self.torque_constant)
			self.pub_torque_L.publish(self.getTorque1()/self.torque_constant)
			self.pub_vel_R.publish(self.getVel0())
			self.pub_vel_L.publish(self.getVel1())
			self.pub_acc_R.publish(wheelR_acc)
			self.pub_acc_L.publish(wheelL_acc)
			self.pub_R_enc.publish(enc0)
			self.pub_L_enc.publish(enc1)
			#self.pub_des_vel_L.publish(vel1)
			#self.pub_des_vel_R.publish(vel0)
			self.pub_current_set_L.publish(self.getCurrentSetP1())
			self.pub_current_set_R.publish(self.getCurrentSetP0())
  			
			time.sleep(dt)
			
		
		# STOP
		# set control mode to velocity control
		self.setVelocity_control_mode()
		self.setVel0(0)
		self.setVel1(0)
		
		print("FINISHED")
		
		
		
	"""
	Rough terrain 2D hibrid position control
	
	@params:
		desired_x: desired x position in [m]
		desired_y: desired y position in [m]
	"""
	def rough_terrain_hibrid_control(self, desired_x, desired_y):
		
		# set control mode to velocity control
		self.setVelocity_control_mode()
		
		# PID params for the heading angle control
		Kp_a = 7 #15 #7 10
		Ki_a = 1   #1
		Kd_a = 5  #10 #5.0 7
		
		epsilon = 0.01
		
		
		enc0_prev = self.getEncoder0()
		enc1_prev = self.getEncoder1()
		
		e_prev_angle = 0
		ei_angle = 0
		dt = 0.01
		
		# for velocity approximation
		x_prev = self.current_pos.x
		y_prev = self.current_pos.y
		
		
		# starting speed
		avg_vel = 0.3
		max_vel = 0.4
		vel_thresh = 0.5
		# acc threshold for bumper stuff
		acc_treshold = 4.0
		travel_threshold = 0.15
		
		self.setVel0(avg_vel)
		self.setVel1(avg_vel)
		
		# initial distance
		dx = desired_x - self.current_pos.x
		dy = desired_y - self.current_pos.y
		
		bumper_L = False
		bumper_R = False
		wheelR_vel_prev = 0
		wheelL_vel_prev = 0
		torque0 = 50
		torque1 = 50
		breakingR = False
		breakingL = False
		
		# control mode of the wheels
		control_mode_R = 1
		control_mode_L = 1
		
		# distance travelled by the wheels since the bumper is detected
		d_trav_L = 0
		d_trav_R = 0
		
		finish = False
		
		time_prev = rospy.get_time()
		
		while epsilon < np.sqrt((dx)**2 + (dy)**2) and not rospy.is_shutdown() and not finish:
			
			enc0 = self.getEncoder0()
			enc1 = self.getEncoder1()
			# distance the left wheel travelled
			dR = (enc0 - enc0_prev) * self.circumference #/ self.ticks
			# distance the right wheel travelled
			dL = (enc1 - enc1_prev) * self.circumference #/ self.ticks
			
			# angle of the leg from the hall sensors (this should be changed later to the leg angle ecoder data I guess)
			self.fi += 1/self.wheel_dist * (dR - dL)
			
			fi_deg = self.fi/np.pi*180

			#print("calculated heading angles: {}".format(fi_deg))
			
			
			# current position calculated from the hall data 
			self.current_pos.x = self.current_pos.x + 1/2 * (dR + dL) * np.cos(self.fi)
			self.current_pos.y = self.current_pos.y + 1/2 * (dR + dL) * np.sin(self.fi)
			
			#store pos
			self.pos_x_list.append(self.current_pos.x)
			self.pos_y_list.append(self.current_pos.y)
			
			#print("current x: {} m".format(self.current_pos.x))
			#print("current y: {} m".format(self.current_pos.y))
			
			# current position difference
			dx = desired_x - self.current_pos.x
			dy = desired_y - self.current_pos.y
			
			#distance travelled ion this time step
			distx = self.current_pos.x - x_prev
			disty = self.current_pos.y - y_prev
			
			time_now = rospy.get_time()
			d_time = time_now - time_prev
			time_prev = time_now
			# leg velocity calculation
			self.current_vel.x = distx / d_time #dt
			self.current_vel.y = disty / d_time #dt
			
			
			#print("dx: {} m".format(dx))
			#print("dy: {} m".format(dy))
			print("distance: {}".format(np.sqrt(dx**2+dy**2)))

			# -180/180 orientation fixing
			desired_angle = math.atan2(dy, dx)
			if abs(desired_angle + 2 * np.pi - self.fi) < abs(desired_angle - self.fi):
				desired_angle += 2 * np.pi

			# error
			e_angle = desired_angle - self.fi
			ed_angle = (e_angle - e_prev_angle) / d_time #dt
			ei_angle = ei_angle + e_angle*dt
			
			#print("desired angle {}".format(math.atan2(dy, dx)))
			#print("current angle {}".format(self.fi))
			
			# PID equation
			fi_dot = Kp_a*e_angle + Ki_a* ei_angle + Kd_a * ed_angle
			
			
			# next input velocity calculation
			vel0 = avg_vel + 1/2 * self.wheel_dist * fi_dot
			vel1 = avg_vel - 1/2 * self.wheel_dist * fi_dot
			     
                  
			# limit the input velocities
			vel0 = max(min(vel0, max_vel), -max_vel)
			vel1 = max(min(vel1, max_vel), -max_vel)


			if np.sqrt((dx)**2 + (dy)**2) < 0.3:
				vel0 = vel0 * 0.5
				vel1 = vel1 * 0.5
			
			wheelR_vel = self.getVel0()	
			wheelR_acc = (wheelR_vel - wheelR_vel_prev) / d_time #dt
			wheelR_vel_prev = wheelR_vel
			torqueR = self.getTorque0()
			
			wheelL_vel = self.getVel1()	
			wheelL_acc = (wheelL_vel - wheelL_vel_prev) / d_time #dt
			wheelL_vel_prev = wheelL_vel
			torqueL = self.getTorque1()
			
			####################################
			# check whether there is a bumper or not
			if abs(torqueL) > 60 and dL == 0.0 and not bumper_L:
				bumper_L = True
				enc_start_L = self.getEncoder1()
				# switch to torque controller
				self.setTorque_control_mode1()
				torque1 = self.getTorque1()
				
			if abs(torqueR) > 60 and dR == 0.0 and not bumper_R:
				bumper_R = True
				enc_start_R = self.getEncoder0()
				# switch to torque controller
				self.setTorque_control_mode0()
				torque0 = self.getTorque0()
				
			if bumper_L == False and bumper_R == False:
				# set the new input velocities
				self.setVel0(vel0)
				self.setVel1(vel1)
				control_mode_R = 1
				control_mode_L = 1
			else:
				if bumper_R == True and bumper_L == True:
					#print("TORQUE CONTROL R")
					# calculate travelled distance by the wheel
					d_trav_R = (self.getEncoder0() - enc_start_R) * self.circumference
					# if right wheel gets over the bumper then stop
					#if (abs(wheelR_acc) > acc_treshold and abs(self.getVel0()) > vel_thresh) or breakingR or abs(d_trav_R) > travel_threshold :
					if breakingR or abs(d_trav_R) > travel_threshold:
						breakingR = True
						
						self.setTorque0(-self.getTorque0())
						#print("BREAK")
						control_mode_R = 3
						control_mode_L = 2
						
						if self.getVel0() < 0.3:	
							breakingR = False
							self.setVelocity_control_mode0()
							vel0 = 0
							vel1 = 0
							self.setVel0(vel0)
							bumper_R = False
					
					# calculate travelled distance by the wheel
					d_trav_L = (self.getEncoder1() - enc_start_L) * self.circumference	
					# if lefty wheel gets over the bumper then stop
					#if (abs(wheelL_acc) > acc_treshold and abs(self.getVel1()) > vel_thresh) or breakingL or abs(d_trav_R) > travel_threshold:
					if breakingL or abs(d_trav_R) > travel_threshold:
						
						breakingL = True
						
						self.setTorque1(-self.getTorque1())
						#print("BREAK")
						control_mode_R = 2
						control_mode_L = 3
						
						if self.getVel1() < 0.3:	
							breakingL = False
							self.setVelocity_control_mode1()
							vel0 = 0
							vel1 = 0
							self.setVel0(vel0)
							self.setVel1(vel1)
							bumper_L = False
							
					else:

						
						
						# check the direction
						if torque0 > 0:
							torque0 += 10 
						else:
							torque0 -= 10 
						
						if torque1 > 0:
							torque1 += 10 
						else:
							torque1 -= 10 
							
						#limit input torque
						# limit the input velocities
						torque0 = max(min(torque0, self.torque_lim), -self.torque_lim)
						torque1 = max(min(torque1, self.torque_lim), -self.torque_lim)
						
							
						# set the new input velocities
						self.setTorque0(torque0) # vel0
						self.setTorque1(torque1) #vel1
						control_mode_R = 2
						control_mode_L = 2
					
				#-----------------------------------------------------------------------
				
				elif bumper_R == True:
					self.setTorque_control_mode0()
					#print("bumper R")
					# calculate travelled distance by the wheel
					d_trav_R = (self.getEncoder0() - enc_start_R) * self.circumference
					# if right wheel gets over the bumper then stop
					
					#if (abs(wheelR_acc) > acc_treshold and abs(self.getVel0()) > vel_thresh) or breakingR or abs(d_trav_R) > travel_threshold:
					if abs(d_trav_R) > travel_threshold:
						
						breakingR = True
						
						
						#print("BREAK")
						control_mode_R = 3
						control_mode_L = 1
						
						if self.getVel0() < 0.3:	
							breakingR = False
							self.setVelocity_control_mode0()
							vel0 = 0
							vel1 = 0
							self.setVel0(vel0)
							self.setVel1(vel1)
							bumper_R = False
						else:
							self.setTorque0(-self.getTorque0())
							
						
						
					else:	
						
						# check the direction
						if torque0 > 0:
							torque0 += 10 
						else:
							torque0 -= 10 
							
						print(torque0)
						# limit the input torque
						torque0 = max(min(torque0, self.torque_lim), -self.torque_lim)
						print("after lim")
						print(torque0)
						print(self.getTorque0())
						print(self.getCurrentSetP0())
						
						self.setTorque0(torque0)
						self.setVel1(vel1) #vel1
						control_mode_R = 2
						control_mode_L = 1
							
				#------------------------------------------------------------------	

				elif bumper_L == True:
					#print("bumper L")
					# calculate travelled distance by the wheel
					d_trav_L = (self.getEncoder1() - enc_start_L) * self.circumference	
					# if right wheel gets over the bumper then stop
					#if (abs(wheelL_acc) > acc_treshold and abs(self.getVel1()) > vel_thresh) or breakingL or abs(d_trav_L) > travel_threshold:
					if breakingL or abs(d_trav_L) > travel_threshold:
						breakingL = True
						
						#print("BREAK")
						control_mode_R = 1
						control_mode_L = 3
						
						if self.getVel1() < 0.3:	
							breakingL = False
							self.setVelocity_control_mode1()
							vel0 = 0
							vel1 = 0
							self.setVel0(vel0)
							self.setVel1(vel1)
							bumper_L = False
						else:
							self.setTorque1(-self.getTorque1())
						
					
					else:
						
						# check the direction
						if torque1 > 0:
							torque1 += 10 
						else:
							torque1 -= 10 
							
						# limit the input torque
						torque1 = max(min(torque1, self.torque_lim), -self.torque_lim) 
						
						self.setVel0(vel0) # vel0
						self.setTorque1(torque1)
						control_mode_L = 2
						control_mode_R = 1
				
			
			#############################################3
				
			
			
			# set the new input velocities
			#self.setVel0(vel0)
			#self.setVel1(vel1)
			
			#Update
			e_prev_angle = e_angle
			#e_prev_pos = e_pos
			enc0_prev = enc0
			enc1_prev = enc1
			x_prev = self.current_pos.x
			y_prev = self.current_pos.y
			
			# publish data
			self.pub_pos.publish(self.current_pos)
			self.pub_vel.publish(self.current_vel)
			self.pub_ha.publish(self.fi)
			self.pub_des_ha.publish(math.atan2(dy, dx))
			self.pub_torque_R.publish(self.getTorque0()/self.torque_constant)
			self.pub_torque_L.publish(self.getTorque1()/self.torque_constant)
			self.pub_vel_R.publish(self.getVel0())
			self.pub_vel_L.publish(self.getVel1())
			self.pub_acc_R.publish(wheelR_acc)
			self.pub_acc_L.publish(wheelL_acc)
			self.pub_R_enc.publish(enc0)
			self.pub_L_enc.publish(enc1)
			self.pub_des_vel_L.publish(vel1)
			self.pub_des_vel_R.publish(vel0)
			self.pub_current_set_L.publish(self.getCurrentSetP1())
			self.pub_current_set_R.publish(self.getCurrentSetP0())
			self.cont_mode_L.publish(control_mode_L)
			self.cont_mode_R.publish(control_mode_R)
  			
			#time.sleep(dt)
			
		
		# STOP
		self.setVel0(0)
		self.setVel1(0)
		
		print("FINISHED")
		print("current x pos: {}".format(self.current_pos.x))
		print("current y pos: {}".format(self.current_pos.y))
		


