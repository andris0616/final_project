#!/usr/bin/env python3

# SPDX-FileCopyrightText: 2020 Bryan Siepert, written for Adafruit Industries
#
# SPDX-License-Identifier: Unlicense
import time
import board
import busio
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR,
)
from adafruit_bno08x.i2c import BNO08X_I2C

i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
bno = BNO08X_I2C(i2c, address=0x4B)

bno.enable_feature(BNO_REPORT_ACCELEROMETER)
bno.enable_feature(BNO_REPORT_GYROSCOPE)
bno.enable_feature(BNO_REPORT_MAGNETOMETER)
bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)

import numpy as np
import math
import rospy
from std_msgs.msg import Bool,Int32, Float32
from geometry_msgs.msg import Vector3
#from tf.transformations import quaternion_from_euler


TL_leg_accel = Vector3()
TL_leg_gyro = Vector3()
TL_leg_magneto = Vector3()
TL_leg_rotation = Vector3()
avg_accel = Vector3()
avg_gyro = Vector3()
avg_rot = Vector3()

def euler_from_quaternion(x, y, z, w):
        """
        ref: https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians


if __name__ == '__main__':
    print("hello")
    rospy.init_node('IMU_node')
    pub_accel = rospy.Publisher('TL_leg_acceleration', Vector3, queue_size = 10)
    pub_gyro = rospy.Publisher('TL_leg_gyro', Vector3, queue_size = 10)
    pub_magneto = rospy.Publisher('TL_leg_magnetometer', Vector3, queue_size = 10)
    pub_rot = rospy.Publisher('TL_leg_rotation', Vector3, queue_size = 10)
    pub_IMU = rospy.Publisher('TL_IMU_ready', Bool, queue_size = 1)
    
    rate = rospy.Rate(100)
    
    IMU_ready = False
    
    accel = np.zeros((3, 100))
    gyro = np.zeros((3, 100))
    rotation = np.zeros((3, 100))
    
    for i in range(100):
        
        TL_leg_accel.x, TL_leg_accel.y, TL_leg_accel.z = bno.acceleration
        TL_leg_gyro.x, TL_leg_gyro.y, TL_leg_gyro.z = bno.gyro
        
        quat_x, quat_y, quat_z, quat_w = bno.quaternion
        
        TL_leg_rotation.x, TL_leg_rotation.y, TL_leg_rotation.z = euler_from_quaternion(quat_x, quat_y, quat_z, quat_w)
        
        accel[0,i] = TL_leg_accel.x
        accel[1,i] = TL_leg_accel.y
        accel[2,i] = TL_leg_accel.z
        
        gyro[0,i] = TL_leg_gyro.x
        gyro[1,i] = TL_leg_gyro.y
        gyro[2,i] = TL_leg_gyro.z
        
        rotation[0,i] = TL_leg_rotation.x
        rotation[1,i] = TL_leg_rotation.y
        rotation[2,i] = TL_leg_rotation.z
        
    # calculate the average of these values for zeroing down the IMU    
    avg_accel.x = np.sum(accel[0,:]) /100
    avg_accel.y = np.sum(accel[1,:]) /100
    avg_accel.z = np.sum(accel[2,:]) /100
    
    avg_gyro.x = np.sum(gyro[0,:]) /100
    avg_gyro.y = np.sum(gyro[1,:]) /100
    avg_gyro.z = np.sum(gyro[2,:]) /100
    
    avg_rot.x = np.sum(rotation[0,:]) /100
    avg_rot.y = np.sum(rotation[1,:]) /100
    avg_rot.z = np.sum(rotation[2,:]) /100
    
    # publish that IMU is ready to use
    pub_IMU.publish(Bool(True))
    print("IMU is ready!!!")

    while not rospy.is_shutdown():

        #time.sleep(0.001)
        # Acceleration:
        """print("Acceleration:")
        accel_x, accel_y, accel_z = bno.acceleration  # pylint:disable=no-member
        print("X: %0.6f  Y: %0.6f Z: %0.6f  m/s^2" % (accel_x, accel_y, accel_z))
        print("")
    
    
        # Gyro
        print("Gyro:")
        gyro_x, gyro_y, gyro_z = bno.gyro  # pylint:disable=no-member
        print("X: %0.6f  Y: %0.6f Z: %0.6f rads/s" % (gyro_x, gyro_y, gyro_z))
        print("")
        
        # Magnetometer
        print("Magnetometer:")
        mag_x, mag_y, mag_z = bno.magnetic  # pylint:disable=no-member
        print("X: %0.6f  Y: %0.6f Z: %0.6f  m/s^2" % (mag_x, mag_y, mag_z))
        print("")

        # Rotation Vector
        print("Rotation Vector Quaternion:")
        quat_i, quat_j, quat_k, quat_real = bno.quaternion  # pylint:disable=no-member
        print("I: %0.6f J: %0.6f  K: %0.6f Real: %0.6f rads/s" % (quat_i, quat_j, quat_k, quat_real))
        print("")"""
        
        accel_x, accel_y, accel_z = bno.acceleration
        TL_leg_accel.y = accel_x - avg_accel.x
        TL_leg_accel.z = accel_y - avg_accel.y
        TL_leg_accel.x = accel_z - avg_accel.z
        
        gyro_x, gyro_y, gyro_z = bno.gyro
        TL_leg_gyro.y = gyro_x - avg_gyro.x
        TL_leg_gyro.z = gyro_y - avg_gyro.y
        TL_leg_gyro.x = gyro_z - avg_gyro.z
        
        quat_x, quat_y, quat_z, quat_w = bno.quaternion
        
        rot_x, rot_y, rot_z = euler_from_quaternion(quat_x, quat_y, quat_z, quat_w)
        TL_leg_rotation.y = rot_x - avg_rot.x
        TL_leg_rotation.z = rot_y - avg_rot.y
        TL_leg_rotation.x = rot_z - avg_rot.z
        #print(euler_from_quaternion(quat_x, quat_y, quat_z, quat_w))
        
        pub_accel.publish(TL_leg_accel)
        pub_gyro.publish(TL_leg_gyro)
        pub_rot.publish(TL_leg_rotation)
        
        rate.sleep()
        
        
