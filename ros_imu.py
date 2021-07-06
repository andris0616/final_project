#!/usr/bin/env python3


# SPDX-FileCopyrightText: 2020 Bryan Siepert, written for Adafruit Industries
#
# SPDX-License-Identifier: Unlicense
import time
import board
import busio
import rospy
from geometry_msgs.msg import Vector3
#from tf.transformations import quaternion_from_euler

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

leg_accel = Vector3()
leg_gyro = Vector3()
leg_magneto = Vector3()

if __name__ == '__main__':
    rospy.init_node('IMU_node')
    pub_accel = rospy.Publisher('leg_acceleration', Vector3, queue_size = 10)
    pub_gyro = rospy.Publisher('leg_gyro', Vector3, queue_size = 10)
    pub_magneto = rospy.Publisher('leg_magnetometer', Vector3, queue_size = 10)

    while not rospy.is_shutdown():

        time.sleep(0.01)
        # Acceleration:
        print("Acceleration:")
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
        print("")
        
        
