#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool,Int32, Float32
import RPi.GPIO as GPIO
import Encoder

#from encoder import Encoder

print('hello')

enc = Encoder.Encoder(16, 20)
#enc = Encoder(16, 20)

MAGNET_GPIO = 12

if __name__ == '__main__':
    rospy.init_node('heading_angle_publisher')
    pub = rospy.Publisher('heading_angle', Float32, queue_size = 10)
    
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(MAGNET_GPIO, GPIO.IN, pull_up_down = GPIO.PUD_UP)
    
    full_rot_ticks = 4700 #5079.6

    tick_prev = 0
    current_angle = 0
    
    rate = rospy.Rate(100)
    
    while not rospy.is_shutdown():
        magnet_state = GPIO.input(MAGNET_GPIO)
        """if magnet_state == 0:
            
            print('reset!!!')
            print(magnet_state)
            enc.position = 0
            enc.state = 0
            print(enc.state)
            #enc.write(0)"""
        tick = enc.read()
        
        print(tick)
        dtick = tick - tick_prev

        dangle = 360/full_rot_ticks * dtick

        current_angle = current_angle + dangle
        
        """if current_angle > 360:
            current_angle = current_angle - 360
        elif current_angle < -360:
            current_angle = current_angle + 360"""
            

        #print(-current_angle)
        #print(tick)
            
        pub.publish(-current_angle)
        rate.sleep()
        
        #update
        tick_prev = tick
            
        
        
        
    """while not rospy.is_shutdown():
        magnet_state = GPIO.input(MAGNET_GPIO)
        if magnet_state == 0:
            
            print('reset!!!')
            print(magnet_state)
            enc.retsetValue()
            
        print('value is {}'.format(enc.getValue()))
            
       
        
        pub.publish(enc.getValue())
        rate.sleep()"""
    
