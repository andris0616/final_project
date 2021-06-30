#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool,Int32
import RPi.GPIO as GPIO
import Encoder

from encoder import Encoder

print('hello')

#enc = Encoder.Encoder(16, 20)
enc = Encoder(16, 20)

MAGNET_GPIO = 12

if __name__ == '__main__':
    rospy.init_node('encoder_publisher')
    pub = rospy.Publisher('encoder_state', Int32, queue_size = 10)
    
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(MAGNET_GPIO, GPIO.IN, pull_up_down = GPIO.PUD_UP)
    

    
    rate = rospy.Rate(10)
    
    '''while not rospy.is_shutdown():
        magnet_state = GPIO.input(MAGNET_GPIO)
        if magnet_state == 0:
            
            print('reset!!!')
            print(magnet_state)
            enc.position = 0
            enc.state = 0
            print(enc.state)
            #enc.write(0)
        print(enc.read())
        
        pub.publish(enc.read())
        rate.sleep()'''
        
    while not rospy.is_shutdown():
        magnet_state = GPIO.input(MAGNET_GPIO)
        if magnet_state == 0:
            
            print('reset!!!')
            print(magnet_state)
            enc.retsetValue()
            
        print('value is {}'.format(enc.getValue()))
            
       
        
        pub.publish(enc.getValue())
        rate.sleep()
    
