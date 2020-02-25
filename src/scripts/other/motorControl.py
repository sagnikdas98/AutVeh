#!/usr/bin/env python
import RPi.GPIO as GPIO
import rospy
from std_msgs.msg import String

from std_msgs.msg import Int16
from std_msgs.msg import Float32

GPWML = 13
GDIRNL = 5

GPWMR = 12
GDIRNR = 6

def motorLeftCall(msg):
    if(msg.data > 0):
	GPIO.output(GDIRNL, GPIO.LOW)
    else:
         GPIO.output(GDIRNL, GPIO.HIGH)
    pwm_per = abs(msg.data)
    pwml.ChangeDutyCycle(pwm_per)
    rospy.loginfo("left")

def motorRightCall(msg):
    if(msg.data > 0):
        GPIO.output(GDIRNR, GPIO.LOW)
    else:
	GPIO.output(GDIRNR, GPIO.HIGH)
    pwm_per = abs(msg.data)
    pwmr.ChangeDutyCycle(pwm_per)
    rospy.loginfo("right") 

def motorStop(msg):
    global pwml
    global pwmr

    pwml.ChangeDutyCycle(0)
    pwmr.ChangeDutyCycle(0)
    GPIO.output(5, GPIO.LOW)
    GPIO.output(6, GPIO.LOW)

def motor():
    while not rospy.is_shutdown():
        rospy.spin()

rospy.init_node('motorControl', anonymous=True)
rospy.Subscriber("lmotor_cmd", Float32, motorLeftCall)
rospy.Subscriber("rmotor_cmd", Float32, motorRightCall)
rospy.Subscriber("motorStop", Int16, motorStop)


GPIO.setmode(GPIO.BCM)

GPIO.setup(GPWML, GPIO.OUT)
GPIO.setup(GDIRNL, GPIO.OUT)

GPIO.setup(GPWMR, GPIO.OUT)
GPIO.setup(GDIRNR, GPIO.OUT)

pwml = GPIO.PWM(GPWML, 1000)
pwml.start(0)

pwmr = GPIO.PWM(GPWMR, 1000)
pwmr.start(0)



if __name__ == '__main__':
    
    motor()

'''
Motor Left:
pwm: gpio13
dirn: gpio5

Motor Right:
pwm: gpio12
dirn: gpio6

'''
