#!/usr/bin/env python
import RPi.GPIO as GPIO
import rospy
from std_msgs.msg import Int16

class SpeedEncoder():

    def __init__(self):
        rospy.init_node("speedEncoderClass", anonymous=True)
        self.nodename = rospy.get_name()
        
        rospy.loginfo("%s started" % self.nodename)

        ### initialize variables

        ##front##
        self.counter_valF = 0
        self.error_valF = 0
        self.Encoder_AF = 0
        self.Encoder_BF = 0
        self.oldAF = 0
        self.oldBF = 0

        ##rear##
        self.counter_valR = 0
        self.error_valR = 0
        self.Encoder_AR = 0
        self.Encoder_BR = 0
        self.oldAR = 0
        self.oldBR = 0

        ### get parameters #### 
        self.rate_freq = rospy.get_param('/rate',30)
        self.EncAF = int(rospy.get_param("~EncAF"))
        self.EncBF = int(rospy.get_param("~EncBF"))

        self.EncAR = int(rospy.get_param("~EncAR"))
        self.EncBR = int(rospy.get_param("~EncBR"))
        
        self.pub = rospy.Publisher('wheel', Int16, queue_size=10)
        self.rate = rospy.Rate(self.rate_freq)
        
        ### setup ###
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        GPIO.setup(self.EncAF, GPIO.IN)    
        GPIO.setup(self.EncBF, GPIO.IN)

        GPIO.setup(self.EncAR, GPIO.IN)    
        GPIO.setup(self.EncBR, GPIO.IN)
        
        GPIO.add_event_detect(self.EncAF, GPIO.BOTH, callback = self.frontWheelCb)
        GPIO.add_event_detect(self.EncBF, GPIO.BOTH, callback = self.frontWheelCb)
        
        GPIO.add_event_detect(self.EncAR, GPIO.BOTH, callback = self.rearWheelCb)
        GPIO.add_event_detect(self.EncBR, GPIO.BOTH, callback = self.rearWheelCb)
        
    def frontWheelCb(self, pin):
        self.Encoder_AF = GPIO.input(self.EncAF)  # stores the value of the encoders at time of interrupt
        self.Encoder_BF = GPIO.input(self.EncBF)

        if self.Encoder_AF == self.oldAF and self.Encoder_BF == self.oldBF:
            self.error_valF += 1
        elif (self.Encoder_AF == 1 and self.oldBF == 0) or (self.Encoder_AF == 0 and self.oldBF == 1):
            self.counter_valF += 1
        elif (self.Encoder_AF == 1 and self.oldBF == 1) or (self.Encoder_AF == 0 and self.oldBF == 0):
            self.counter_valF -= 1
        else:
            self.error_valF += 1

        # store the current encoder values as old values to be used as comparison in the next loop
        self.oldAF = self.Encoder_AF 
        self.oldBF = self.Encoder_BF
        
    def rearWheelCb(self, pin):
        self.Encoder_AR = GPIO.input(self.EncAR)  # stores the value of the encoders at time of interrupt
        self.Encoder_BR = GPIO.input(self.EncBR)

        if self.Encoder_AR == self.oldAR and self.Encoder_BR == self.oldBR:
            self.error_valR += 1
        elif (self.Encoder_AR == 1 and self.oldBR == 0) or (self.Encoder_AR == 0 and self.oldBR == 1):
            self.counter_valR += 1
        elif (self.Encoder_AR == 1 and self.oldBR == 1) or (self.Encoder_AR == 0 and self.oldBR == 0):
            self.counter_valR -= 1
        else:
            self.error_valR += 1

        # store the current encoder values as old values to be used as comparison in the next loop
        self.oldAR = self.Encoder_AR 
        self.oldBR = self.Encoder_BR

    def publishCB(self):
        cnt = int((self.counter_valF + self.counter_valR)/2) 

        self.counter_valF = 0
        self.counter_valR = 0

        self.error_valF = 0
        self.error_valR = 0

        self.pub.publish(cnt)

        self.rate.sleep()


if __name__ == '__main__':
    try:
        speedencoder = SpeedEncoder()
        while not rospy.is_shutdown():              
            speedencoder.publishCB()
    except rospy.ROSInterruptException:
        pass
    finally:
        GPIO.cleanup()



