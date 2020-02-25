#!/usr/bin/env python
import RPi.GPIO as GPIO
import rospy
from std_msgs.msg import Int16
import threading



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

        ##rear##
        self.counter_valR = 0
        self.error_valR = 0
        self.Encoder_AR = 0
        self.Encoder_BR = 0


        ### get parameters #### 
        self.rate_freq = rospy.get_param('/rate',30)
        self.EncAF = int(rospy.get_param("~EncAF"))
        self.EncBF = int(rospy.get_param("~EncBF"))

        self.EncAR = int(rospy.get_param("~EncAR"))
        self.EncBR = int(rospy.get_param("~EncBR"))


        ### setup ###
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        GPIO.setup(self.EncAF, GPIO.IN)    
        GPIO.setup(self.EncBF, GPIO.IN)

        GPIO.setup(self.EncAR, GPIO.IN)    
        GPIO.setup(self.EncBR, GPIO.IN)

        self.tF = threading.Thread(target=self.encoderCBF)#, daemon=True)
        self.tR = threading.Thread(target=self.encoderCBR)#, daemon=True)
        self.tF.setDaemon(True)
        self.tR.setDaemon(True)

        self.pub = rospy.Publisher('wheel', Int16, queue_size=10)
        self.rate = rospy.Rate(self.rate_freq)


    def encoderCBF(self):

        old_val_A = 0
        old_val_B = 0

        while(1):
    
            self.Encoder_AF = GPIO.input(self.EncAF)  # stores the value of the encoders at time of interrupt
            self.Encoder_BF = GPIO.input(self.EncBF)

            if self.Encoder_AF == old_val_A and self.Encoder_BF == old_val_B:
                pass #self.error_valF += 1
            elif (self.Encoder_AF == 1 and old_val_B == 0) or (self.Encoder_AF == 0 and old_val_B == 1):
                self.counter_valF += 1
            elif (self.Encoder_AF == 1 and old_val_B == 1) or (self.Encoder_AF == 0 and old_val_B == 0):
                self.counter_valF -= 1
            else:
                pass #self.error_valF += 1

            old_val_A = self.Encoder_AF # store the current encoder values as old values to be used as comparison in the next loop
            old_val_B = self.Encoder_BF

    def encoderCBR(self):

        old_val_A = 0
        old_val_B = 0

        while(1):
    
            self.Encoder_AR = GPIO.input(self.EncAR)  # stores the value of the encoders at time of interrupt
            self.Encoder_BR = GPIO.input(self.EncBR)

            if self.Encoder_AR == old_val_A and self.Encoder_BR == old_val_B:
                pass #self.error_valR += 1
            elif (self.Encoder_AR == 1 and old_val_B == 0) or (self.Encoder_AR == 0 and old_val_B == 1):
                self.counter_valR += 1
            elif (self.Encoder_AR == 1 and old_val_B == 1) or (self.Encoder_AR == 0 and old_val_B == 0):
                self.counter_valR -= 1
            else:
                pass #self.error_valR += 1

            old_val_A = self.Encoder_AR # store the current encoder values as old values to be used as comparison in the next loop
            old_val_B = self.Encoder_BR

    def publishCB(self):
        
        self.tF.start()
        self.tR.start()

        while not rospy.is_shutdown():

            cnt = int((self.counter_valF + self.counter_valR)/2) 

            self.counter_valF = 0
            self.counter_valR = 0

            self.error_valF = 0
            self.error_valR = 0
            
            self.pub.publish(cnt)
            
            self.rate.sleep()


if __name__ == '__main__':
    """ main """
    try:
        speedencoder = SpeedEncoder()
        speedencoder.publishCB()
    except rospy.ROSInterruptException:
        pass
    finally:
        GPIO.cleanup()


