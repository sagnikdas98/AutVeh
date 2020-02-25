#!/usr/bin/env python
import RPi.GPIO as GPIO
import rospy
from std_msgs.msg import Int16
import threading 
# import numpy as np

EncALR = 17
EncBLR = 18

EncALF = 27
EncBLF = 22

EncARR = 23
EncBRR = 24

EncARF = 16
EncBRF = 26

counter_val = {}
error_val = {}
old_val = {}

rate_freq = rospy.get_param('~rate',10.0) 

counter_val[(EncALR,EncBLR)] = 0
counter_val[(EncALF,EncBLF)] = 0
counter_val[(EncARR,EncBRR)] = 0
counter_val[(EncARF,EncBRF)] = 0

# error_val[(EncALR,EncBLR)] = 0
# error_val[(EncALF,EncBLF)] = 0
# error_val[(EncARR,EncBRR)] = 0
# error_val[(EncARF,EncBRF)] = 0

old_val[str(EncALR)] = 0
old_val[str(EncBLR)] = 0
old_val[str(EncALF)] = 0
old_val[str(EncBLF)] = 0

old_val[str(EncARR)] = 0
old_val[str(EncBRR)] = 0
old_val[str(EncARF)] = 0
old_val[str(EncBRF)] = 0





#thread function
def encoderCBF():

    global counter_val
    global old_val 
    global EncALR, EncBLR
    global EncALF, EncBLF
    global EncARR, EncBRR
    global EncARF, EncBRF

    while not rospy.is_shutdown():

        #LR
        Encoder_A = GPIO.input(EncALR)  # stores the value of the encoders at time of interrupt
        Encoder_B = GPIO.input(EncBLR)

        if Encoder_A == old_val[str(EncALR)] and Encoder_B == old_val[str(EncBLR)]:
            pass #error_val[(EncBLR,EncBLR)] += 1
        elif (Encoder_A == 1 and old_val[str(EncBLR)] == 0) or (Encoder_A == 0 and old_val[str(EncBLR)] == 1):
            counter_val[(EncALR,EncBLR)] += 1
        elif (Encoder_A == 1 and old_val[str(EncBLR)] == 1) or (Encoder_A == 0 and old_val[str(EncBLR)] == 0):
            counter_val[(EncALR,EncBLR)] -= 1
        else:
            pass #error_val[(EncBLR,EncBLR)] += 1

        old_val[str(EncALR)] = Encoder_A     # store the current encoder values as old values to be used as comparison in the next loop
        old_val[str(EncBLR)] = Encoder_B 

        #LF

        Encoder_A = GPIO.input(EncALF)  # stores the value of the encoders at time of interrupt
        Encoder_B = GPIO.input(EncBLF)


        if Encoder_A == old_val[str(EncALF)] and Encoder_B == old_val[str(EncBLF)]:
            pass #error_val[(EncALF,EncBLF)] += 1
        elif (Encoder_A == 1 and old_val[str(EncBLF)] == 0) or (Encoder_A == 0 and old_val[str(EncBLF)] == 1):
            counter_val[(EncALF,EncBLF)] += 1
        elif (Encoder_A == 1 and old_val[str(EncBLF)] == 1) or (Encoder_A == 0 and old_val[str(EncBLF)] == 0):
            counter_val[(EncALF,EncBLF)] -= 1
        else:
            pass #error_val[(EncALF,EncBLF)] += 1

        old_val[str(EncALF)] = Encoder_A     # store the current encoder values as old values to be used as comparison in the next loop
        old_val[str(EncBLF)] = Encoder_B 

    #right
        #RR

        Encoder_A = GPIO.input(EncARR)  # stores the value of the encoders at time of interrupt
        Encoder_B = GPIO.input(EncBRR)

        if Encoder_A == old_val[str(EncARR)] and Encoder_B == old_val[str(EncBRR)]:
            pass #error_val[(EncARR,EncBRR)] += 1
        elif (Encoder_A == 1 and old_val[str(EncBRR)] == 0) or (Encoder_A == 0 and old_val[str(EncBRR)] == 1):
            counter_val[(EncARR,EncBRR)] += 1
        elif (Encoder_A == 1 and old_val[str(EncBRR)] == 1) or (Encoder_A == 0 and old_val[str(EncBRR)] == 0):
            counter_val[(EncARR,EncBRR)] -= 1
        else:
            pass #error_val[(EncARR,EncBRR)] += 1

        old_val[str(EncARR)] = Encoder_A     # store the current encoder values as old values to be used as comparison in the next loop
        old_val[str(EncBRR)] = Encoder_B 



        #RF
        Encoder_A = GPIO.input(EncARF)  # stores the value of the encoders at time of interrupt
        Encoder_B = GPIO.input(EncBRF)

        if Encoder_A == old_val[str(EncARF)] and Encoder_B == old_val[str(EncBRF)]:
            pass #error_val[(EncARF,EncBRF)] += 1
        elif (Encoder_A == 1 and old_val[str(EncBRF)] == 0) or (Encoder_A == 0 and old_val[str(EncBRF)] == 1):
            counter_val[(EncARF,EncBRF)] += 1
        elif (Encoder_A == 1 and old_val[str(EncBRF)] == 1) or (Encoder_A == 0 and old_val[str(EncBRF)] == 0):
            counter_val[(EncARF,EncBRF)] -= 1
        else:
            pass #error_val[(EncARF,EncBRF)] += 1

        old_val[str(EncARF)] = Encoder_A     # store the current encoder values as old values to be used as comparison in the next loop
        old_val[str(EncBRF)] = Encoder_B 


def publishCB():

    global counter_val
    global error_val 
    global EncALR, EncBLR
    global EncALF, EncBLF
    global EncARR, EncBRR
    global EncARF, EncBRF

    global rate_freq

    rospy.init_node('speedEncoder', anonymous=True)
    pubL = rospy.Publisher('lwheel', Int16, queue_size=10)
    pubR = rospy.Publisher('rwheel', Int16, queue_size=10)
    rate = rospy.Rate(rate_freq)

    tF = threading.Thread(target=encoderCBF)#, daemon=True)
    tF.setDaemon(True)
    tF.start()


    while not rospy.is_shutdown():

        cntLR = counter_val[(EncALR,EncBLR)]
        cntLF = counter_val[(EncALF,EncBLF)]
        cntRR = counter_val[(EncARR,EncBRR)]
        cntRF = counter_val[(EncARF,EncBRF)]

        counter_val[(EncALR,EncBLR)] = 0
        counter_val[(EncALF,EncBLF)] = 0
        counter_val[(EncARR,EncBRR)] = 0
        counter_val[(EncARF,EncBRF)] = 0

        # error_val[(EncALR,EncBLR)] = 0
        # error_val[(EncALF,EncBLF)] = 0
        # error_val[(EncARR,EncBRR)] = 0
        # error_val[(EncARF,EncBRF)] = 0

        cL = int((cntLR + cntLF)/2)
        cR = int((cntRR + cntRF)/2)

        pubL.publish(cL)
        pubR.publish(cR)

        rate.sleep()



GPIO.setmode(GPIO.BCM)

GPIO.setup(EncALR, GPIO.IN)    
GPIO.setup(EncBLR, GPIO.IN) 

GPIO.setup(EncALF, GPIO.IN) 
GPIO.setup(EncBLF, GPIO.IN) 

GPIO.setup(EncARR, GPIO.IN) 
GPIO.setup(EncBRR, GPIO.IN) 

GPIO.setup(EncARF, GPIO.IN) 
GPIO.setup(EncBRF, GPIO.IN) 

        


# GPIO.add_event_detect(EncALR, GPIO.BOTH, callback = encoderCBLR)   # Encoder A  
# GPIO.add_event_detect(EncALF, GPIO.BOTH, callback = encoderCBLF)
# GPIO.add_event_detect(EncARR, GPIO.BOTH, callback = encoderCBRR)
# GPIO.add_event_detect(EncARF, GPIO.BOTH, callback = encoderCBRF)
# Initialize the interrupts - these trigger on the both the rising and falling 


# This is the part of the code which runs normally in the background
if __name__ == "__main__":
    publishCB()
