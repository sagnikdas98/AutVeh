2 Arduino Uno are used:
 a. Speed Encoder and Image Capture
 b. Motor Control and IMU
 

sudo chmod a+rw /dev/ttyACM0


2B:
    Msg: 
        <yondarospack::Ard2bmotormsg>
    Topic:
        Ard2bMotortopic
    Publisher: 
        
    Subscriber:
        motor_msg
        

    Msg: 
        <yondarospack::Ard2bResponemsg>
    Topic:
        Ard2bResponetopic
    Publisher: 
        response_pub
    Subscriber:
        

    Msg: 
        <yondarospack::Ard2bmotorstopmsg>
    Topic:
        Ard2bMotorStoptopic
    Publisher: 

    Subscriber:
        motor_stop_msg_sub



    Msg: 
        <yondarospack::Ard2bimumsg>
    Topic:
        Ard2bIMUtopic
    Publisher: 
        imu_pub
    Subscriber:
        

    Msg: 
        <yondarospack::Ard2bimusetmsg>
    Topic:
        Ard2bIMUSettopic
    Publisher: 
         
    Subscriber:
        imu_set_sub

2A:
