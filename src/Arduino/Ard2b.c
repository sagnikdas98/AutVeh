#include <AFMotor.h>
#include<Wire.h>  
#define LR 4
#define LF 3
#define RF 2
#define RR 1



//MPU6050 stuff
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu(0x68);


// MPU control/status vars

uint8_t imusetStatus;      
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint8_t fifoBuffer[42]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity 
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


//Ros stuff
#include <ros.h>
#include <yondarospack/Ard2bresponsemsg.h>
#include <yondarospack/Ard2bmotormsg.h>
#include <yondarospack/Ard2bmotorstopmsg.h>
#include <yondarospack/Ard2bimumsg.h>

ros::NodeHandle nh;
ros::Subscriber<yondarospack::Ard2bmotormsg> motor_msg_sub("Ard2bMotortopic", &motor_callback);  
ros::Subscriber<yondarospack::Ard2bmotorstopmsg> motor_stop_msg_sub("Ard2bMotorStoptopic", &motor_stop_callback);//create a subscriber to ROS topic for velocity commands (will execute "handle_cmd" function when receiving data)

ros::Subscriber<yondarospack::Ard2bimusetmsg> imu_set_sub("Ard2bIMUSettopic", &imu_set_callback);

yondarospack::Ard2bresponsemsg resp;
ros::Publisher response_pub("Ard2bResponetopic", &resp); 


yondarospack::Ard2bimumsg imu_data;
ros::Publisher imu_pub("Ard2bIMUtopic", &imu_data); 



void imusetup(){
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif
  
  nh.loginfo("Init IMU");
  mpu.initialize();
  if(mpu.testConnection())
    nh.loginfo("IMU cnnted");
  else
    nh.loginfo("IMU NOT cnnted");

  if (mpu.dmpInitialize() == 0) {

    mpu.setDMPEnabled(true);
    packetSize = mpu.dmpGetFIFOPacketSize();
    nh.loginfo("DMP Enabled");
    nh.loginfo("Stating 30s delay");
    delay(30000);
    nh.loginfo("Done");
  } 
  else {
      //CHECK this for err!
    nh.loginfo("DMP Fail" + devStatus);

  }
  
}

void imu_read(){
    mpu.resetFIFO();
    while (mpu.getFIFOCount() < packetSize && !(mpu.getIntStatus() & 0x02));
    if((mpu.getIntStatus() & 0x02))
    mpu.getFIFOBytes(fifoBuffer, packetSize);
        


    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    
    
    imu_data.qw = q.w;
    imu_data.qx = q.x;
    imu_data.qy = q.y;
    imu_data.qz = q.z;
    
    imu_data.yaw = ypr[0];
    imu_data.pitch = ypr[0];
    imu_data.roll = ypr[0];

    imu_data.ax = aaReal.x;
    imu_data.ay = aaReal.y;
    imu_data.az = aaReal.z;
    
    imu_pub.publish(&imu_data);    

    
}



void imu_set_callback (const yondarospack::Ard2bimusetmsg& msg) {
   imusetStatus = msg.set;
   
}

void motor_stop_callback (const yondarospack::Ard2bmotorstopmsg& msg) {
   motorstop();
   response_pub.publish(&resp);
}



void motor_callback (const yondarospack::Ard2bmotorstopmsg& msg) {
  
  switch(msg.dirn){
    case 'f': forward(msg.spd);
    break;
    case 'b': back(msg.spd);
    break;
    case 'l': left(msg.spd);
    break;
    case 'r': rigth(msg.spd);
    break;
  
    //write response default
  }
    response_pub.publish(&resp);
}
  


// DC motors
AF_DCMotor motorLR(LR);
AF_DCMotor motorLF(LF);
AF_DCMotor motorRF(RF);
AF_DCMotor motorRR(RR);
//spd : 0 - 255(pwm) 

void forward(int spd)
{
 motorLF.setSpeed(spd); //Define maximum velocity
 motorLF.run(FORWARD); //rotate the motor clockwise
 motorRF.setSpeed(spd); //Define maximum velocity
 motorRF.run(FORWARD); //rotate the motor clockwise
 
 motorLR.setSpeed(spd); //Define maximum velocity
 motorLR.run(FORWARD); //rotate the motor clockwise
 motorRR.setSpeed(spd); //Define maximum velocity
 motorRR.run(FORWARD); //rotate the motor clockwise
 resp.resp = "Fwd";
}

void back(int spd)
{
 motorLF.setSpeed(spd); //Define maximum velocity
 motorLF.run(BACKWARD); //rotate the motor clockwise
 motorRF.setSpeed(spd); //Define maximum velocity
 motorRF.run(BACKWARD); //rotate the motor clockwise
 
 motorLR.setSpeed(spd); //Define maximum velocity
 motorLR.run(BACKWARD); //rotate the motor clockwise
 motorRR.setSpeed(spd); //Define maximum velocity
 motorRR.run(BACKWARD); //rotate the motor clockwise
 resp.resp = "Back";
}

void left((int spd)
{
 motorRF.setSpeed(spd); //Define maximum velocity
 motorRF.run(FORWARD); //rotate the motor clockwise
 motorLF.setSpeed(spd); //Define maximum velocity
 motorLF.run(BACKWARD); //rotate the motor counterclockwise

 motorRR.setSpeed(spd); //Define maximum velocity
 motorRR.run(FORWARD); //rotate the motor clockwise
 motorLR.setSpeed(spd); //Define maximum velocity
 motorLR.run(BACKWARD); //rotate the motor counterclockwise
 resp.resp = "Left";
}

void right((int spd)
{
 motorRF.setSpeed(spd); //Define maximum velocity
 motorRF.run(BACKWARD); //rotate the motor clockwise
 motorLF.setSpeed(spd); //Define maximum velocity
 motorLF.run(FORWARD); //rotate the motor counterclockwise

 motorRR.setSpeed(spd); //Define maximum velocity
 motorRR.run(BACKWARD); //rotate the motor clockwise
 motorLR.setSpeed(spd); //Define maximum velocity
 motorLR.run(FORWARD); //rotate the mot or counterclockwise
 resp.resp = "Right";
}





void setup() {
  
//  Serial.begin(9600);           // set up Serial library at 9600 bps
//  Serial.println("Motor party!");

  
  

  //ros Setup
  nh.getHardware()->setBaud(9600);   //set baud for ROS serial communication
  nh.initNode();                            //init ROS node
  nh.subscribe(Ard2bMotortopic);       
  nh.subscribe(Ard2bMotorStoptopic);     //suscribe to ROS topic for velocity commands
  nh.advertise(Ard2bResponetopic);                  //prepare to publish speed in ROS topic
  nh.advertise(Ard2bIMUtopic);

  //Setup MPU6050
  imusetup();
  



  // turn on motors
  motorLR.run(RELEASE);
  motorLF.run(RELEASE);
  motorRF.run(RELEASE);
  motorRR.run(RELEASE);
}



void motorstop()
{
 motorLR.setSpeed(0);
 motorLR.run(RELEASE); //turn motor1 off
 motorRR.setSpeed(0);
 motorRR.run(RELEASE); //turn motor2 off

 motorLF.setSpeed(0);
 motorLF.run(RELEASE); //turn motor3 off
 motorRF.setSpeed(0);
 motorRF.run(RELEASE); //turn motor4 off
 resp.resp = "Stop";
}

void loop() {

  nh.spinOnce();
  if(imusetStatus)
    imu_read();
      
 }