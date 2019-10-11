int16_t ax, ay, az,gx, gy, gz;
int mean_ax,mean_ay,mean_az,mean_gx,mean_gy,mean_gz;
int ax_offset,ay_offset,az_offset,gx_offset,gy_offset,gz_offset;

void meansensors(){
  
  int buffersize=1000; 
  long i=0,buff_ax=0,buff_ay=0,buff_az=0,buff_gx=0,buff_gy=0,buff_gz=0;


  for(;i<100;i++){
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    delay(3);
  }
  for(i=0;i<buffersize;i++){
      mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      buff_ax=buff_ax+ax;
      buff_ay=buff_ay+ay;
      buff_az=buff_az+az;
      buff_gx=buff_gx+gx;
      buff_gy=buff_gy+gy;
      buff_gz=buff_gz+gz;
      delay(3);    
  }
      mean_ax=buff_ax/buffersize;
      mean_ay=buff_ay/buffersize;
      mean_az=buff_az/buffersize;
      mean_gx=buff_gx/buffersize;
      mean_gy=buff_gy/buffersize;
      mean_gz=buff_gz/buffersize;
}

void calibration(){

  int acel_deadzone=8;     //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
  int giro_deadzone=1;
  
  ax_offset=-mean_ax/8;
  ay_offset=-mean_ay/8;
  az_offset=(16384-mean_az)/8;
  Serial.println("xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx");
  Serial.print(ax_offset);
  Serial.print("\t");
  Serial.print(ax_offset);
  Serial.print("\t");
  Serial.print(ax_offset);

  gx_offset=-mean_gx/4;
  gy_offset=-mean_gy/4;
  gz_offset=-mean_gz/4;
  
//  while (1){
//    int ready=0;
    mpu.setXAccelOffset(ax_offset);
    mpu.setYAccelOffset(ay_offset);
    mpu.setZAccelOffset(az_offset);

    mpu.setXGyroOffset(gx_offset);
    mpu.setYGyroOffset(gy_offset);
    mpu.setZGyroOffset(gz_offset);

//    meansensors();
//    Serial.println("...");
//
//    if (abs(mean_ax)<=acel_deadzone) ready++;
//    else ax_offset=ax_offset-mean_ax/acel_deadzone;
//
//    if (abs(mean_ay)<=acel_deadzone) ready++;
//    else ay_offset=ay_offset-mean_ay/acel_deadzone;
//
//    if (abs(16384-mean_az)<=acel_deadzone) ready++;
//    else az_offset=az_offset+(16384-mean_az)/acel_deadzone;
//
//    if (abs(mean_gx)<=giro_deadzone) ready++;
//    else gx_offset=gx_offset-mean_gx/(giro_deadzone+1);
//
//    if (abs(mean_gy)<=giro_deadzone) ready++;
//    else gy_offset=gy_offset-mean_gy/(giro_deadzone+1);
//
//    if (abs(mean_gz)<=giro_deadzone) ready++;
//    else gz_offset=gz_offset-mean_gz/(giro_deadzone+1);
//
//    if (ready==6) break;
//  }
}

void caliber_imu(){
   
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);

  meansensors();
  calibration();
  
  
}