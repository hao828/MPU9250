#include "MPU9250.h"

MPU9250 IMU(SPI,10);

int status;
float ax_before,ay_before,az_before,ax,ay,az;
float gx,gy,gz;
float mx,my,mz;
float temp;
float roll,pitch,yaw;
void setup() {

  Serial.begin(19200);
  while(!Serial){}

  status = IMU.begin();
  if (status < 0){
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while (1) {}
  }
  // 加速度 (before)
  IMU.readSensor();
  ax_before = IMU.getAccelX_mss();
  ay_before = IMU.getAccelY_mss();
  az_before = IMU.getAccelZ_mss() + 9.8;
}


void loop() {
  IMU.readSensor();
  // 加速度 accel
  ax = IMU.getAccelX_mss() - ax_before;
  ay = IMU.getAccelY_mss() - ay_before;
  ay = IMU.getAccelZ_mss() - az_before + 9.8;
  // 陀螺儀 gyro
  gx = IMU.getGyroX_rads();
  gy = IMU.getGyroY_rads();
  gz = IMU.getGyroZ_rads();
  // 磁場 magnetic
  mx = IMU.getMagX_uT();
  my = IMU.getMagY_uT();
  mz = IMU.getMagZ_uT();
  //  溫度
  temp = IMU.getTemperature_C();
  // roll 
  roll = atan2(-ax,(sqrt((ay*ay)+(az*az))));
  roll = roll*57.3;
  // pitch
  pitch = atan2(ay,(sqrt((ax*ax)+(az*az))));
  pitch = pitch*57.3;
  // yaw
  float Yh = (my*cos(roll))-(mz*sin(roll));
  float Xh = (mx*cos(pitch))+(my*sin(roll)*sin(pitch))+(mz*cos(roll)*sin(pitch));
  yaw = atan2(Yh,Xh);
  yaw = yaw*57.3;
  
  Serial.print("加速度: ");
  Serial.print(ax,6);
  Serial.print("\t");
  Serial.print(ay,6);
  Serial.print("\t");
  Serial.println(az,6);
  
  Serial.print("陀螺儀: ");
  Serial.print(gx,6);
  Serial.print("\t");
  Serial.print(gy,6);
  Serial.print("\t");
  Serial.println(gz,6);
  
  Serial.print("磁場: ");
  Serial.print(mx,6);
  Serial.print("\t");
  Serial.print(my,6);
  Serial.print("\t");
  Serial.println(mz,6);
  
  Serial.print("溫度: ");
  Serial.println(temp);
  
  Serial.print("前後旋轉角度: ");
  Serial.println(pitch);
  
  Serial.print("左右旋轉角度: ");
  Serial.println(roll);
  
  Serial.print("水平旋轉");
  Serial.println(yaw);
  delay(5000);
  }
