#include "MPU9250.h"

MPU9250 IMU(SPI,10);

int status;
float ax_before,ay_before,az_before,ax,ay,az;
float gx_before,gx,gy,gz;
float mx_before,my_before,mz_before,mx,my,mz;
float temp;
float roll_before,pitch_before,yaw_before,roll,pitch,yaw;
void setup() {

  Serial.begin(115200);
  while(!Serial){}

  status = IMU.begin();
 /* if (status < 0){
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while (1) {}
  }*/
  // 加速度 (before)
  IMU.readSensor();
  ax_before = IMU.getAccelX_mss();
  ay_before = IMU.getAccelY_mss();
  az_before = IMU.getAccelZ_mss() + 9.8;
  // 磁場 magnetic (before)
  mx_before = IMU.getMagX_uT();
  my_before = IMU.getMagY_uT();
  mz_before = IMU.getMagZ_uT();
  // 旋轉角度(before)
  roll_before = atan2(-ax_before,(sqrt((ay_before*ay_before)+(az_before*az_before))));
  roll_before = roll_before*57.3;
  pitch_before = atan2(ay_before,(sqrt((ax_before*ax_before)+(az_before*az_before))));
  pitch_before = pitch_before*57.3;
  float Yh_before = (my_before*cos(roll_before))-(mz_before*sin(roll_before));
  float Xh_before = (mx_before*cos(pitch_before))+(my_before*sin(roll_before)*sin(pitch_before))+(mz_before*cos(roll_before)*sin(pitch_before));
  yaw_before = atan2(Yh_before,Xh_before);
  yaw_before = yaw_before*57.3;
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
  roll = roll - roll_before;
  // pitch
  pitch = atan2(ay,(sqrt((ax*ax)+(az*az))));
  pitch = pitch*57.3 - pitch_before;
  // yaw
  float Yh = (my*cos(roll))-(mz*sin(roll));
  float Xh = (mx*cos(pitch))+(my*sin(roll)*sin(pitch))+(mz*cos(roll)*sin(pitch));
  yaw = atan2(Yh,Xh);
  yaw = yaw*57.3 - yaw_before;
  
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
