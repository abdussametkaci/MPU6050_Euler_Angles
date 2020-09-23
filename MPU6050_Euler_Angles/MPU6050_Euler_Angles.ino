#include "MPU6050T.hpp"
MPU6050T mpu;

void setup() {
  Wire.begin();
  Serial.begin(115200);
  mpu.begin();
  mpu.initialize(ACCEL_FULL_SCALE_2G, GYRO_FULL_SCALE_250DPS);
  
}

void loop() {
  mpu.readDatas();
  Serial.print(mpu.getRoll());
  Serial.print(",");
  Serial.print(mpu.getPitch());
  Serial.print(",");
  Serial.println(mpu.getYaw());

  delay(100);

}
