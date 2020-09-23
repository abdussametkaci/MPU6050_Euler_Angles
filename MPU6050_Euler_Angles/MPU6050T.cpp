#include "MPU6050T.hpp"

MPU6050T::MPU6050T() {
  roll = pitch = yaw = 0;
}

void MPU6050T::I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data) {
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();

  // Read Nbytes
  Wire.requestFrom(Address, Nbytes);

  uint8_t index = 0;
  while (Wire.available()) {
    Data[index++] = Wire.read();
  }
}

void MPU6050T::I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data) {
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}

void MPU6050T::begin() {
  Wire.beginTransmission(0x68);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission();
}

void MPU6050T::initialize(uint8_t accelScale, uint8_t gyroScale) {
  // Set accelerometers low pass filter at 5Hz
  I2CwriteByte(MPU6050_ADDRESS, 29, 0x06);
  // Set gyroscope low pass filter at 5Hz
  I2CwriteByte(MPU6050_ADDRESS, 26, 0x06);


  // Configure gyroscope range
  I2CwriteByte(MPU6050_ADDRESS, 0x1B, gyroScale); //default 250 dps
  // Configure accelerometers range
  I2CwriteByte(MPU6050_ADDRESS, 0x1C, accelScale); // default 2g

  selectedAccel = accelScale;
  selectedGyro = gyroScale;

  if (selectedAccel = ACCEL_FULL_SCALE_2G) {
    accelSensitivity = ACCEL_2G_LSB_SENSITIVITY;
  } else if (selectedAccel = ACCEL_FULL_SCALE_4G) {
    accelSensitivity = ACCEL_4G_LSB_SENSITIVITY;
  } else if (selectedAccel = ACCEL_FULL_SCALE_8G) {
    accelSensitivity = ACCEL_8G_LSB_SENSITIVITY;
  } else if (selectedAccel = ACCEL_FULL_SCALE_16G) {
    accelSensitivity = ACCEL_16G_LSB_SENSITIVITY;
  }

  if (selectedGyro = GYRO_FULL_SCALE_250DPS) {
    gyroSensitivity = GYRO_250DPS_LSB_SENSITIVITY;
  } else if (selectedGyro = GYRO_FULL_SCALE_500DPS) {
    gyroSensitivity = GYRO_500DPS_LSB_SENSITIVITY;
  } else if (selectedGyro = GYRO_FULL_SCALE_1000DPS) {
    gyroSensitivity = GYRO_1000DPS_LSB_SENSITIVITY;
  } else if (selectedGyro = GYRO_FULL_SCALE_2000DPS) {
    gyroSensitivity = GYRO_2000DPS_LSB_SENSITIVITY;
  }

  calculateOffsets();
}

void MPU6050T::calculateOffsets() {
  int i = 0;
  while (i < 200) {
    I2Cread(MPU6050_ADDRESS, BEGINING_OF_DATAS, 14, Buf);
    axOffset += (Buf[0] << 8 | Buf[1]) / accelSensitivity;
    ayOffset += (Buf[2] << 8 | Buf[3]) / accelSensitivity;
    azOffset += (Buf[4] << 8 | Buf[5]) / accelSensitivity;

    gxOffset += (Buf[8] << 8  | Buf[9]) / gyroSensitivity;
    gyOffset += (Buf[10] << 8 | Buf[11]) / gyroSensitivity;
    gzOffset += (Buf[12] << 8 | Buf[13]) / gyroSensitivity;

    i++;
  }

  axOffset /= 200.0;
  ayOffset /= 200.0;
  azOffset /= 200.0;

  gxOffset /= 200.0;
  gyOffset /= 200.0;
  gzOffset /= 200.0;

}

void MPU6050T::readDatas() {
  // Read accelerometer and gyroscope
  I2Cread(MPU6050_ADDRESS, BEGINING_OF_DATAS, 14, Buf);

  // Create 16 bits values from 8 bits data

  // Accelerometer
  ax = (Buf[0] << 8 | Buf[1]) / accelSensitivity - axOffset;
  ay = (Buf[2] << 8 | Buf[3]) / accelSensitivity - ayOffset;
  az = (Buf[4] << 8 | Buf[5]) / accelSensitivity;

  //Temperature

  int16_t temp = ((int16_t)Buf[6] << 8 | (int16_t)Buf[7]);
  tempC = ((int32_t)temp) / 340.0 + 36.53;

  // Gyroscope
  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = millis();            // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds
  gx = (Buf[8] << 8  | Buf[9]) / gyroSensitivity - gxOffset;
  gy = (Buf[10] << 8 | Buf[11]) / gyroSensitivity - gyOffset;
  gz = (Buf[12] << 8 | Buf[13]) / gyroSensitivity - gzOffset;
}

float MPU6050T::getax(){
  return ax;
}

float MPU6050T::getay(){
  return ay;
}

float MPU6050T::getaz(){
  return az;
}

float MPU6050T::getgx(){
  return gx;
}

float MPU6050T::getgy(){
  return gy;
}

float MPU6050T::getgz(){
  return gz;
}

float MPU6050T::getTemperatureC(){
  return tempC;
}

float MPU6050T::getRoll(){
  roll = 180 * atan (ay / sqrt(ax * ax + az * az)) / M_PI;
  return roll;
}

float MPU6050T::getPitch(){
  pitch = 180 * atan (ax / sqrt(ay * ay + az * az)) / M_PI;
  return pitch;
}

float MPU6050T::getYaw(){
  preYaw = gz * elapsedTime;
  
  if(preYaw > 0.1 || preYaw < -0.1){
    yaw += preYaw;
  }
  
  return yaw;
}
