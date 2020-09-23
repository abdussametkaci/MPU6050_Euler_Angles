#ifndef MPU6050T_HPP
#define MPU6050T_HPP

#include <Arduino.h>
#include <Wire.h>

#define MPU6050_ADDRESS         0x68
#define BEGINING_OF_DATAS       0x3B

typedef enum Accel_Scale{
  ACCEL_FULL_SCALE_2G = 0b00000000,
  ACCEL_FULL_SCALE_4G = 0b00001000,
  ACCEL_FULL_SCALE_8G = 0b00010000,
  ACCEL_FULL_SCALE_16G = 0b00011000
};

typedef enum Gyro_Scale{
  GYRO_FULL_SCALE_250DPS = 0b00000000,
  GYRO_FULL_SCALE_500DPS = 0b00001000,
  GYRO_FULL_SCALE_1000DPS = 0b00010000,
  GYRO_FULL_SCALE_2000DPS = 0b00011000
};

typedef enum Accel_Sensitivity{
  ACCEL_2G_LSB_SENSITIVITY = 16384,
  ACCEL_4G_LSB_SENSITIVITY = 8192,
  ACCEL_8G_LSB_SENSITIVITY = 4096,
  ACCEL_16G_LSB_SENSITIVITY = 2048
};

typedef enum Gyro_Sensitivity{
  GYRO_250DPS_LSB_SENSITIVITY = 131,
  GYRO_500DPS_LSB_SENSITIVITY = 65,
  GYRO_1000DPS_LSB_SENSITIVITY = 32,
  GYRO_2000DPS_LSB_SENSITIVITY = 16
};

class MPU6050T {
  public:
    MPU6050T();
    float getax();
    float getay();
    float getaz();
    float getgx();
    float getgy();
    float getgz();
    float getRoll();
    float getPitch();
    float getYaw();
    void readDatas();
    void begin();
    void initialize(uint8_t acccelScale, uint8_t gyroScale);
    float getTemperatureC();
  private:
    float ax, ay, az, gx, gy, gz, roll, pitch, yaw, preYaw;
    float axOffset, ayOffset, azOffset, gxOffset, gyOffset, gzOffset;
    uint8_t selectedAccel, selectedGyro;
    float accelSensitivity, gyroSensitivity;
    float tempC;
    uint8_t Buf[14];
    float previousTime, currentTime, elapsedTime;
    void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data);
    void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data);
    void calculateOffsets();
};


#endif
