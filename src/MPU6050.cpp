// MPU6050 Library for ESP-32
// Written by: Muhammad Ameer Hamza
// Date of creation: 20-04-2024
// Library Includes Following:
//  ---------------------------------------------------------
//  |   Functionality                         | Description |
//  ---------------------------------------------------------
//  |       readAccel                         |Read        |
//  |       readGyro                          |Read        |
//  |       calcRoll                          |Read        |
//  |       calcPitch                         |Read        |
//  |       AngelQuaternion                   |Read        |
//  ---------------------------------------------------------
//   mpu.readAccel(ax, ay, az);
//   mpu.readGyro(gx, gy, gz);
//   roll = mpu.calcRoll(ax, ay, az);
//   pitch = mpu.calcPitch(ax, ay, az);
//   mpu.AngelQuaternion(q0, q1, q2, q3, roll, pitch, yaw);
//
// Description:
// This library provides a set of functions to interface with the MPU6050 IMU sensor over I2C using ESP-32. It includes functions for reading acceleration data, reading gyroscope data, calculating roll and pitch, and estimating angles from quaternions. The library is designed to be simple to use and integrates seamlessly with the Arduino framework.
//
// Usage:
// - Include the library in your project:
//   #include "MPU6050.h"
//
// - Create an instance of the MPU6050 class with the default I2C address (0x68):
//   MPU6050 mpu;
//
// - Initialize the sensor in the setup() function:
//   mpu.begin();
//
// - Use the library functions to read data and calculate angles in the loop() function:
//   float ax, ay, az;
//   float gx, gy, gz;
//   float roll, pitch, yaw;
//
// Repository:
// The source code for this library can be found on GitHub/GitLab at:
// https://github.com/ameerhamza987/ESP32_MPU6050

#include "MPU6050.h"
#include <Arduino.h>
#include <math.h>


#define  PI 3.14159
#define  RAD_TO_DEG  57.29577
#define  AccelX_H_Addr  0x3B
#define  AccelY_H_Addr  0x3D
#define  AccelZ_H_Addr  0x3F
#define  GyroX_H_Addr   0x43
#define  GyroY_H_Addr   0x45
#define  GyroZ_H_Addr   0x47

// Constructor
// MPU6050::MPU6050(uint8_t address) : i2cAddr(address) {}
MPU6050::MPU6050() {}

// Initialize the MPU6050
bool MPU6050::begin(uint8_t address)
{
    i2cAddr = address; // Set the device address
    Wire.begin();
    // Initialize the MPU6050 with configuration from the datasheet where the Slave address for MPU6050 is 0x68
    Wire.beginTransmission(i2cAddr);
    Wire.write(0x6B); // Power Management 1 register, allows the user to configure the power mode and clock source.
    Wire.write(0x00); // Wake up MPU6050
    Wire.endTransmission(true);

    // Additional configurations can be added here if necessary
    return true;
}

/*
 * Each 16-bit accelerometer measurement has a full scale defined in ACCEL_FS
 * (Register 28). For each full scale setting, the accelerometers' sensitivity
 * per LSB in ACCEL_xOUT is shown in the table below:
 *
 * <pre>
 * AFS_SEL | Full Scale Range | LSB Sensitivity
 * --------+------------------+----------------
 * 0       | +/- 2g           | 16384 LSB/mg
 * 1       | +/- 4g           | 8192 LSB/mg
 * 2       | +/- 8g           | 4096 LSB/mg
 * 3       | +/- 16g          | 2048 LSB/mg
 * </pre>
 */

// Read raw acceleration data from MPU6050 and converting it into useable form
void MPU6050::readAccel(float &accelX, float &accelY, float &accelZ)
{
    // Read acceleration data from registers
    int16_t accelXRaw, accelYRaw, accelZRaw;
    readMPU6050(AccelX_H_Addr, accelXRaw); // Register address 0x3B for accelerometer X
    readMPU6050(AccelY_H_Addr, accelYRaw); // Register address 0x3D for accelerometer Y
    readMPU6050(AccelZ_H_Addr, accelZRaw); // Register address 0x3F for accelerometer Z

    // Convert raw data to acceleration values
    accelX = accelXRaw / 16384.0; // Assuming MPU6050 set to +/-2g range
    accelY = accelYRaw / 16384.0;
    accelZ = accelZRaw / 16384.0;
}

/*
 * Each 16-bit gyroscope measurement has a full scale defined in FS_SEL
 * (Register 27). For each full scale setting, the gyroscopes' sensitivity per
 * LSB in GYRO_xOUT is shown in the table below:
 *
 * <pre>
 * FS_SEL | Full Scale Range   | LSB Sensitivity
 * -------+--------------------+----------------
 * 0      | +/- 250 degrees/s  | 131 LSB/deg/s
 * 1      | +/- 500 degrees/s  | 65.5 LSB/deg/s
 * 2      | +/- 1000 degrees/s | 32.8 LSB/deg/s
 * 3      | +/- 2000 degrees/s | 16.4 LSB/deg/s
 * </pre>
 */

// Read raw gyroscope data from MPU6050 and converting it into useable form
void MPU6050::readGyro(float &gyroX, float &gyroY, float &gyroZ)
{
    // Read acceleration data from registers
    int16_t gyroXRaw, gyroYRaw, gyroZRaw;
    readMPU6050(GyroX_H_Addr, gyroXRaw); // Register address 0x43 for gyroscope X
    readMPU6050(GyroY_H_Addr, gyroYRaw); // Register address 0x45 for gyroscope Y
    readMPU6050(GyroZ_H_Addr, gyroZRaw); // Register address 0x47 for gyroscope Z

    // Convert raw data to acceleration values
    gyroX = gyroXRaw / 131.0; // Assuming MPU6050 set to +/-250 degrees/s range
    gyroY = gyroYRaw / 131.0;
    gyroZ = gyroZRaw / 131.0;
}

// Calculate roll from accleroeter data
float MPU6050::calcRoll(float accelX, float accelY, float accelZ)
{
    // different theires denotes different formulas for calculating roll from accelerometer data
    // formula to calculate the  Roll angle using Accelerometer Data i.e. arctan(ay/az) * 180/pi
    return atan2(accelY, accelZ) * RAD_TO_DEG;
}

// Calculate pitch from accleroeter data
float MPU6050::calcPitch(float accelX, float accelY, float accelZ)
{
    // different theires denotes different formulas for calculating pitch from accelerometer data
    // formula to calculate pitch angle using accelerometer data i.e. arctan(-ax/(ay^2 + az^2)) * 180/pi
    return atan2(-accelX, sqrt((accelY * accelY) + (accelZ * accelZ))) * RAD_TO_DEG;
}

// Estimate angle from quaternions
void MPU6050::AngelQuaternion(float q0, float q1, float q2, float q3, float &roll, float &pitch, float &yaw)
{
    // For quaternion-based angle estimation, you may use external libraries or reference the MPU6050 datasheet.
    // Also the quaternion angel can only be estimated (not the exact values), since quaternion needs data of yaw
    // yaw data is calaculted from magnetometer data
    // mpu6050 is only for gyro and accelero data
    // Assuming quaternion values are read from the MPU6050 in q0, q1, q2, q3
    // int q0, q1, q2, q3;

    // Convert quaternions to roll, pitch, yaw angles
    roll = atan2(2.0 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2));
    pitch = asin(2.0 * (q0 * q2 - q3 * q1));
    yaw = atan2(2.0 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3));

    // Convert angles from radians to degrees
    roll *= RAD_TO_DEG;
    pitch *= RAD_TO_DEG;
    yaw *= RAD_TO_DEG;
}

// Private function to read data from MPU6050
void MPU6050::readMPU6050(uint8_t regAddr, int16_t &data)
{
    // Begin a transmission to the I2C device with the specified address
    Wire.beginTransmission(i2cAddr);
    // Write the register address to request data from
    Wire.write(regAddr);
    // End the transmission without releasing the bus
    Wire.endTransmission(false);
    // Request data from the I2C device starting from the register address, and request 2 bytes
    Wire.requestFrom(i2cAddr, 2);
    // Check if 2 bytes are available to read
    if (Wire.available() == 2)
    {
        // Combine the two received bytes into a 16-bit integer
        uint8_t highByte = (Wire.read() << 8);
        uint8_t lowByte = Wire.read();
        data = highByte | lowByte;
        // data = (Wire.read() << 8) | Wire.read();
    }
}
