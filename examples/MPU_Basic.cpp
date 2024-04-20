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

#include <Arduino.h>
#include "MPU6050.h" // Include your MPU6050 library
#include <Wire.h>

// Define I2C pins for ESP-32 (optional, default pins will work too)
#define I2C_SDA 21
#define I2C_SCL 22

// Create an instance of the MPU6050 class
MPU6050 mpu; // Default I2C address of MPU6050 is 0x68

void setup()
{
    // Initialize the Serial port for debugging
    Serial.begin(115200);
    delay(1000); // Give time for serial port to stabilize

    // Start I2C communication
    Wire.begin(I2C_SDA, I2C_SCL);
    mpu.begin(0x68); // Connect to the MPU6050 over I2C

    // Initialize MPU6050 sensor
    if (!mpu.begin(0x68))
    {
        Serial.println("Failed to initialize MPU6050 sensor");
        while (true)
        {
            delay(1000);
        }
    }

    // Print a message indicating successful initialization
    Serial.println("MPU6050 sensor initialized successfully");
}

void loop()
{
    // Variables to store acceleration data
    float ax, ay, az;
    // Variables to store gyroscope data
    float gx, gy, gz;
    // Variables to store roll and pitch
    float roll, pitch;
    // Variables to store quaternion angles
    float yaw;
    // Variables to for quaternionss
    float q0, q1, q2, q3;

    // Read acceleration data
    mpu.readAccel(ax, ay, az);
    Serial.print("Acceleration (m/s^2): ");
    Serial.print("X: ");
    Serial.print(ax, 2);
    Serial.print(", Y: ");
    Serial.print(ay, 2);
    Serial.print(", Z: ");
    Serial.print(az, 2);
    Serial.println();

    // Read gyroscope data
    mpu.readGyro(gx, gy, gz);
    Serial.print("Gyroscope (deg/s): ");
    Serial.print("X: ");
    Serial.print(gx, 2);
    Serial.print(", Y: ");
    Serial.print(gy, 2);
    Serial.print(", Z: ");
    Serial.print(gz, 2);
    Serial.println();

    // Calculate roll and pitch
    roll = mpu.calcRoll(ax, ay, az);
    pitch = mpu.calcPitch(ax, ay, az);
    Serial.print("Roll: ");
    Serial.print(roll, 2);
    Serial.print(", Pitch: ");
    Serial.print(pitch, 2);
    Serial.println();

    // Estimate angles from quaternions (this function might need adjustments) since yaw data can be taken from magnetometer instead of gyro and accelero
  mpu.AngelQuaternion(q0, q1, q2, q3, roll, pitch, yaw);
    Serial.print("Roll (Quaternion): ");
    Serial.print(roll, 2);
    Serial.print(", Pitch (Quaternion): ");
    Serial.print(pitch, 2);
    Serial.print(", Yaw: ");
    Serial.print(yaw, 2);
    Serial.println();

    // Add a delay between readings
    delay(1000);
}
