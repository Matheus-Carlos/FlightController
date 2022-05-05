#include <BMI160Gen.h>
#include <imuFilter.h>
// Sensor fusion
constexpr float GAIN = 0.1;     // Fusion gain, value between 0 and 1 - Determines heading correction with respect to gravity vector. 
imuFilter <&GAIN> fusion;
float acc[3];
float gyro[3];

void setup() {
  Serial.begin(38400); // initialize Serial communication
  while (!Serial);    // wait for the serial port to open
  Serial.println("Initializing IMU device...done.");
  BMI160.begin(BMI160GenClass::I2C_MODE);
  BMI160.setGyroRange(250);
  BMI160.autoCalibrateGyroOffset(); 
  readAccelerometer(acc);
  fusion.setup(acc[0], acc[1], acc[2]);

//  BMI160.setAccelDLPFMode(2);
//  BMI160.setGyroDLPFMode(2);
//  BMI160.setAccelRate(10);
//  BMI160.setGyroRate(10);
//  Serial.print("Gyro Range: ");
//  Serial.println(BMI160.getFullScaleGyroRange());
//  Serial.print("Acc Range: ");
//  Serial.println(BMI160.getFullScaleAccelRange());
//  Serial.print("Acc DLPFMode: ");
//  Serial.println(BMI160.getAccelDLPFMode());
//  Serial.print("Gyro DLPFMode: ");
//  Serial.println(BMI160.getGyroDLPFMode());
//  Serial.print("Acc rate: ");
//  Serial.println(BMI160.getAccelRate());
//  Serial.print("Gyro rate: ");
//  Serial.println(BMI160.getGyroRate());
}

void loop() {
  // read raw gyro measurements from device
  readAccelerometer(acc);
  readGyro(gyro);
  fusion.update(gyro[0], gyro[1], gyro[2], acc[0], acc[1], acc[2] );
  Serial.print("pitch:");
  Serial.print(fusion.pitch());
  Serial.print(" roll:");
  Serial.print(fusion.roll());
  Serial.print(" yaw:");
  Serial.print(fusion.yaw());
  Serial.println(" ");
}


/*
   Copyright (c) 2016 Intel Corporation.  All rights reserved.
   This library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2.1 of the License, or (at your option) any later version.
   This library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Lesser General Public License for more details.
   You should have received a copy of the GNU Lesser General Public
   License along with this library; if not, write to the Free Software
   Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

void readAccelerometer(float acc[]) {
    long acc_raw[3];
    BMI160.readAccelerometer(acc_raw[0], acc_raw[1], acc_raw[2]);
    for (int i = 0; i < 3; i++){
        acc[i] = (float)acc_raw[i]/16384.0;
    }
}

void readGyro(float gyro[]) {
    int gyro_raw[3];
    BMI160.readGyro(gyro_raw[0], gyro_raw[1], gyro_raw[2]);
    for (int i = 0; i < 3; i++){
        gyro[i] = (float)gyro_raw[i]*(PI/180.0)/131.2;
    }
}
