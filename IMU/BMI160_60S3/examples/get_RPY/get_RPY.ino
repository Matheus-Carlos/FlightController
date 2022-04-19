#include <BMI160_60S3.h>

float roll, pitch, yaw;

BMI160_60S3 sensor;
void setup() {
  sensor.begin(9600, true); // Set the serial speed. 
                            // The boolean is set to true only if you want to print the status of the IMU on the serial monitor. 
                           
}

void loop() {
  sensor.BMI_160_update_RPY(roll, pitch, yaw);
  Serial.print("Roll:");
  Serial.print(roll);
  Serial.print(" ");
  Serial.print("Pitch:");
  Serial.print(pitch);
  Serial.print(" ");
  Serial.print("Yaw:");
  Serial.print(yaw);
  Serial.println(" ");
}
