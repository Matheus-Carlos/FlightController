#ifndef _BMI160_6053_H
#define _BMI160_6053_H

#include <Arduino.h>
#include <BMI160Gen.h>


class BMI160_60S3{
    private:

    public:
        BMI160_60S3();
        void begin(int speed, bool serial);
        void calibrate_initial(float roll, float pitch)
        void calibrate_gyro();
        void read_BMI_160_data();
        void BMI_160_update_RPY(float& roll, float& pitch, float& yaw);
};

#endif