#ifndef _RGB_H
#define _RGB_H

#include <Arduino.h>


class RGB{
    private:
        red();
        green();
        blue();
        yellow();
        int pinR, pinG, pinB;

    public:
        RGB(int R, int G, int B);
        void starting();
        void calibrating();
        void flying();
};

#endif