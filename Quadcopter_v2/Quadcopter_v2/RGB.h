#ifndef _RGB_H
#define _RGB_H

#include <Arduino.h>


class RGB{
    private:
        void red();
        void green();
        void blue();
        void yellow();
        

    public:
        RGB(int R, int G, int B);
        void starting();
        void calibrating();
        void flying();
};

#endif
