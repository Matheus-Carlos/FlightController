#include "RGB.h"
#include <Arduino.h>

RGB::RGB(int R, int G, int B){
    pinR = R;
    pinG = G;
    pinB = B;
    pinMode(pinR, OUTPUT);
    pinMode(pinG, OUTPUT);
    pinMode(pinB, OUTPUT);
}

void setColor(int r, int g, int b){

    analogWrite(pinR, 255 - r); 
    analogWrite(pinG, 255 - g); 
    analogWrite(pinB, 255 - b); 

}

void RGB::red(){ setColor(255,0,0); }
  
void RGB::green(){ setColor(0,255,0) }

void RGB::blue(){ setColor(0,0,255); }
 
void RGB::yellow(){ setColor(255,255,0); }

void RGB::starting(){

}

void calibrating(){

}

void flying(){

}