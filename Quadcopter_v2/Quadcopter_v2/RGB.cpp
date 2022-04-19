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

void RGB::red(){

}
  
void RGB::green(){

}

void RGB::blue(){

}
 
void RGB::yellow(){

}