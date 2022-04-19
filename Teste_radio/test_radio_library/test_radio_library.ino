#define PORTS_TO_USE 1

//INCLUDES 
//helper libary for reading pulse signals from rc receiver
//download from https://github.com/timoxd7/FastRCReader
#include "FastRCReader.h"

const byte numInputChannels = 4;
const uint8_t channelPins[numInputChannels] = {53, 52, 51, 50};
FastRCReader RC;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  RC.begin();
  RC.addChannel(channelPins, numInputChannels);

}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("Channel_1:");
  Serial.print(RC.getFreq(channelPins[0]));
  Serial.print(" ");
  Serial.print("Channel_2:");
  Serial.print(RC.getFreq(channelPins[1]));
  Serial.print(" ");
  Serial.print("Channel_3:");
  Serial.print(RC.getFreq(channelPins[2]));
  Serial.print(" ");
  Serial.print("Channel_4:");
  Serial.print(RC.getFreq(channelPins[3]));
  Serial.println(" ");

}
