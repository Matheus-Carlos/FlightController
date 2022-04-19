//We create variables for the time width values of each PWM input signal
unsigned long counter_1, counter_2, counter_3, counter_4, counter_5, counter_6, current_count;
//We create 4 variables to stopre the previous value of the input signal (if LOW or HIGH)
byte last_CH1_state, last_CH2_state, last_CH3_state, last_CH4_state, last_CH5_state, last_CH6_state;

//To store the 1000us to 2000us value we create variables and store each channel
int input_ROLL;     //In my case channel 1 of the receiver and pin D53 of arduino
int input_PITCH;    //In my case channel 2 of the receiver and pin D52 of arduino
int input_THROTTLE; //In my case channel 3 of the receiver and pin D51 of arduino
int input_YAW;      //In my case channel 4 of the receiver and pin D50 of arduino
int input_FLY_MODE; //In my case channel 5 of the receiver and pin D10 of arduino
int input_ARM;      //In my case channel 6 of the receiver and pin D11 of arduino

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  PCICR |= (1 << PCIE0);    //enable PCMSK0 scan - PORT B OF THE ARDUINO MEGA                                                 
  PCMSK0 |= (1 << PCINT0);  //Set pin D53 trigger an interrupt on state change - CHANNEL 1. 
  PCMSK0 |= (1 << PCINT1);  //Set pin D52 trigger an interrupt on state change - CHANNEL 2.                                             
  PCMSK0 |= (1 << PCINT2);  //Set pin D51 trigger an interrupt on state change - CHANNEL 3.                                               
  PCMSK0 |= (1 << PCINT3);  //Set pin D50 trigger an interrupt on state change - CHANNEL 4. 
  PCMSK0 |= (1 << PCINT4);  //Set pin D10 trigger an interrupt on state change - CHANNEL 5.                                               
  PCMSK0 |= (1 << PCINT5);  //Set pin D11 trigger an interrupt on state change - CHANNEL 6. 
}

void loop() {
  // put your main code here, to run repeatedly:
  get_RadioValues();
}

void get_RadioValues(){
  Serial.print("Channel_1:");
  Serial.print(input_ROLL);
  Serial.print(" ");
  Serial.print("Channel_2:");
  Serial.print(input_PITCH);
  Serial.print(" ");
  Serial.print("Channel_3:");
  Serial.print(input_THROTTLE);
  Serial.print(" ");
  Serial.print("Channel_4:");
  Serial.print(input_YAW);
  Serial.print(" ");
  Serial.print("Channel_5:");
  Serial.print(input_FLY_MODE);
  Serial.print(" ");
  Serial.print("Channel_6:");
  Serial.print(input_ARM);
  Serial.println(" ");
}
ISR(PCINT0_vect){
//First we take the current count value in micro seconds using the micros() function 
  current_count = micros();
  ///////////////////////////////////////Channel 1
  if(PINB & B00000001){                              //We make an AND with the pin state register, We verify if pin 8 is HIGH???
    if(last_CH1_state == 0){                         //If the last state was 0, then we have a state change...
      last_CH1_state = 1;                            //Store the current state into the last state for the next loop
      counter_1 = current_count;                     //Set counter_1 to current value.
    }
  }
  else if(last_CH1_state == 1){                      //If pin 8 is LOW and the last state was HIGH then we have a state change      
    last_CH1_state = 0;                              //Store the current state into the last state for the next loop
    //input_ROLL = current_count - counter_1;          //We make the time difference. Channel 1 is current_time - timer_1.
    input_ARM = current_count - counter_1;
  }
  ///////////////////////////////////////Channel 2
  if(PINB & B00000010 ){                             //pin D9 -- B00000010                                              
    if(last_CH2_state == 0){                                               
      last_CH2_state = 1;                                                   
      counter_2 = current_count;                                             
    }
  }
  else if(last_CH2_state == 1){                                           
    last_CH2_state = 0;                                                     
    //input_PITCH = current_count - counter_2; 
    input_FLY_MODE = current_count - counter_2;                            
  }
  ///////////////////////////////////////Channel 3
  if(PINB & B00000100 ){                             //pin D10 - B00000100                                         
    if(last_CH3_state == 0){                                             
      last_CH3_state = 1;                                                  
      counter_3 = current_count;                                               
    }
  }
  else if(last_CH3_state == 1){                                             
    last_CH3_state = 0;                                                    
    //input_THROTTLE = current_count - counter_3;
    input_YAW = current_count - counter_3;                            

  }
  ///////////////////////////////////////Channel 4
  if(PINB & B00001000 ){                             //pin D12  -- B00010000                      
    if(last_CH4_state == 0){                                               
      last_CH4_state = 1;                                                   
      counter_4 = current_count;                                              
    }
  }
  else if(last_CH4_state == 1){                                             
    last_CH4_state = 0;                                                  
    //|input_YAW = current_count - counter_4;
    input_THROTTLE = current_count - counter_4;                            
  } 
  ///////////////////////////////////////Channel 5
  if(PINB & B00010000 ){                             //pin D12  -- B00010000                      
    if(last_CH5_state == 0){                                               
      last_CH5_state = 1;                                                   
      counter_5 = current_count;                                              
    }
  }
  else if(last_CH5_state == 1){                                             
    last_CH5_state = 0;                                                  
    //input_FLY_MODE = current_count - counter_5; 
    input_PITCH = current_count - counter_5;                           
  } 
  ///////////////////////////////////////Channel 6
  if(PINB & B00100000 ){                             //pin D12  -- B00010000                      
    if(last_CH6_state == 0){                                               
      last_CH6_state = 1;                                                   
      counter_6 = current_count;                                              
    }
  }
  else if(last_CH6_state == 1){                                             
    last_CH6_state = 0;                                                  
    //input_ARM = current_count - counter_6;
    input_ROLL = current_count - counter_6;                            
  }
}
