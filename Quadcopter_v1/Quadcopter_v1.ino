#include <Servo.h>
#include <BMI160Gen.h>

/*------------------*/
//Motors

Servo L_F_prop;
Servo L_B_prop;
Servo R_F_prop;
Servo R_B_prop;

//Variables for the sensor

const int select_pin = 10;
const int i2c_addr = 0x69;
float acAngle_x, acAngle_y, acAngle_z, acAngle_x_inicial, acAngle_y_inicial, off_x=0, off_y=0;
int ax, ay, az; // raw acc values
float Total_angle_x, Total_angle_y, Total_angle_z;

//Variables for the radio

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

//More variables for the code
int i;
int mot_activated=0;
long activate_count=0;
long des_activate_count=0;
float elapsedTime, time, timePrev;        //Variables for time control

// PID VARIABLES 
//////////////////////////////PID FOR ROLL///////////////////////////
float roll_PID, pwm_L_F, pwm_L_B, pwm_R_F, pwm_R_B, roll_error, roll_previous_error;
float roll_pid_p=0;
float roll_pid_i=0;
float roll_pid_d=0;
///////////////////////////////ROLL PID CONSTANTS////////////////////
double roll_kp=0.7;//3.55
double roll_ki=0.006;//0.003
double roll_kd=1.2;//2.05
float roll_desired_angle = 0;     //This is the angle in which we whant the

//////////////////////////////PID FOR PITCH//////////////////////////
float pitch_PID, pitch_error, pitch_previous_error;
float pitch_pid_p=0;
float pitch_pid_i=0;
float pitch_pid_d=0;
///////////////////////////////PITCH PID CONSTANTS///////////////////
double pitch_kp=0.72;//3.55
double pitch_ki=0.006;//0.003
double pitch_kd=1.22;//2.05
float pitch_desired_angle = 0;     //This is the angle in which we whant the
//////////////////////////////PID FOR YAW//////////////////////////
float yaw_PID, yaw_error, yaw_previous_error;
float yaw_pid_p=0;
float yaw_pid_i=0;
float yaw_pid_d=0;
///////////////////////////////YAW PID CONSTANTS///////////////////
double yaw_kp=0.72;//3.55
double yaw_ki=0.006;//0.003
double yaw_kd=1.22;//2.05
float yaw_desired_angle = 0;     //This is the angle in which we whant the

void setup() {
  Serial.begin(19200); // initialize Serial communication
  delay(1000);
  time = millis();  

  DDRB |= B00100000;  //D13 as output
  PORTB &= B11011111; //D13 set to LOW
  
  //Radio Inicialization

  PCICR |= (1 << PCIE0);    //enable PCMSK0 scan - PORT B OF THE ARDUINO MEGA                                                 
  PCMSK0 |= (1 << PCINT0);  //Set pin D53 trigger an interrupt on state change - CHANNEL 1. 
  PCMSK0 |= (1 << PCINT1);  //Set pin D52 trigger an interrupt on state change - CHANNEL 2.                                             
  PCMSK0 |= (1 << PCINT2);  //Set pin D51 trigger an interrupt on state change - CHANNEL 3.                                               
  PCMSK0 |= (1 << PCINT3);  //Set pin D50 trigger an interrupt on state change - CHANNEL 4. 
  PCMSK0 |= (1 << PCINT4);  //Set pin D10 trigger an interrupt on state change - CHANNEL 5.                                               
  PCMSK0 |= (1 << PCINT5);  //Set pin D11 trigger an interrupt on state change - CHANNEL 6. 

  // Initialize the motors' pins
  
  L_F_prop.attach(4); //left front motor
  L_B_prop.attach(5); //left back motor
  R_F_prop.attach(7); //right front motor 
  R_B_prop.attach(6); //right back motor 
  /*in order to make sure that the ESCs won't enter into config mode
  *I send a 1000us pulse to each ESC.*/
  L_F_prop.writeMicroseconds(1000); 
  L_B_prop.writeMicroseconds(1000);
  R_F_prop.writeMicroseconds(1000); 
  R_B_prop.writeMicroseconds(1000);
  delay(1000);
  L_F_prop.writeMicroseconds(1200); 
  L_B_prop.writeMicroseconds(1200);
  R_F_prop.writeMicroseconds(1200); 
  R_B_prop.writeMicroseconds(1200);
  delay(3000);
  L_F_prop.writeMicroseconds(1000); 
  L_B_prop.writeMicroseconds(1000);
  R_F_prop.writeMicroseconds(1000); 
  R_B_prop.writeMicroseconds(1000);
  delay(2000);
  
  //Sensor inicialization
  
  while (!Serial);    // wait for the serial port to open
  // initialize device
  // BMI160.begin(BMI160GenClass::SPI_MODE, select_pin);
  Serial.print("Sensor...");
  BMI160.begin(BMI160GenClass::I2C_MODE, i2c_addr);
  Serial.print("Done...");

}

void loop() {

  // Time reading for PID calculations
  timePrev = time;  // the previous time is stored before the actual time read
  time = millis();  // actual time read
  elapsedTime = (time - timePrev) / 1000;   

  // read raw gyro measurements from device
  BMI160.readAccelerometer(ax, ay, az);
  acAngle_x = map(ax, -20000, 20000, 0 ,180) - 90;
  acAngle_y = map(ay, -20000, 20000, 0 ,180) - 90;

  //////////////////////////////////////Total angle and filter/////////////////////////////////////
  /*---X axis angle(Pitch)---*/
  Total_angle_x = acAngle_x;
  /*---Y axis angle(Roll)---*/
  Total_angle_y = acAngle_y;
  /*---Z axis angle(Yaw)---*/
  Total_angle_z = acAngle_z;

  // Begining of the PID calculations  
  roll_desired_angle = map(input_ROLL,1000,2000,-10,10);
  pitch_desired_angle = map(input_PITCH,1000,2000,-10,10);
  yaw_desired_angle = map(input_YAW,1000,2000,-10,10);

  //First calculate the error between the desired angle and the real measured angle
  roll_error = Total_angle_y - roll_desired_angle;
  pitch_error = Total_angle_x - pitch_desired_angle; 
  yaw_error = Total_angle_z - yaw_desired_angle;    
  /*Next the proportional value of the PID is just a proportional constant
  *multiplied by the error*/
  roll_pid_p = roll_kp*roll_error;
  pitch_pid_p = pitch_kp*pitch_error;
  yaw_pid_p = yaw_kp*yaw_error;
  /*The integral part should only act if we are close to the
  desired position but we want to fine tune the error. That's
  why I've made a if operation for an error between -2 and 2 degree.
  To integrate we just sum the previous integral value with the
  error multiplied by  the integral constant. This will integrate (increase)
  the value each loop till we reach the 0 point*/
  if(-3 < roll_error <3)
  {
    roll_pid_i = roll_pid_i+(roll_ki*roll_error);  
  }
  if(-3 < pitch_error <3)
  {
    pitch_pid_i = pitch_pid_i+(pitch_ki*pitch_error);  
  }
  if(-3 < yaw_error <3)
  {
    yaw_pid_i = yaw_pid_i+(yaw_ki*yaw_error);  
  }
  /*The last part is the derivate. The derivate acts upon the speed of the error.
  As we know the speed is the amount of error that produced in a certain amount of
  time divided by that time. For that we will use a variable called previous_error.
  We substract that value from the actual error and divide all by the elapsed time. 
  Finnaly we multiply the result by the derivate constant*/
  roll_pid_d = roll_kd*((roll_error - roll_previous_error)/elapsedTime);
  pitch_pid_d = pitch_kd*((pitch_error - pitch_previous_error)/elapsedTime);
  yaw_pid_d = yaw_kd*((yaw_error - yaw_previous_error)/elapsedTime);
  /*The final PID values is the sum of each of this 3 parts*/
  roll_PID = roll_pid_p + roll_pid_i + roll_pid_d;
  pitch_PID = pitch_pid_p + pitch_pid_i + pitch_pid_d;
  yaw_PID = yaw_pid_p + yaw_pid_i + yaw_pid_d;

  /*We know taht the min value of PWM signal is 1000us and the max is 2000. So that
  tells us that the PID value can/s oscilate more than -1000 and 1000 because when we
  have a value of 2000us the maximum value taht we could substract is 1000 and when
  we have a value of 1000us for the PWM signal, the maximum value that we could add is 1000
  to reach the maximum 2000us. But we don't want to act over the entire range so -+400 should be enough*/
  if(roll_PID < -400){roll_PID=-400;}
  if(roll_PID > 400) {roll_PID=400; }
  if(pitch_PID < -4000){pitch_PID=-400;}
  if(pitch_PID > 400) {pitch_PID=400;}
  if(yaw_PID < -4000){yaw_PID=-400;}
  if(yaw_PID > 400) {yaw_PID=400;}

  /*Finnaly we calculate the PWM width. We sum the desired throttle and the PID value*/
  pwm_R_F  = 115 + input_THROTTLE - roll_PID - pitch_PID - yaw_PID;
  pwm_R_B  = 115 + input_THROTTLE - roll_PID + pitch_PID + yaw_PID;
  pwm_L_B  = 115 + input_THROTTLE + roll_PID + pitch_PID - yaw_PID;
  pwm_L_F  = 115 + input_THROTTLE + roll_PID - pitch_PID + yaw_PID;

  /*Once again we map the PWM values to be sure that we won't pass the min
  and max values. Yes, we've already maped the PID values. But for example, for 
  throttle value of 1300, if we sum the max PID value we would have 2300us and
  that will mess up the ESC.*/
  //Right front
  if(pwm_R_F < 1100)
  {
    pwm_R_F= 1100;
  }
  if(pwm_R_F > 2000)
  {
    pwm_R_F=2000;
  }
  
  //Left front
  if(pwm_L_F < 1100)
  {
    pwm_L_F= 1100;
  }
  if(pwm_L_F > 2000)
  {
    pwm_L_F=2000;
  }
  
  //Right back
  if(pwm_R_B < 1100)
  {
    pwm_R_B= 1100;
  }
  if(pwm_R_B > 2000)
  {
    pwm_R_B=2000;
  }
  
  //Left back
  if(pwm_L_B < 1100)
  {
    pwm_L_B= 1100;
  }
  if(pwm_L_B > 2000)
  {
    pwm_L_B=2000;
  }

  roll_previous_error = roll_error; //Remember to store the previous error.
  pitch_previous_error = pitch_error; //Remember to store the previous error.
  yaw_previous_error = yaw_error; //Remember to store the previous error.

    if(mot_activated)
  {
  L_F_prop.writeMicroseconds(pwm_L_F); 
  L_B_prop.writeMicroseconds(pwm_L_B);
  R_F_prop.writeMicroseconds(pwm_R_F); 
  R_B_prop.writeMicroseconds(pwm_R_B);
  }
  if(!mot_activated)
  {
    L_F_prop.writeMicroseconds(1000); 
    L_B_prop.writeMicroseconds(1000);
    R_F_prop.writeMicroseconds(1000); 
    R_B_prop.writeMicroseconds(1000);
  }

  // Arming 
  if(input_THROTTLE < 1100 && input_YAW > 1800 && input_FLY_MODE > 1700)
  {
    mot_activated=1;
    if(activate_count==200)
    {
         
      PORTB |= B00100000; //D13 LOW   
      
    }
    activate_count=activate_count+1;
  }
  if(input_FLY_MODE < 1200){
    mot_activated=0;
  }
//  if(!(input_THROTTLE < 1100 && input_YAW > 1800) && !mot_activated)
//  {
//    activate_count=0;    
//  }
//
//   if(input_THROTTLE < 1100 && input_YAW < 1100 && mot_activated)
//  {
//    if(des_activate_count==300)
//    {
//      mot_activated=0;       
//      PORTB &= B11011111; //D13 LOW   
//    }
//    des_activate_count=des_activate_count+1;
//  }
//  if(!(input_THROTTLE < 1100 && input_YAW < 1100) && mot_activated)
//  {
//    des_activate_count=0;
//  }
  
  Serial.print("angle_x:");
  Serial.print(acAngle_x);
  Serial.print(" ");
  Serial.print("angle_y:");
  Serial.print(acAngle_y);
  Serial.print(" ");
  Serial.print("Roll_pid:");
  Serial.print(roll_PID);
  Serial.print(" ");
  Serial.print("Pitch_pid:");
  Serial.print(pitch_PID);
  Serial.print(" ");
  Serial.print("pwm_L_F:");
  Serial.print(pwm_L_F);
  Serial.print(" ");
  Serial.print("pwm_R_F:");
  Serial.print(pwm_R_F);
  Serial.print(" ");
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
//Interrupt function for reading the radio values
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
    input_ROLL = current_count - counter_1;          //We make the time difference. Channel 1 is current_time - timer_1.
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
    input_PITCH = current_count - counter_2;                             
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
    input_THROTTLE = current_count - counter_3;                            

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
    input_YAW = current_count - counter_4;                            
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
    input_FLY_MODE = current_count - counter_5;                            
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
    input_ARM = current_count - counter_6;                            
  }
}
