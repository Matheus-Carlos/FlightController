#include <Servo.h>
#include <BMI160Gen.h>
#include <BMI160_60S3.h>
#include "utils.h"
#include "PIDController.h"
#include <EEPROM.h>

#define BitTst

void EEPROM_writeDouble(int ee, float value)
{
    byte* p = (byte*)(void*)&value;
    for (int i = 0; i < sizeof(value); i++)
        EEPROM.write(ee++, *p++);
}

float EEPROM_readDouble(int ee)
{
    float value = 0.0;
    byte* p = (byte*)(void*)&value;
    for (int i = 0; i < sizeof(value); i++)
        *p++ = EEPROM.read(ee++);
    return value;
}

enum Input
{
  ROLL, PITCH, THROTTLE, YAW, FLY_MODE, ARM
};

enum U_ {
  U_F, U_PHI, U_THETA, U_PSI
};

/*------------------*/
// Motors

Servo motors[4];
float pwm[4];
int motor_pins[4] = {36, 35, 34, 37};

// sensor

BMI160_60S3 imu;
float roll, pitch, yaw;
float Total_angle_x, Total_angle_y, Total_angle_z;

// More variables for the code
int i;
int mot_activated = 1;
long activate_count = 0;
float elapsedTime, time, timePrev; // Variables for time control

// Variables for the radio
unsigned current_count;
typedef struct
{
  char last_state;
  unsigned counter;
} channel_data;

channel_data channels[6];

// To store the 1000us to 2000us value we create variables and store each channel
int inputs[6] = {0, 0, 0, 0, 0, 0};

PIDController controllers[3] = {
  PIDController(10, 0.01, 0.3),
  PIDController(10, 0.01, 0.3),
  PIDController(4.5, 0.1, 0.2)
};

double U[4] = { 0, 0, 0, 0 };
double rot[3];  // [phi, theta, psi] mesuarement
double rotd[3]; // [phid, thetad, psid] desired


void printMesument() {
  Serial.print("Roll:");
  Serial.print(rot[0]);
  Serial.print(" ");
  Serial.print("Pitch:");
  Serial.print(rot[1]);
  Serial.print(" ");
  Serial.print("Yaw:");
  Serial.print(rot[2]);
  Serial.print(" ");
}



void printReference() {
  Serial.print("ref_phi:");
  Serial.print(rotd[0]);
  Serial.print(" ");
  Serial.print("ref_theta:");
  Serial.print(rotd[1]);
  Serial.print(" ");
  Serial.print("ref_psi:");
  Serial.print(rotd[2]);
  Serial.print(" ");
}

void print_PID_out() {
  Serial.print("Roll_pid:");
  Serial.print(U[2]);
  Serial.print(" ");
  Serial.print("Pitch_pid:");
  Serial.print(U[1]);
}

void print_PWM() {
  Serial.print("pwm_L_F:");
  Serial.print(pwm[0]);
  Serial.print(" ");
  Serial.print("pwm_R_F:");
  Serial.print(pwm[1]);
  Serial.print(" ");
  Serial.print("pwm_L_B:");
  Serial.print(pwm[2]);
  Serial.print(" ");
  Serial.print("pwm_R_B:");
  Serial.print(pwm[3]);
}

void print_RadioValues(){
  for(int i=0;i<6;i++){
    Serial.print("Channel_");
    Serial.print(i+1);
    Serial.print(":");
    Serial.print(inputs[i]);
    Serial.print(" ");
  }
}


float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}


void setup()
{

  imu.begin(9600, true);

  Serial.println("*************");
  Serial.print("Valor kp: ");
  Serial.print(EEPROM_readDouble(0));
  Serial.print(" ");
  Serial.print("Valor kd: ");
  Serial.println(EEPROM_readDouble(50));
  Serial.println("*************");

  PCICR |= (1 << PCIE0);   // enable PCMSK0 scan - PORT B OF THE ARDUINO MEGA
  PCMSK0 |= (1 << PCINT0); // Set pin D53 trigger an interrupt on state change - CHANNEL 1.
  PCMSK0 |= (1 << PCINT1); // Set pin D52 trigger an interrupt on state change - CHANNEL 2.
  PCMSK0 |= (1 << PCINT2); // Set pin D51 trigger an interrupt on state change - CHANNEL 3.
  PCMSK0 |= (1 << PCINT3); // Set pin D50 trigger an interrupt on state change - CHANNEL 4.
  PCMSK0 |= (1 << PCINT4); // Set pin D10 trigger an interrupt on state change - CHANNEL 5.
  PCMSK0 |= (1 << PCINT5); // Set pin D11 trigger an interrupt on state change - CHANNEL 6.

  // Initialize the motors' pins
  for(int i =0; i<4 ; i++){
    motors[i].attach(motor_pins[i]);
  }
  /*in order to make sure that the ESCs won't enter into config mode
   *I send a 1000us pulse to each ESC.*/
  for(int i =0; i<4 ; i++){
    motors[i].writeMicroseconds(1400);
  }
  delay(2000);
//  for(int i =0; i<4 ; i++){
//    motors[i].writeMicroseconds(1000);
//  }
//  delay(5000);
}

void loop()
{
    static int current_tuning_controller = 2;
    static int desarming_counter = 0;
    if(inputs[THROTTLE] < 1100){ desarming_counter++; }
    if(desarming_counter == 50) {
      mot_activated = 0;
      EEPROM_writeDouble(0, controllers[current_tuning_controller].Kp);
      EEPROM_writeDouble(50, controllers[current_tuning_controller].Kd);
    }

  // Time reading for PID calculations
  timePrev = time; // the previous time is stored before the actual time read
  time = millis(); // actual time read
  elapsedTime = (time - timePrev) / 1000;

  //imu.BMI_160_update_RPY(roll, pitch, yaw);
  long accx, accy, accz;
  imu.getRaw_acc(accx, accy, accz);
  roll = mapfloat((float)accx, -20000, 20000, -90, 90);
  pitch = mapfloat((float)accy, -20000, 20000, -90, 90);
  yaw = mapfloat((float)accz, -20000, 20000, -90, 90);

  controllers[0].Kp = mapfloat((float)inputs[FLY_MODE], 1000, 2000, 0, 15);
  controllers[1].Kp = mapfloat((float)inputs[ARM], 1000, 2000, 0, 10);

  //////////////////////////////////////Total angle/////////////////////////////////////
  /*---X axis angle(Pitch)---*/
  rot[0] = pitch;
  /*---Y axis angle(Roll)---*/
  rot[1] = roll;
  /*---Z axis angle(Yaw)---*/
  rot[2] = yaw;

  // Begining of the PID calculations
  rotd[0] = 0 - mapfloat((float)inputs[PITCH], 1000, 2000, -10, 10);
  rotd[1] = 0 - mapfloat((float)inputs[ROLL], 1000, 2000, -10, 10);
  rotd[2] = 0 - mapfloat((float)inputs[YAW], 1000, 2000, -45, 45);

  // First calculate the error between the desired angle and the real measured angle
  for (int i = 0; i < 3; i++) {
    U[i + 1] = controllers[i].calc(rot[i], rotd[i], elapsedTime);
    U[i + 1] = sat(U[i + 1], -400, 400);
  }

  /*Finnaly we calculate the PWM width. We sum the desired throttle and the PID value*/
  pwm[0] = 115 + inputs[THROTTLE] - U[U_THETA] + U[U_PHI] + U[U_PSI];
  pwm[1] = 115 + inputs[THROTTLE] - U[U_THETA] - U[U_PHI] - U[U_PSI];
  pwm[2] = 115 + inputs[THROTTLE] + U[U_THETA] + U[U_PHI] - U[U_PSI];
  pwm[3] = 115 + inputs[THROTTLE] + U[U_THETA] - U[U_PHI] + U[U_PSI];
//  pwm[1] = 115 + inputs[THROTTLE]  - U[U_PSI];
//  pwm[0] = 115 + inputs[THROTTLE]  + U[U_PSI];
//  pwm[2] = 115 + inputs[THROTTLE]  - U[U_PSI];
//  pwm[3] = 115 + inputs[THROTTLE]  + U[U_PSI];

  /*Once again we map the PWM values to be sure that we won't pass the min
  and max values. Yes, we've already maped the PID values. But for example, for
  throttle value of 1300, if we sum the max PID value we would have 2300us and
  that will mess up the ESC.*/
  // Right front;
  for (int i = 0; i < 4; i++) {
    pwm[i] = sat(pwm[i], 1100, 2000);
  }

  if (mot_activated==1)
  {
    for (int i = 0; i < 4; i++) {
      motors[i].writeMicroseconds(pwm[i]);
    }
  }
  if (mot_activated==0)
  {
    for (int i = 0; i < 4; i++) {
      motors[i].writeMicroseconds(1000);
    }
  }

  // Arming
  //if (inputs[ARM] > 1800 && inputs[THROTTLE] < 1100) mot_activated = 1;
  //else if (inputs[ARM] < 1400 && inputs[THROTTLE] < 1100) mot_activated = 0;

  printMesument();
  //printReference();
  //print_PID_out();
  //print_PWM();
  //print_RadioValues();
//  Serial.print("Motor_activate:");
//  Serial.print(mot_activated);
  Serial.println();
}


ISR(PCINT0_vect)
{
  // First we take the current count value in micro seconds using the micros() function
  current_count = micros();
  ///////////////////////////////////////Channels 1 to 6
  for (int i = 0; i < 6; i++)
  {
    if (PINB & (1 << i))
    { // We make an AND with the pin state register, We verify if pin 8 is HIGH???
      if (channels[i].last_state == 0)
      {                                      // If the last state was 0, then we have a state change...
        channels[i].last_state = 1;          // Store the current state into the last state for the next loop
        channels[i].counter = current_count; // Set counter_1 to current value.
      }
    }
    else if (channels[i].last_state == 1)
    {                             // If pin 8 is LOW and the last state was HIGH then we have a state change
      channels[i].last_state = 0; // Store the current state into the last state for the next loop
      inputs[5 - i] = current_count - channels[i].counter;
    }
  }
}
