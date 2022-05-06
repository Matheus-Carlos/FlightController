#include <Servo.h>
#include <BMI160Gen.h>
#define BitTst
#include <imuFilter.h>

double rotd[3]; // [phid, thetad, psid] desired

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
int motor_pins[4] = {36, 34, 35, 37};

// sensor
constexpr float GAIN = 0.1;     // Fusion gain, value between 0 and 1 - Determines heading correction with respect to gravity vector. 
imuFilter <&GAIN> fusion;
float acc[3];
float gyro[3];
float roll, pitch, yaw;

// More variables for the code
int i;
int mot_activated = 0;
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

void printMesument() {
  Serial.print("Roll:");
  Serial.print(roll);
  Serial.print(" ");
//  Serial.print("Original2:");
//  Serial.print(original2);
//  Serial.print(" ");
  Serial.print("Pitch:");
  Serial.print(pitch);
  Serial.print(" ");
  Serial.print("Yaw:");
  Serial.print(yaw);
  Serial.print(" ");
}

void printReference() {
  Serial.print("ref_phi:");
  Serial.print(rotd[0]);
  Serial.print(" ");
  Serial.print("ref_theta:");
  Serial.print(rotd[1]);
  Serial.print(" ");
//  Serial.print("ref_psi:");
//  Serial.print(rotd[2]);
//  Serial.print(" ");
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


void setup(){

  Serial.begin(38400); // initialize Serial communication
  while (!Serial);    // wait for the serial port to open
  Serial.println("Initializing IMU device...done.");
  BMI160.begin(BMI160GenClass::I2C_MODE);
  BMI160.setGyroRange(250);
  BMI160.autoCalibrateGyroOffset(); 
  readAccelerometer(acc);
  fusion.setup(acc[0], acc[1], acc[2]);

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
    motors[i].writeMicroseconds(1000);
  }
  delay(5000);
}

void loop(){

  readAccelerometer(acc);
  readGyro(gyro);
  fusion.update(gyro[0], gyro[1], gyro[2], acc[0], acc[1], acc[2] );

 pitch = fusion.pitch();
 roll = fusion.roll();
 yaw = fusion.yaw();

  pwm[0] = 115 + inputs[THROTTLE];//L_F
  pwm[1] = 115 + inputs[THROTTLE];//R_F
  pwm[2] = 115 + inputs[THROTTLE];//L_B
  pwm[3] = 115 + inputs[THROTTLE];//R_B


  for (int i = 0; i < 4; i++) {
    pwm[i] = sat(pwm[i], 1100, 2000);
  }

  if (mot_activated==1)
  {
//    Serial.print(" *Estou aqui* ");
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
  if (inputs[ARM] > 1800 && inputs[THROTTLE] < 1100) mot_activated = 1;
  else if (inputs[ARM] < 1400 && inputs[THROTTLE] < 1100) mot_activated = 0;

  printMesument();
//  printReference();
//  print_PID_out();
//  print_PWM();
//  Serial.print("kp:");
//  Serial.print(kp);
//  Serial.print(" ");
//  print_RadioValues();
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

double sat(double val, double min, double max){
  return val < min ? min : (val > max ? max : val);
}

void readAccelerometer(float acc[]) {
    long acc_raw[3];
    BMI160.readAccelerometer(acc_raw[0], acc_raw[1], acc_raw[2]);
    for (int i = 0; i < 3; i++){
        acc[i] = (float)acc_raw[i]/16384.0;
    }
}

void readGyro(float gyro[]) {
    int gyro_raw[3];
    BMI160.readGyro(gyro_raw[0], gyro_raw[1], gyro_raw[2]);
    for (int i = 0; i < 3; i++){
        gyro[i] = (float)gyro_raw[i]*(PI/180.0)/131.2;
    }
}
