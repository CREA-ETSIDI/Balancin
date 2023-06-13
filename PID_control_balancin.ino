/********************************************************
 * PID control of two motors with ESC's
 * 
 ********************************************************/

#include <Servo.h>

#define MOTOR_LEFT 6
#define MOTOR_RIGHT 5

Servo leftmotor, rightmotor;

#include "MPU_minimal.h"

MPU_minimal mpu_drone;

#include "PID_minimal.h"

//Define Variables we'll be connecting to
double setpoint, input, output;

//Specify the links and initial tuning parameters
double Kp=0.15, Ki=0.2326, Kd=0.052;
PID_minimal PID_drone(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

#include "wiichuck_minimal.h"

Wiichuck wii;
int joyY = 0, accelX = 0;
uint8_t buttonC = 0, buttonZ = 0;

void setup()
{
  Serial.begin(9600);
	leftmotor.attach(MOTOR_LEFT);
	rightmotor.attach(MOTOR_RIGHT);
  leftmotor.write(0);
  rightmotor.write(0);
  delay(3000);

  wii.init();  
  wii.calibrate();  // calibration

  mpu_drone.init();
  PID_drone.SetOutputLimits(-50, 50);
}
void loop()
{
  mpu_drone.update();

  if(wii.poll())
  {
    //Set sensitivity for Wiichuck controller
    accelX = map(wii.accelX(), 300, 700, 90, -90);
    //Set sensitivity for joystick
    joyY = map(wii.joyY(), 0, 255, 0, 60);
  }
  if(!wii.buttonC())
  {
    rightmotor.write(0);
    leftmotor.write(0);
    setpoint=0;
    PID_drone.SetMode(MANUAL);
  }
  else
  {
    PID_drone.SetMode(AUTOMATIC);
    input = mpu_drone.AngleX();
    setpoint = accelX;
    if(wii.buttonZ()) setpoint = 0;
    PID_drone.Compute();

    rightmotor.write(joyY + output);
    leftmotor.write(joyY - output);
  }
}
