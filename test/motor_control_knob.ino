/********************************************************
 * Basic control of two motors with ESC's (controled as servos)
 * 
 ********************************************************/

#include <Servo.h>

#define MOTOR_LEFT 5
#define MOTOR_RIGHT 6
#define PIN_POT_THROTLE A0
#define PIN_POT_SETPOINT A1

Servo leftmotor, rightmotor;
int pot_throtle = 0, pot_setpoint = 0;

void setup()
{
  Serial.begin(9600);
	leftmotor.attach(MOTOR_LEFT);
	rightmotor.attach(MOTOR_RIGHT);
  leftmotor.write(0);
  rightmotor.write(0);
  delay(3000);
}
void loop()
{
  pot_throtle = analogRead(PIN_POT_THROTLE);
  pot_setpoint = analogRead(PIN_POT_SETPOINT);
  pot_throtle = map(pot_throtle, 0, 1023, 0, 80);
  pot_setpoint = map(pot_setpoint, 0, 1023, -50, 50);

	rightmotor.write(pot_throtle + pot_setpoint);
  leftmotor.write(pot_throtle - pot_setpoint);

  Serial.print(pot_setpoint);
  Serial.print(",");
  Serial.println(pot_throtle);
}
