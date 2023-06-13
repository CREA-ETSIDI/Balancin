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

#include <Wire.h>
 
//Direccion I2C de la IMU
#define MPU 0x68
 
//Ratios de conversion
#define A_R 16384.0
#define G_R 131.0
 
//Conversion de radianes a grados 180/PI
#define RAD_A_DEG = 57.295779
 
//MPU-6050 da los valores en enteros de 16 bits
//Valores sin refinar
int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;
uint32_t timer;
 
//Angulos
float Acc[2];
float Gy[2];
float Angle[2];

void setup()
{
  Serial.begin(9600);
	leftmotor.attach(MOTOR_LEFT);
	rightmotor.attach(MOTOR_RIGHT);
  leftmotor.write(0);
  rightmotor.write(0);
  delay(3000);
  
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  timer = micros(); //Para calcular dt
}
void loop()
{
  pot_throtle = analogRead(PIN_POT_THROTLE);
  pot_setpoint = analogRead(PIN_POT_SETPOINT);
  pot_throtle = map(pot_throtle, 0, 1023, 0, 80);
  pot_setpoint = map(pot_setpoint, 0, 1023, -50, 50);

	rightmotor.write(pot_throtle + pot_setpoint);
  leftmotor.write(pot_throtle - pot_setpoint);

  Serial.print(pot_setpoint); Serial.print(",");
  Serial.print(pot_throtle); Serial.print(",");
  
  //Leer los valores del Acelerometro de la IMU
   Wire.beginTransmission(MPU);
   Wire.write(0x3B); //Pedir el registro 0x3B - corresponde al AcX
   Wire.endTransmission(false);
   Wire.requestFrom(MPU,6,true); //A partir del 0x3B, se piden 6 registros
   AcX=Wire.read()<<8|Wire.read(); //Cada valor ocupa 2 registros
   AcY=Wire.read()<<8|Wire.read();
   AcZ=Wire.read()<<8|Wire.read();
 
    //Se calculan los angulos Y, X respectivamente.
   Acc[1] = atan(-1*(AcX/A_R)/sqrt(pow((AcY/A_R),2) + pow((AcZ/A_R),2)))*RAD_TO_DEG;
   Acc[0] = atan((AcY/A_R)/sqrt(pow((AcX/A_R),2) + pow((AcZ/A_R),2)))*RAD_TO_DEG;
 
   //Leer los valores del Giroscopio
   Wire.beginTransmission(MPU);
   Wire.write(0x43);
   Wire.endTransmission(false);
   Wire.requestFrom(MPU,4,true); //A diferencia del Acelerometro, solo se piden 4 registros
   GyX=Wire.read()<<8|Wire.read();
   GyY=Wire.read()<<8|Wire.read();
 
   //Calculo del angulo del Giroscopio
   Gy[0] = GyX/G_R;
   Gy[1] = GyY/G_R;

    // Calculo delta time
    double dt = (double)(micros()-timer) / 1000000;
    timer = micros();
 
   //Aplicar el Filtro Complementario
   Angle[0] = 0.8 *(Angle[0]+Gy[0]*dt) + 0.2*Acc[0];
   Angle[1] = 0.8 *(Angle[1]+Gy[1]*dt) + 0.2*Acc[1];
 
   //Mostrar los valores por consola
   Serial.print(" X: "); Serial.print(Angle[0]);
   Serial.print(" Y: "); Serial.println(Angle[1]);
}
