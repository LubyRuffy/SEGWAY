#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "math.h"

#define IN1               6
#define IN2               5
#define IN3               10
#define IN4               9


#define Kp  3.47
#define Kd  8.95
#define Ki  94.85
#define tempoAmostr  5
#define anguloDesejado 2.2
MPU6050 mpu;

int16_t accX, accZ, gyroY;
volatile int PotenciaMotor, TaxaGiro;
volatile float anguloAcc, anguloGiro, anguloAtual, angulonAnterior=0, erro, erroAnterior=0, somErro=0;
volatile byte cont=0;

void defineMotor(int velMotorEsq, int velMotorDir) {
  if(velMotorEsq >= 0) {
    analogWrite(1,velMotorEsq);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }
  else {
    analogWrite(1,255 + velMotorEsq);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
  if(velMotorDir >= 0) {
    analogWrite(1,velMotorDir);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
  else {
    analogWrite(1,255 + velMotorDir);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
}

void incia_PID() {  
  cli();          
  TCCR1A = 0;    
  TCCR1B = 0;      
  OCR1A = 9999;    
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS11);
  TIMSK1 |= (1 << OCIE1A);
  sei();
}

void setup() {
  pinMode(1, OUTPUT);
  pinMode(1, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(13, OUTPUT);
  mpu.initialize();
  mpu.setYAccelOffset(107);
  mpu.setZAccelOffset(26230);
  mpu.setXGyroOffset(458);
  incia_PID();
}

void loop() {
  accX = mpu.getAccelerationX();
  accZ = mpu.getAccelerationZ();  
  gyroY = mpu.getRotationY();
  PotenciaMotor = constrain(PotenciaMotor, -255, 255);
  defineMotor(PotenciaMotor, PotenciaMotor);
}

ISR(TIMER1_COMPA_vect)
{
  anguloAcc = atan2(accX, accZ)*RAD_TO_DEG;
  TaxaGiro = map(gyroY, -32768, 32767, -250, 250);
  anguloGiro = (float)TaxaGiro*tempoAmostr;  
  anguloAtual = 0.9934*(angulonAnterior + anguloGiro) + 0.0066*(anguloAcc);
  erro = anguloAtual - anguloDesejado;
  somErro = somErro + erro;  
  somErro = constrain(somErro, -300, 300);
  PotenciaMotor = Kp*(erro) + Ki*(somErro)*tempoAmostr - Kd*(anguloAtual-angulonAnterior)/tempoAmostr;
  angulonAnterior = anguloAtual;
  cont++;
  if(cont == 200)  {
    cont = 0;
    digitalWrite(13, !digitalRead(13));
  }
}
