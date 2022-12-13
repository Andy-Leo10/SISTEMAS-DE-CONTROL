#include <Arduino.h>

#define IN1 32
#define IN2 33
#define nSleep 25
#define encoderA 34
#define encoderB 35

void IRAM_ATTR funcionInterrupcion(){

}

void setup() {
  // put your setup code here, to run once:
  pinMode(nSleep,OUTPUT);

  ledcSetup(1,20000,10);
  ledcSetup(2,20000,10);
  ledcAttachPin(IN1,1);
  ledcAttachPin(IN2,2);

  pinMode(encoderA,INPUT_PULLUP);
  pinMode(encoderB,INPUT_PULLUP);

  attachInterrupt(encoderA,funcionInterrupcion,RISING);

  double vm=5;
  double vp=1;
  uint32_t Duty=(uint32_t)((1-abs(vp/vm))*1024);

  if(vp<0){
    ledcWrite(1,Duty);
    ledcWrite(2,1024);
  }
  else{
    ledcWrite(1,1024);
    ledcWrite(2,Duty);
  }

}

void loop() {
  // put your main code here, to run repeatedly:
}