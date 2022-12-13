#include <Arduino.h>

// general para motores
const int resolution = 10;
const int freq = 300;
int dutyCycle = 0;

// motor1
#define m1en 2
#define m1in1 18
#define m1in2 19
const int pwmChannel1 = 1;

// motor2
#define m2en 4
#define m2in1 15
#define m2in2 26
const int pwmChannel2 = 2;

void setup()
{
  pinMode(m1en, OUTPUT);
  pinMode(m1in1, OUTPUT);
  pinMode(m1in2, OUTPUT);
  // configura pwm
  ledcSetup(pwmChannel1, freq, resolution);
  ledcAttachPin(m1en, pwmChannel1);

  pinMode(m2en, OUTPUT);
  pinMode(m2in1, OUTPUT);
  pinMode(m2in2, OUTPUT);
  // configura pwm
  ledcSetup(pwmChannel2, freq, resolution);
  ledcAttachPin(m2en, pwmChannel2);

  Serial.begin(115200);
}

double voltaje = 2;

void motor1(double voltaje)
{
  double DC = 0;
  double valor = 0;

  if (voltaje > 0)
  {
    DC = 100 / 3.3 * voltaje;
    valor = 1023 / 100.0 * DC;
    ledcWrite(pwmChannel1, valor);
    digitalWrite(m1in1, HIGH);
    digitalWrite(m1in2, LOW);
  }

  else if (voltaje < 0)
  {
    DC = 100 / 3.3 * voltaje;
    valor = -1023 / 100.0 * DC;
    ledcWrite(pwmChannel1, valor);
    digitalWrite(m1in1, LOW);
    digitalWrite(m1in2, HIGH);
  }

  else
  {
    DC = 0;
    valor = 0;
    ledcWrite(pwmChannel1, 0);
    digitalWrite(m1in1, LOW);
    digitalWrite(m1in2, LOW);
  }

  Serial.println("---");
  Serial.println(voltaje);
  Serial.println(DC);
}

void motor2(double voltaje)
{
  double DC = 0;
  double valor = 0;

  if (voltaje > 0)
  {
    DC = 100 / 3.3 * voltaje;
    valor = 1023 / 100.0 * DC;
    ledcWrite(pwmChannel2, valor);
    digitalWrite(m2in1, HIGH);
    digitalWrite(m2in2, LOW);
  }

  else if (voltaje < 0)
  {
    DC = 100 / 3.3 * voltaje;
    valor = -1023 / 100.0 * DC;
    ledcWrite(pwmChannel2, valor);
    digitalWrite(m2in1, LOW);
    digitalWrite(m2in2, HIGH);
  }

  else
  {
    DC = 0;
    valor = 0;
    ledcWrite(pwmChannel2, 0);
    digitalWrite(m2in1, LOW);
    digitalWrite(m2in2, LOW);
  }

  Serial.println("---");
  Serial.println(voltaje);
  Serial.println(DC);
}

void loop()
{
  int t = 50;
  double volt=2;
  for (int i = 0; i < 2000; i++)
  {
    motor1(volt);
    motor2(volt);
    delay(t);
    motor1(-volt);
    motor2(-volt);
    delay(t);
  }
  motor1(0);
  motor2(0);
  delay(1000000);
}