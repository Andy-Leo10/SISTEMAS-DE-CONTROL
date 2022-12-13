/*
  ROBOT TAGADA

  Version   :  estoy perdiendo la cabeza.0
  Autor     :  Andrés Leonardo
  Fecha     :  despues de un golpe de estado
  Hora      :  ya perdi la cuenta

*/

#include <Arduino.h>
#include <XSpaceV2.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

//***** para acelerometro ************************

MPU6050 sensor;
int16_t ax, ay, az;
int16_t gx, gy, gz;

long tiempo_prev;
float dt;
float ang_x, ang_y, ang_z;
float ang_x_prev, ang_y_prev, ang_z_prev;

//***** para motores ****************************

const int resolution = 10;
const int freq = 20000;
int dutyCycle = 0;
const double radio_rueda = 0.07; // en metros

// motor1 (derecho)
#define m1en 2
#define m1in1 18
#define m1in2 19
const int pwmChannel1 = 1;

// motor2 (izquierdo)
#define m2en 4
#define m2in1 15
#define m2in2 26
const int pwmChannel2 = 2;

//***** para encoders ***************************

double resolucion_encoders = 975.0; //  pulsos/vuelta
#define GRADOS_POR_SEGUNDO 1
#define RADS_POR_SEGUNDO 2

// encoder1 (derecho)
#define encoder1_CHA 16
#define encoder1_CHB 17

// encoder2 (izquierdo)
#define encoder2_CHA 23
#define encoder2_CHB 27

volatile double T = 0;
volatile double Periodo = 1000000;
volatile double Tant = 0;
volatile double counter = 0;

double vel;
double vel_ant = 0;

//***** funciones encoders ***************************

void IRAM_ATTR ISR_encoder1()
{
  // Rutina de interrupcion que permite calcular el
  // tiempo entre pulsos del canal A del encoder
  //(el canal B determina el signo)
  T = micros();

  Periodo = T - Tant;
  Tant = T;

  if (digitalRead(encoder1_CHB) == LOW)
  {
    Periodo = (-1) * Periodo;
    counter--;
  }
  else
  {
    counter++;
  }
}

void configura_encoders()
{
  pinMode(encoder1_CHA, INPUT_PULLUP);
  pinMode(encoder1_CHB, INPUT_PULLUP);
  pinMode(encoder2_CHA, INPUT_PULLUP);
  pinMode(encoder2_CHB, INPUT_PULLUP);
  attachInterrupt(encoder1_CHA, ISR_encoder1, RISING);
}

double velocidad_encoder1(int modo)
{
  vel = 0;
  switch (modo)
  {
  case GRADOS_POR_SEGUNDO:
    vel = 360000000.0 / (975.0 * Periodo);
    // vel = 360000000.0 / (resolucion_encoders * Periodo);
    //  vel = 375000.0/Periodo;
    if (abs(vel) > 800)
      vel = vel_ant;
    if (abs(vel - vel_ant) > 100)
      vel = vel_ant;
    break;

  case RADS_POR_SEGUNDO:
    vel = 360000000.0 / (resolucion_encoders * Periodo);
    vel = vel / 57.2958;
    if (abs(vel) > 800 / 57.2958)
      vel = vel_ant;
    if (abs(vel - vel_ant) > 100 / 57.2958)
      vel = vel_ant;
    break;

  default:
    break;
  }

  vel_ant = vel;
  return vel;
}

double velocidad_lineal()
{
  vel = 0;

  vel = 360000000.0 / (resolucion_encoders * Periodo);
  vel = vel / 57.2958;
  if (abs(vel) > 800 / 57.2958)
    vel = vel_ant;
  if (abs(vel - vel_ant) > 100 / 57.2958)
    vel = vel_ant;

  vel_ant = vel;
  return vel * radio_rueda;
}

double posicion_lineal()
{
  // el valor 4433.8 son los pulsos/m
  return counter / 4433.8;
}

//***** funciones motores ***************************

void configura_motores()
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
}

void motor1(double voltaje_software)
{
  double DC = 0;
  double valor = 0;
  double voltaje_max_driver = 6.0;
  double voltaje_esp32 = 0;

  if (voltaje_software > 0)
  {
    voltaje_esp32 = 3.3 / voltaje_max_driver * voltaje_software;
    DC = 100 / 3.3 * voltaje_esp32;
    valor = 1023 / 100.0 * DC;
    ledcWrite(pwmChannel1, valor);
    digitalWrite(m1in1, LOW);
    digitalWrite(m1in2, HIGH);
  }

  else if (voltaje_software < 0)
  {
    voltaje_esp32 = 3.3 / voltaje_max_driver * voltaje_software;
    DC = 100 / 3.3 * voltaje_esp32;
    valor = -1023 / 100.0 * DC;
    ledcWrite(pwmChannel1, valor);
    digitalWrite(m1in1, HIGH);
    digitalWrite(m1in2, LOW);
  }

  else
  {
    DC = 0;
    valor = 0;
    ledcWrite(pwmChannel1, 0);
    digitalWrite(m1in1, LOW);
    digitalWrite(m1in2, LOW);
  }

  // Serial.println("---");
  // Serial.println(voltaje);
  // Serial.println(DC);
}

void motor2(double voltaje_software)
{
  double DC = 0;
  double valor = 0;
  double voltaje_max_driver = 6.0;
  double voltaje_esp32 = 0;

  if (voltaje_software > 0)
  {
    voltaje_esp32 = 3.3 / voltaje_max_driver * voltaje_software;
    DC = 100 / 3.3 * voltaje_esp32;
    valor = 1023 / 100.0 * DC;
    ledcWrite(pwmChannel2, valor);
    digitalWrite(m2in1, HIGH);
    digitalWrite(m2in2, LOW);
  }

  else if (voltaje_software < 0)
  {
    voltaje_esp32 = 3.3 / voltaje_max_driver * voltaje_software;
    DC = 100 / 3.3 * voltaje_esp32;
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

  // Serial.println("---");
  // Serial.println(voltaje);
  // Serial.println(DC);
}

//***** funciones acelerometro ***************************

void configura_acelerometro()
{
  Wire.begin();        // Iniciando I2C
  sensor.initialize(); // Iniciando el sensor

  if (sensor.testConnection())
    Serial.println("Sensor iniciado correctamente");
  else
    Serial.println("Error al iniciar el sensor");
}

float angulo_inclinacion()
{
  // Lee las aceleraciones y velocidades angulares
  sensor.getAcceleration(&ax, &ay, &az);
  sensor.getRotation(&gx, &gy, &gz);

  dt = (millis() - tiempo_prev) / 1000.0;
  tiempo_prev = millis();

  // Calcular los ángulos con acelerometro
  float accel_ang_x = atan(ax / sqrt(pow(ay, 2) + pow(az, 2))) * (180.0 / 3.14);
  float accel_ang_y = atan(ay / sqrt(pow(ax, 2) + pow(az, 2))) * (180.0 / 3.14);
  float accel_ang_z = atan(az / sqrt(pow(ax, 2) + pow(ay, 2))) * (180.0 / 3.14);

  // Calcular angulo de rotación con giroscopio y filtro complemento
  ang_x = 0.98 * (ang_x_prev + (gx / 131) * dt) + 0.02 * accel_ang_x;
  ang_y = 0.98 * (ang_y_prev + (gy / 131) * dt) + 0.02 * accel_ang_y;
  ang_z = 0.98 * (ang_z_prev + (gz / 131) * dt) + 0.02 * accel_ang_z;

  ang_x_prev = ang_x;
  ang_y_prev = ang_y;
  ang_z_prev = ang_z;

  // Serial.print("Rotacion en X:  ");
  // Serial.print(ang_x);
  // Serial.print("  Rotacion en Y: ");
  // Serial.print(ang_y);
  // Serial.print("  Rotacion en Z: ");
  // Serial.println(ang_z);

  return ang_y;
}

double velocidad_inclinacion()
{
  sensor.getRotation(&gx, &gy, &gz);
  return gy * (250.0 / 32768.0);
}

//***** ROBOT DEL TEAM TAGADA ***************************

double saturador(double u_control, double valor_min, double valor_max)
{
  if (u_control < valor_min)
  {
    u_control = valor_min;
  }
  else if (u_control > valor_max)
  {
    u_control = valor_max;
  }
  return u_control;
}

void setup()
{
  configura_motores();
  configura_encoders();
  Serial.begin(115200);
  configura_acelerometro();
}

volatile double x3, x4, x3_ant;
void loop()
{
  while (1)
  {
    x3 = angulo_inclinacion();
    x4 = (x3 - x3_ant) / 0.001;
    Serial.print("derivado: ");
    Serial.print(x4);
    Serial.print("sensado: ");
    Serial.println(velocidad_inclinacion());
    x3_ant = x3;
    delay(1);
  }

  delay(3000);
  double volt_atras = -2;
  double volt_adelante = 2;
  double volt = volt_atras;

  Serial.println("clc;");
  Serial.println("clear;");
  Serial.println("close all;");
  Serial.println("t=0:0.001:1.999");

  Serial.print("Datos = [");
  for (int i = 0; i < 2000; i++)
  {
    if (i % 50 == 0)
    {
      if (volt == volt_atras)
      {
        volt = volt_adelante;
        motor1(volt);
        motor2(volt);
      }
      else
      {
        volt = volt_atras;
        motor1(volt);
        motor2(volt);
      }
    }

    Serial.print(volt);
    Serial.print(" ");
    Serial.print(angulo_inclinacion());
    Serial.print(" ");
    Serial.print(posicion_lineal());
    Serial.print(" ; ");
    delay(15); // X ms de tiempo entre muestras
  }
  Serial.println("];");

  Serial.println("plot(t,Datos);");
  motor1(0);
  motor2(0);
  delay(100000);
}