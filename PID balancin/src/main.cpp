#include <Arduino.h>
#include <XSpaceV2.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

#include <PID_v1.h>
#include <LMotorController.h>
#include "I2Cdev.h"

#define MIN_ABS_SPEED 30
MPU6050 sensor;

int16_t ax, ay, az;
int16_t gx, gy, gz;

long tiempo_prev;
float dt;
float ang_x, ang_y, ang_z;
float ang_x_prev, ang_y_prev, ang_z_prev;
//*************************************** Ajustes ***************************************
double MotorVelocidadIzq = 0.5; // double MotorVelocidadIzq = 0.3;
double MotorVelocidadDer = 0.5; // double MotorVelocidadDer = 0.3;
double PuntoEquilibrio = 5;     // grados de inclinacion
//-----------------Control de Motores
int ENA = 2;
int IN1 = 18;
int IN2 = 19;
int IN3 = 15;
int IN4 = 26;
int ENB = 4;
//------------------Los Valors de PID cambian con cada diseño
double Kp = 60;  // double Kp = 60;
double Kd = 3.5; // double Kd = 2.2;
double Ki = 250; // double Ki = 270;

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

//***************************************************************************************

// PID

double originalSetpoint = PuntoEquilibrio; // double originalSetpoint = 172.50;

double setpoint = originalSetpoint;
double movingAngleOffset = 0.1;
double input, output;

PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

double motorSpeedFactorLeft = MotorVelocidadIzq;  // double motorSpeedFactorLeft = 0.6;
double motorSpeedFactorRight = MotorVelocidadDer; // double motorSpeedFactorRight = 0.5;

LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, motorSpeedFactorLeft, motorSpeedFactorRight);

void setup()
{
  // Serial.begin(115200);
  configura_acelerometro();

  // setup PID
  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(10);
  pid.SetOutputLimits(-255, 255);
}

void loop()
{
  input = angulo_inclinacion();
  pid.Compute();
  motorController.move(output, MIN_ABS_SPEED);
}