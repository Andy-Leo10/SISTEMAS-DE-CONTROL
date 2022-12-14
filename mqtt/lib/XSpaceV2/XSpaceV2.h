/*
  XSpaceV2 Library

  Version   :  2.0
  Autor     :  Pablo Cardenas
  Fecha     :  4/15/2022
  Hora      :  10:15pm

*/

#ifndef XSPACEV2_H
#define XSPACEV2_H

#include <Arduino.h>
#include <stdint.h>
#include <WiFi.h>
#include <PubSubClient.h>

#define IN1 32
#define IN2 33
#define nSLEEP 25

#define encoder_CHA 34
#define encoder_CHB 35

#define DEGREES_PER_SECOND 1
#define RADS_PER_SECOND 2

#define DEGREES 1
#define RADS 2

extern volatile double Periodo;

class XSpaceV2
{
private:
  double vel_ant = 0;
  bool XSpace_info = false;
  double _enc_resolution;

public:
  void init(int freq, double enc_resolution);
  void DRV8837_Sleep();
  void DRV8837_Wake();
  void DRV8837_Voltage(double vp);

  double GetEncoderSpeed(int modo);
  double GetEncoderPosition(int modo);

  void SerialInfo(bool mode);
  void Wifi_init(const char *ssid, const char *password);
  void Mqtt_init(const char *mqtt_server, uint16_t mqtt_port);
  void Mqtt_Connect(const char *clientId, const char *mqtt_user, const char *mqtt_pass);
  void Mqtt_Publish(const char *topic, const char *payload);
  void Mqtt_Suscribe(const char *topic);
  void Mqtt_KeepAlive();

  bool Mqtt_IsConnected();
};

/***********************************************/

#define FORWARD_EULER 1

class Controller
{
private:
    /* data */
public:
    Controller(/* args */);
    ~Controller();
    double Controller::leyControl(double var_real, double var_ref,double kp, double ki, double Ts,int aproximation);
};

Controller::Controller(/* args */)
{
}

Controller::~Controller()
{
}

#endif