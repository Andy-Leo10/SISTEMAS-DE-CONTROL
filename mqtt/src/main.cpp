#include <Arduino.h>
#include <XSpaceV2.h>

XSpaceV2 tarjeta;
Controller controlador;
double vel;

void Mqtt_Callback();

void ControlDeVelocidad(void *parameters)
{
  double kp = 0.00457;
  double ki = 0.0229;
  double T = 0.01;

  double uk, ek, vel;
  double uk_1 = 0, ek_1 = 0;

  tarjeta.DRV8837_Wake();

  while (1)
  {
    vel = tarjeta.GetEncoderSpeed(DEGREES_PER_SECOND);

    ek = vel_ref - vel;
    uk = controlador.leyControl(vel,vel_ref,);

    ek_1 = ek;
    uk_1 = uk;

    tarjeta.DRV8837_Voltage(uk);
    vTaskDelay(10); // espera de 10ms que es el tiempo de muestreo T
  }
}

void MqttTASK(void *parameters)
{
  while (1)
  {
    // esta es mi tarea1

    // no se usa ya delay()
    vTaskDelay(1000);
  }
  vTaskDelete(NULL);
  // para autodestruirse
}

void setup()
{
  // frecuencia de la pwm es 20k
  // resolucion del encoder+reduccionEngranaje
  tarjeta.init(20000, 1280);
  // vel=tarjeta.GetEncoderSpeed(DEGREES_PER_SECOND)
  tarjeta.Wifi_init("redpucp", "C9AA28BA93");
  tarjeta.Mqtt_init("www.xspace.pe", 1883);

  // tarjeta.Mqtt_Connect("IDusuario","usuario","contrasena");

  uint32_t espacioPila = 2000;
  xTaskCreate(MqttTASK, "nada", espacioPila, NULL, 1, NULL);

  // put your setup code here, to run once:
}

void loop()
{
  // put your main code here, to run repeatedly:
  Serial.begin(115200);
  if (!tarjeta.Mqtt_IsConnected())
  {
    tarjeta.Mqtt_Connect("IDusuario", "usuario", "contrasena");
    tarjeta.Mqtt_Suscribe("xspacev2/clase922");
  }
  tarjeta.Mqtt_KeepAlive();
  delay(1000);
}