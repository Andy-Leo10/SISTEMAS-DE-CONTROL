#include <Arduino.h>
#include <XSpaceV2.h>

XSpaceV2 mcu;
double vel_ref = 0;
double mqttvel = 0;
char buffer[50];

void Mqtt_Callback(char *topic, byte *payload, unsigned int length)
{
  String incoming = "";
  Serial.print("mensaje recibido desde -> ");
  Serial.println(topic);

  for (int i = 0; (i < length); i++)
  {
    incoming = incoming + (char)payload[i];
  }
  Serial.println("mensaje -> " + incoming);
  vel_ref = incoming.toDouble();
}

void ControlVelocidad(void *pvParameters)
{
  double kp = 0.00457;
  double ki = 0.0229;
  double T = 0.01;

  double uk, ek, vel;
  double uk_1 = 0, ek_1 = 0;

  mcu.DRV8837_Wake();

  while (1)
  {
    vel = mcu.GetEncoderSpeed(DEGREES_PER_SECOND);

    ek = vel_ref - vel;
    uk = (kp + T / 2 * ki) * ek + (T / 2 * ki - kp) * ek_1 + uk_1;

    ek_1 = ek;
    uk_1 = uk;

    mcu.DRV8837_Voltage(uk);
    vTaskDelay(10); // espera de 10ms que es el tiempo de muestreo T
  }
}

void ComunicacionMqtt(void *pvParameters)
{
  while (1)
  {
    if (!mcu.Mqtt_IsConnected())
    {
      mcu.Mqtt_Connect("SoyLeo", "user", "psw");
      mcu.Mqtt_Suscribe("xspacev2/setpoint");
    }

    if (mcu.Mqtt_IsConnected())
    {
      mqttvel = mcu.GetEncoderSpeed(DEGREES_PER_SECOND);
      ((String)mqttvel).toCharArray(buffer, 10);
      mcu.Mqtt_Publish("xspacev2/leo/velocidad", buffer);
      vTaskDelay(1000); // cada segundo se hace esta tarea
    }
    mcu.Mqtt_KeepAlive();
  }
  vTaskDelete(NULL);
}

void setup()
{
  Serial.begin(115200);
  mcu.SerialInfo(true); // habilita enviar por monitor serial
  // resolucion del encoder+reduccionEngranaje
  mcu.init(20000, 1280); // para pwm de motor
  mcu.Wifi_init("OozmaKappa", "Ax3dr315");
  mcu.Mqtt_init("34.95.131.183", 1883);

  // crear tareas
  xTaskCreatePinnedToCore(ControlVelocidad, "", 4000, NULL, 1, NULL, 0);
  //xTaskCreatePinnedToCore(ComunicacionMqtt, "", 4000, NULL, 1, NULL, 0);//
}

void loop()
{
  // put your main code here, to run repeatedly:
}