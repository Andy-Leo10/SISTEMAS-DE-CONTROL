#include <Arduino.h>
#include <XSpaceV2.h>

XSpaceV2Board mcu;
XSpaceControl controlador;

double vel_ref = 0;
// dato que recibo
double pos_ref = 90;
// dato que envio
double mqttpos = 0;
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
  pos_ref = incoming.toDouble();
}

void ControlPosicion(void *pvParameters)
{
  double pos_real, vel_sp;
  while (1)
  {
    pos_real = mcu.GetEncoderPosition(DEGREES);
    vel_ref = (pos_ref - pos_real) * 2.5;
    Serial.println(pos_real);
    vTaskDelay(20);
  }
}

void ControlVelocidad(void *pvParameters)
{
  double uk, ek, vel;
  double uk_1 = 0, ek_1 = 0;

  mcu.DRV8837_Wake();

  while (1)
  {
    vel = mcu.GetEncoderSpeed(DEGREES_PER_SECOND);

    ek = vel_ref - vel;
    // mi motor xspace
    uk = 0.006395 * ek - 0.006205 * ek_1 + uk_1;
    // motor lab
    // uk=0.02011*ek-0.01911*ek_1+uk_1;
    ek_1 = ek;
    uk_1 = uk;

    mcu.DRV8837_Voltage(uk);

    // Serial.print("valor de control: ");
    // Serial.println(uk);

    // Serial.print("velocidad: ");
    // Serial.println(vel);

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
      mcu.Mqtt_Suscribe("xspacev2/orden");
    }

    if (mcu.Mqtt_IsConnected())
    {

      if (mcu.sw1_presionado())
      {
        Serial.println("presionado 1");
        mqttpos += 45;
      }

      if (mcu.sw2_presionado())
      {
        Serial.println("presionado 2");
        mqttpos -= 45;
      }

      ((String)mqttpos).toCharArray(buffer, 10);
      mcu.Mqtt_Publish("xspacev2/leo/pos", buffer);
      vTaskDelay(10000); // cada segundo se hace esta tarea
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
  mcu.init(20000, 960); // para pwm de motor
  // resoluacion motor lab=1280
  // resolucion motor xspace=960

  // mcu.Wifi_init("OozmaKappa", "Ax3dr315");
  mcu.Wifi_init("redpucp", "C9AA28BA93");
  mcu.Mqtt_init("34.95.131.183", 1883);

  // crear tareas
  xTaskCreatePinnedToCore(ControlVelocidad, "", 4000, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(ControlPosicion, "", 4000, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(ComunicacionMqtt, "", 4000, NULL, 3, NULL, 0);
}

void loop()
{
}