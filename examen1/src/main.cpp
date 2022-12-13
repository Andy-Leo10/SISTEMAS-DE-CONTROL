#include <Arduino.h>
#include <XSpaceV2.h>

XSpaceV2Board mcu;
XSpaceControl controlador;

// dato que recibo
double vel_ref = 0;
// dato que envio
double mqttpos = 0;

char buffer[50];
double tempRef = 225;

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
  double uk, ek, vel;
  double x1, x2, x2_1 = 0, ek_1 = 0;
  double Ts = 0.001;
  double k2 = -0.5191, k1 = 0.0282;

  mcu.DRV8837_Wake();

  while (1)
  {
    vel = mcu.GetEncoderSpeed(DEGREES_PER_SECOND);
    ek = vel_ref - vel;
    x1 = vel;
    x2 = ek_1 * Ts + x2_1;

    // mi motor xspace
    uk = -k2 * x2 - k1 * x1;
    // motor lab
    // uk=0.02011*ek-0.01911*ek_1+uk_1;
    ek_1 = ek;
    x2_1 = x2;

    mcu.DRV8837_Voltage(uk);
    // Serial.print("velocidad: ");
    Serial.println(vel);

    vTaskDelay(1); // espera de 1ms que es el tiempo de muestreo T
  }
  vTaskDelete(NULL);
}

void ComunicacionMqtt(void *pvParameters)
{
  while (1)
  {
    if (!mcu.Mqtt_IsConnected())
    {
      mcu.Mqtt_Connect("SoyLeo", "user", "psw");
      // mcu.Mqtt_Suscribe("xspacev2/orden");
      mcu.Mqtt_Suscribe("xspacev2/leo/pos");
    }

    if (mcu.Mqtt_IsConnected())
    {

      if (mcu.sw1_presionado())
      {
        Serial.println("presionado 1");
        mqttpos += 50;
      }

      if (mcu.sw2_presionado())
      {
        Serial.println("presionado 2");
        mqttpos -= 50;
      }

      ((String)mqttpos).toCharArray(buffer, 10);
      mcu.Mqtt_Publish("xspacev2/leo/pos", buffer);
      vTaskDelay(200); 
    }
    mcu.Mqtt_KeepAlive();
  }
  vTaskDelete(NULL);
}

void controlTemperatura(void *pvParameters)
{
  double tempRead, pwm;
  while (1)
  {
    tempRead = mcu.temperatura();
    pwm = controlador.control_temperatura(tempRead, tempRef, 0.233, 0.1962);
    mcu.DRV8837_Voltage(pwm * 5);
    vTaskDelay(26 * 1000);
  }
}

void setup()
{
  Serial.begin(115200);
  mcu.SerialInfo(true); // habilita enviar por monitor serial
  // resolucion del encoder+reduccionEngranaje
  mcu.init(20000, 1280); // para pwm de motor
  // resoluacion motor xspace=1280
  // resolucion motor lab=960

  mcu.Wifi_init("OozmaKappa", "Ax3dr315");
  // mcu.Wifi_init("redpucp", "C9AA28BA93");
  // mcu.Mqtt_init("34.95.131.183", 1883);

  // crear tareas
  xTaskCreatePinnedToCore(ControlVelocidad, "", 4000, NULL, 1, NULL, 0);

  xTaskCreatePinnedToCore(ComunicacionMqtt, "", 4000, NULL, 2, NULL, 0);

  // xTaskCreatePinnedToCore(controlTemperatura, "", 4000, NULL, 1, NULL, 0);
}

void loop()
{
  if (mcu.sw1_presionado())
  {
    Serial.println("presionado 1");
    vel_ref += 50;
  }

  if (mcu.sw2_presionado())
  {
    Serial.println("presionado 2");
    vel_ref -= 50;
  }
  delay(500);
}