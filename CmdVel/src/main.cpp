#include <Arduino.h>
#include <XSpaceV2.h>

XSpaceV2 mcu;
double vel_ref = 30;

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

    //Serial.print("valor de control: ");
    //Serial.println(uk);

    //Serial.print("velocidad: ");
    Serial.println(vel);

    vTaskDelay(10); // espera de 10ms que es el tiempo de muestreo T
  }
}

void setup()
{
  Serial.begin(115200);
  mcu.SerialInfo(true); // habilita enviar por monitor serial
  // resolucion del encoder+reduccionEngranaje
  mcu.init(20000, 1280); // para pwm de motor

  // crear tareas
  xTaskCreatePinnedToCore(ControlVelocidad, "", 4000, NULL, 1, NULL, 0);
}

void loop()
{
  // put your main code here, to run repeatedly:
}