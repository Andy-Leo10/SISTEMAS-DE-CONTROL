#include <Arduino.h>
#include <XSpaceV2.h>

XSpaceV2Board mcu;
XSpaceControl controlador;

double vel_ref = 0;
// dato que recibo
double pos_ref = 90;
// dato que envio

void ControlPosicion(void *pvParameters)
{
  double pos_real, vel_sp;
  while (1)
  {
    pos_real = mcu.GetEncoderPosition(DEGREES);
    vel_ref = (pos_ref - pos_real) * 2;
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
    // uk = 0.03197 * ek - 0.03103 * ek_1 + uk_1;
    // motor lab
    uk = 0.02011 * ek - 0.01911 * ek_1 + uk_1;
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

void setup()
{
  Serial.begin(115200);
  mcu.SerialInfo(true); // habilita enviar por monitor serial
  // resolucion del encoder+reduccionEngranaje
  mcu.init(20000, 1280); // para pwm de motor
  // resoluacion motor xspace=1280
  // resolucion motor lab=960

  // crear tareas
  xTaskCreatePinnedToCore(ControlVelocidad, "", 4000, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(ControlPosicion, "", 4000, NULL, 2, NULL, 0);
}

void loop()
{
}