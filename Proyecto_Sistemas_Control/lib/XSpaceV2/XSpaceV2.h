#ifndef XSPACEV2_H
#define XSPACEV2_H

#include <Arduino.h>
#include <stdint.h>
//#include <WiFi.h>
//#include <PubSubClient.h>

//#define PI 3.141592
#define rueda_derechaEN 2
#define rueda_derechaIN1 18
#define rueda_derechaIN2 19

#define rueda_izquierdaEN 4
#define rueda_izquierdaIN1 15
#define rueda_izquierdaIN2 26

#define CLK_izquierdo 16
#define DT_izquierdo 17
#define CLK_derecho 27
#define DT_derecho 23

#define DEGREES_PER_SECOND 1
#define RADS_PER_SECOND 2

extern volatile double Periodo; //extern significa que puede usarse en distintos CPP. Valatile es un tipo de dato que se usa en las rutinas de interrupcion para que se guarde la info

class XSpaceV2{
    private:
        double vel_ant = 0;
        bool XSpace_info = false;
        double _enc_resolution;

    public:
        void init(int freq, double enc_resolution);
        //void DRV8837_Sleep();
        //void DRV8837_Wake();
        void Voltage(double vp);
        
        double GetEncoderSpeed_derecho(int modo);
        double GetEncoderSpeed_izquierdo(int modo);


};



#endif