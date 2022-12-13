#include <Arduino.h>
#include <XSpaceV2.h>
//#include <PubSubClient.h>

volatile double T_derecho = 0;
volatile double Periodo_derecho = 1000000;
volatile double Tant_derecho = 0;

volatile double T_izquierdo = 0;
volatile double Periodo_izquierdo = 1000000;
volatile double Tant_izquierdo = 0;

void IRAM_ATTR ISR_encoder_derecho()
{
    T_derecho = micros();
    //Serial.println("llega");
    //Serial.println("llega");
    if(((T_derecho-Tant_derecho) > 5000)){
        
        Periodo_derecho = T_derecho - Tant_derecho;
        Tant_derecho = T_derecho;

        if(digitalRead(CLK_derecho) == LOW){
            Periodo_derecho = (-1)*Periodo_derecho;
        }

    // digitalWrite(21,!digitalRead(21));
    }
}

void IRAM_ATTR ISR_encoder_izquierdo()
{
    T_izquierdo = micros();
    //Serial.println("llega");
   if(((T_izquierdo-Tant_izquierdo) > 5000)){
        Periodo_izquierdo = T_izquierdo - Tant_izquierdo;
        Tant_izquierdo = T_izquierdo;

        if(digitalRead(CLK_izquierdo) == LOW){
            Periodo_izquierdo = (-1)*Periodo_izquierdo;
        }
    // digitalWrite(21,!digitalRead(21));
    }  
}

void XSpaceV2::init(int freq, double enc_resolution){

    ledcAttachPin(rueda_derechaEN, 1);
    ledcAttachPin(rueda_izquierdaEN, 2);
    ledcSetup(1, freq, 10);
    ledcSetup(2, freq, 10);

    pinMode(rueda_derechaIN1, OUTPUT);
    pinMode(rueda_derechaIN2, OUTPUT);
    pinMode(rueda_izquierdaIN1, OUTPUT);
    pinMode(rueda_izquierdaIN2, OUTPUT);

    pinMode(CLK_derecho,INPUT);
    pinMode(DT_derecho,INPUT);
    pinMode(CLK_izquierdo, INPUT);
    pinMode(DT_izquierdo, INPUT);

    attachInterrupt(DT_derecho, ISR_encoder_derecho,FALLING);
    attachInterrupt(DT_izquierdo, ISR_encoder_izquierdo, FALLING);

    //pinMode(encoder_CHA,INPUT_PULLUP);
    //pinMode(encoder_CHB,INPUT_PULLUP);
    //attachInterrupt(encoder_CHA, ISR_encoder, RISING);

    //this-> _enc_resolution = enc_resolution;

}

void XSpaceV2::Voltage(double vp){

    double vm = 3.3;

    if(vp>3.3) vp = 3.3;
    if(vp<-3.3) vp = -3.3;
    int Duty = (int) ( (abs(vp)/vm) * 1024.0);

// 
    if(vp<0){
        ledcWrite(1, Duty);
        digitalWrite(rueda_derechaIN1,HIGH);
        digitalWrite(rueda_derechaIN2, LOW);

        ledcWrite(2, Duty);
        digitalWrite(rueda_izquierdaIN1,LOW);
        digitalWrite(rueda_izquierdaIN2, HIGH);
        
    }else{
        ledcWrite(1, Duty);
        digitalWrite(rueda_derechaIN1,HIGH);
        digitalWrite(rueda_derechaIN2, LOW);

        ledcWrite(2, Duty);
        digitalWrite(rueda_izquierdaIN1,HIGH);
        digitalWrite(rueda_izquierdaIN2, LOW);
    }
    Serial.println(Duty);
}

double XSpaceV2::GetEncoderSpeed_derecho(int modo){
    double vel=0;

    switch (modo)
    {
    case DEGREES_PER_SECOND:
        vel = 18*1000000/(Periodo_derecho);
        // if(abs(vel)>800)vel=vel_ant;
        // if(abs(vel-vel_ant)>100)vel=vel_ant;
        break;
    case RADS_PER_SECOND:
        vel = 6544.984694978736/Periodo_derecho;
        break;
    
    default:
        break;
    }

    vel_ant = vel;

    return vel;
}


double XSpaceV2::GetEncoderSpeed_izquierdo(int modo){
    double vel=0;

    switch (modo)
    {
    case DEGREES_PER_SECOND:
        vel = 18*1000000/(Periodo_izquierdo);
        // if(abs(vel)>800)vel=vel_ant;
        // if(abs(vel-vel_ant)>100)vel=vel_ant;
        break;
    case RADS_PER_SECOND:
        vel = 6544.984694978736/Periodo_izquierdo;
        break;
    
    default:
        break;
    }

    vel_ant = vel;

    return vel;
}





