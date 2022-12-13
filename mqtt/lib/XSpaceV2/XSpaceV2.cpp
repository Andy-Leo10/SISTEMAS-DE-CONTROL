//#include <Arduino.h>
#include <XSpaceV2.h>
//#include <PubSubClient.h>

volatile double T = 0;
volatile double Periodo = 1000000;
volatile double Tant = 0;
volatile double counter = 0;

WiFiClient XSpaceV2WifiClient;
PubSubClient XSpaceV2MQTT(XSpaceV2WifiClient);
void Mqtt_Callback(char* topic, byte* payload, unsigned int length);

void IRAM_ATTR ISR_encoder()
{
    T = micros();

    if(digitalRead(encoder_CHA) == HIGH){
        Periodo = T - Tant;
        Tant = T;
        counter++;

        if(digitalRead(encoder_CHB) == LOW){
            Periodo = (-1)*Periodo;
            counter--;
        }

        //digitalWrite(21,!digitalRead(21));
    }
    

    
}


void XSpaceV2::init(int freq, double enc_resolution){
    pinMode(nSLEEP,OUTPUT);

    ledcAttachPin(IN1, 1);
    ledcAttachPin(IN2, 2);

    ledcSetup(1, freq, 10);
    ledcSetup(2, freq, 10);

    pinMode(encoder_CHA,INPUT_PULLUP);
    pinMode(encoder_CHB,INPUT_PULLUP);
    attachInterrupt(encoder_CHA, ISR_encoder, RISING);

    this->_enc_resolution=enc_resolution;
}

void XSpaceV2::DRV8837_Sleep(){
    digitalWrite(nSLEEP,LOW);
}

void XSpaceV2::DRV8837_Wake(){
    digitalWrite(nSLEEP,HIGH);
}

void XSpaceV2::DRV8837_Voltage(double vp){

    double vm = 5;

    if(vp>5) vp = 5;
    if(vp<-5) vp = -5;
    int Duty = (int) ( (1 - abs(vp)/vm) * 1024.0);


    if(vp<0){
        ledcWrite(1, Duty);
        ledcWrite(2, 1024);
    }else{
        ledcWrite(1, 1024);
        ledcWrite(2, Duty);
    }
}

double XSpaceV2::GetEncoderSpeed(int modo){
    double vel=0;

    switch (modo)
    {
    case DEGREES_PER_SECOND:
        //con resolucion de 5pulsos/revoluacion
        vel = 360000000.0/(this->_enc_resolution*Periodo);
        if(abs(vel)>800)vel=vel_ant;
        if(abs(vel-vel_ant)>100)vel=vel_ant;
        break;
    case RADS_PER_SECOND:
        vel = 6544.984694978736/Periodo;
        break;
    
    default:
        break;
    }

    vel_ant = vel;

    return vel;
}

double XSpaceV2::GetEncoderPosition(int modo){
    double pos=0;

    switch (modo)
    {
    case DEGREES:
        //con resolucion de 5pulsos/revoluacion
        pos = counter/this->_enc_resolution*360.0;
        
        break;
    case RADS:
        pos = counter/this->_enc_resolution*2*PI;
        break;
    
    default:
        break;
    }

    return pos;
}

/* Metodos reescritos para un mejor entendimiento */

void XSpaceV2::Wifi_init(const char* ssid, const char* password){
	
    if(this->XSpace_info){
        Serial.println();
	    Serial.print("Conectando a ssid: ");
	    Serial.println(ssid);
    }

	WiFi.begin(ssid, password);

	while (WiFi.status() != WL_CONNECTED) {
		delay(500);
		if(this->XSpace_info) Serial.print(".");
	}

    if(this->XSpace_info){
	    Serial.println("");
	    Serial.println("Conectado!!");
	    Serial.println("Dirección IP: ");
	    Serial.println(WiFi.localIP());
    }
    WiFi.setSleep(false);

}

void XSpaceV2::Mqtt_Connect(const char *clientId, const char *mqtt_user, const char *mqtt_pass){

	while (!XSpaceV2MQTT.connected()) {
		
		
		// Intentamos conectar
		if (XSpaceV2MQTT.connect(clientId,mqtt_user,mqtt_pass)) {
				if(this->XSpace_info) Serial.println("Conectado al Broker!");
                
		}else {
			if(this->XSpace_info){
                Serial.print("falló :( con error -> ");
			    Serial.print(XSpaceV2MQTT.state());
			    Serial.println(" Intentamos de nuevo en 5 segundos");
			    delay(5000);
            }
		}
	}

}



void XSpaceV2::Mqtt_init(const char *mqtt_server, uint16_t mqtt_port){
    XSpaceV2MQTT.setServer(mqtt_server, mqtt_port);
    XSpaceV2MQTT.setCallback([this] (char* topic, byte* payload, unsigned int length) { Mqtt_Callback(topic, payload, length); });
}

bool XSpaceV2::Mqtt_IsConnected(){
    return XSpaceV2MQTT.connected();
}

void XSpaceV2::Mqtt_Publish(const char* topic, const char* payload){
    XSpaceV2MQTT.publish(topic,payload);
}

void XSpaceV2::Mqtt_Suscribe(const char* topic){
    if(this->Mqtt_IsConnected()){
         XSpaceV2MQTT.subscribe(topic);
         if(this->XSpace_info){
             Serial.print("Suscrito a: ");
             Serial.println(topic);
         }else{
             Serial.println("Subscripcion fallida, no se tiene conexion con el broker");
         }
    }
   

}

void XSpaceV2::Mqtt_KeepAlive(){
    XSpaceV2MQTT.loop();
}

void XSpaceV2::SerialInfo(bool mode){
    this->XSpace_info = mode;
}

/***********************************************/

double Controller::leyControl(double var_real, double var_ref,double kp, double ki, double Ts,int aproximation){
    double u;
    

}
