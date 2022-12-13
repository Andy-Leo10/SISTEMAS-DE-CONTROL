#include <Arduino.h>

#define A 17 			//variable A a pin digital (DT en modulo)
#define B 16  			//variable B a pin digital (CLK en modulo)
        
int ANTERIOR = 0;		// almacena valor anterior de la variable POSICION

volatile int POSICION = 0;	// variable POSICION con valor inicial de 50 y definida
				// como global al ser usada en loop e ISR (encoder)
volatile double tiempoInterrupcion=0;
volatile double ultimaInterrupcion=0;

void IRAM_ATTR ISR_encoder()  {

  //tiempoInterrupcion = millis();	// variable almacena valor de func. millis
  tiempoInterrupcion = micros();	// variable almacena valor de func. millis

  if (tiempoInterrupcion - ultimaInterrupcion > 5000) {	// rutina antirebote desestima
							// pulsos menores a 5 mseg.
    if (digitalRead(B) == HIGH)			// si B es HIGH, sentido horario
    {
      POSICION++ ;				// incrementa POSICION en 1
    }
    else {					// si B es LOW, senti anti horario
      POSICION-- ;				// decrementa POSICION en 1
    }

    ultimaInterrupcion = tiempoInterrupcion;	// guarda valor actualizado del tiempo
  }						// de la interrupcion en variable static
}

void setup() {
  pinMode(A, INPUT_PULLDOWN);		// A como entrada
  pinMode(B, INPUT);		// B como entrada

  Serial.begin(115200);		// incializacion de comunicacion serie a 9600 bps

  attachInterrupt(A, ISR_encoder, FALLING);// interrupcion sobre pin A con
							  // funcion ISR encoder y modo LOW
  Serial.println("quePasa");
}


void loop() {
  if (POSICION != ANTERIOR) {	// si el valor de POSICION es distinto de ANTERIOR
    Serial.println(POSICION);	// imprime valor de POSICION
    ANTERIOR = POSICION ;	// asigna a ANTERIOR el valor actualizado de POSICION
  }
}

