#include <Arduino.h>
#include <math.h>

#include "defines.h"
#include "CtrlMotores.h"

extern int estado;


extern float parameters[21];

extern float omega;

// Variables Estado 1080
// Representación gráfica (lineal) PWM-Velocidad Angular
static float PWM_Omega[8]; // Valores de velocidad angular máximo y mínimos, Orden: D_avanza, I_avanza, D_retrocede, I_retrocede
static int numOmegas;      // Numero de omegas para promediar
static unsigned long tiempoOmega=0;
static float mediaOmega=0.0;
static int incPWM=0;

void pararMotores();


      //////////////////////////////////////////////////////////////////////////
      // Estado 1080 y siguientes 1081,1082,1083:
      // Buscando valores de PWM de velocidades MAX / MIN angulares, para después
      // interpolar por una línea
      // 20 segundos girando y aumentando la velocidad, cambiando de sentido
      // permite obtener de forma gráfica la relacion entre valores del PWM
      // y la velocidad angular dada por giróscopo.
      //////////////////////////////////////////////////////////////////////////



void Test08iiiaaa()
{
		
        if (estado == 1080) {
		  motorIzqPWM (0,0);  
          motorDchPWM (incPWM,0); 
        } else if (estado == 1081) {
		  motorIzqPWM (incPWM,0);  
          motorDchPWM (0,0); 
        } else if (estado == 1082) {
		  motorIzqPWM (0,0);  
          motorDchPWM (0,incPWM); 
        } else if (estado == 1083) {
		  motorIzqPWM (0,incPWM);  
          motorDchPWM (0,0); 
        }

        // Despues de 10 segundos girando y aumentando el PWM entre la diferencia de ((int)parameters[18]) y ((int)parameters[19]), cambia de sentido

        if (abs(omega) > 0.02) {
          numOmegas += 1;
          mediaOmega += omega;
        }


        if ((tiempoOmega + 20000) < millis() && incPWM <= ((int)parameters[18])) {
          int aux;
          pararMotores();
          delay(500);
          tiempoOmega = millis();
          aux = 2 * (estado - 1080) + ((incPWM - ((int)parameters[19])) / (((int)parameters[18]) - ((int)parameters[19])));
          PWM_Omega[aux] = (float)(mediaOmega / numOmegas);
          //Serial.println(";" + String(aux) + ";" + incPWM + ";" + String(PWM_Omega[aux]) + ";" + String(mediaOmega) + ";" + String(numOmegas)  + ";" + "###");
          Serial.println("oooccc" + String(estado) + "ww" + String(aux) + "ww" + incPWM + "ww" + String(PWM_Omega[aux]) + "ww" + String(mediaOmega) + "ww" + String(numOmegas)  + "ww" + "###");
          mediaOmega = 0;
          numOmegas = 0;
          incPWM = incPWM + (((int)parameters[18]) - ((int)parameters[19]));
        } else if (incPWM > ((int)parameters[18]) && estado == 1083) {
          estado = 0;
        } else if (incPWM > ((int)parameters[18]) && (estado >= 1080 && estado < 1083)) {
          //Cambio de sentido de giro
          delay(2000);
          estado = estado + 1;
          mediaOmega = 0;
          numOmegas = 0;
          incPWM = ((int)parameters[19]); //Valor inicial del PWM
          tiempoOmega = millis();
        }

        // Imprime la tabla PWM / velocidad angular en un CSV para fácil representación y mmanipulación de datos
        Serial.println("oooccc" + String(estado) + "ww" + String(omega) + "ww" + incPWM + "ww"  + "###");

	
}


void Test08iiittt()
{
	        //estado que crea tabla w / pwm. Calibración automática
        estado = 1080;
        incPWM = ((int)parameters[19]); //Valor inicial del PWM
        tiempoOmega = millis();
        numOmegas = 0;
        mediaOmega = 0;


	
	
}
