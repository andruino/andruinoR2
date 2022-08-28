#include <Arduino.h>
#include "defines.h"
#include "CtrlMotores.h"


extern int estado;

extern float omega;
extern float parameters[21];

static float omegaLP=0.0;

static int incPWM=0;
static unsigned long tiempoOmega=0;
static unsigned long  tiempoEstado1040=0;

      //////////////////////////////////////////////////////////////////////////
      // Estado 1040 y siguientes 1041,1042,1043:
      // Creando la tabla de velocidades angulares / PWM
      // 20 segundos girando y aumentando la velocidad, cambiando de sentido
      // permite obtener de forma gráfica la relacion entre valores del PWM
      // y la velocidad angular dada por giróscopo.
      //////////////////////////////////////////////////////////////////////////



void Test04iiiaaa()
{
			  
        if (estado == 1040) {
		  motorIzqPWM (0,0);
          motorDchPWM (incPWM,0);	
        } else if (estado == 1041) {
          motorIzqPWM (incPWM,0);
          motorDchPWM (0,0); 
        } else if (estado == 1042) {
		  motorIzqPWM (0,0);
          motorDchPWM (0,incPWM);	
        } else if (estado == 1043) {
          motorIzqPWM (0,incPWM);
          motorDchPWM (0,0); 
        }
        // Despues de 10 segundos girando y aumentando el PWM en 10, cambia de sentido

        if ((tiempoOmega + 5000) < millis() && incPWM <= ((int)parameters[18])) {
          tiempoOmega = millis();
          incPWM = incPWM + 10;
        } else if (incPWM > ((int)parameters[18]) && estado == 1041) {
          estado = 0;
          pararMotores();
        } else if (incPWM > ((int)parameters[18]) && (estado >= 1040 && estado < 1041)) { 
          //Cambio de sentido de giro
          pararMotores();
          delay(500);
          estado = estado + 1;
          incPWM = ((int)parameters[19]); //Valor inicial del PWM
          tiempoOmega = millis();
          tiempoEstado1040 = millis() ;
          omegaLP = 0;
          omega = 0;
          //Arranque de motores
          if (estado == 1040) {
			motorIzqPWM (0,0);
            motorDchPWM (255,0); 
          } else if (estado == 1041) {
			motorIzqPWM (255,0);
            motorDchPWM (0,0); 
          } else if (estado == 1042) {
			motorIzqPWM (0,0);
            motorDchPWM (0,255); 
          } else if (estado == 1043) {
			motorIzqPWM (0,255);
            motorDchPWM (0,0); 
          }

        }
        //Filtro paso de baja (para quitar el ruido del giróscopo)
        if (omegaLP == 0)
          omegaLP = omega;
        omegaLP = omegaLP + 0.1 * (omega - omegaLP);

        // Imprime la tabla PWM / velocidad angular en un CSV para fácil representación y mmanipulación de datos.
        // Solo valores válidos después tres segundo espues del inicio(elimina transitorios) y tres segundos antes del fin
        if ((millis() > (tiempoOmega + 500)) && (millis() < tiempoOmega + 4500)   ) {
          Serial.println("oooccc" + String(estado) + "ww" + String(omegaLP) + "ww" + incPWM + "ww" + String(millis()) + "ww" + String(omega) + "ww###"); //En radianes
        }

	
}



void Test04iiittt()
{
	       //estado que crea tabla w / pwm
        estado = 1040;
        incPWM = ((int)parameters[19]); //Valor inicial del PWM
        tiempoOmega = millis();
        tiempoEstado1040 = 140;
        omegaLP = 0;
        // Arranque de motores
		motorIzqPWM (0,0);
        motorDchPWM (incPWM,0); 
	
}
