#include <Arduino.h>
#include "defines.h"
#include "CtrlMotores.h"

extern int estado;
extern float omega;


static int incPWM=0;
static unsigned long tiempoOmega=0;
static float omegaLP=0.0;



void pararMotores();

      
      //////////////////////////////////////////////////////////////////////////
      // Estado 1100 y siguientes 1101,1102,1103....:
      // Trata de calcular la acelararcion angular máxima pasando de PWMe 180 a PWM de 250
      // y viendo el tiempo
      //////////////////////////////////////////////////////////////////////////

void Test10iiiaaa()
{
	    if (estado == 1100) {
          incPWM = 180;
		  motorIzqPWM (0,0);  
          motorDchPWM (incPWM,0); 
        } else if (estado == 1101) {
          incPWM = 250;
		  motorIzqPWM (0,0);  
          motorDchPWM (incPWM,0); 
        }


        if ((tiempoOmega + 20000) < millis()) {
          estado = estado + 1;
          tiempoOmega = millis();
        }
        //
        if (estado >= 1102) {
          pararMotores();
          delay(500);
          estado = 0;

        }

        //Filtro paso de baja (para quitar el ruido del giróscopo)
        //y[i] := y[i-1] + α * (x[i] - y[i-1])
        omegaLP = omegaLP + 0.25 * (omega - omegaLP);

        // Imprime la tabla PWM / velocidad angular en un CSV para fácil representación y mmanipulación de datos.
        Serial.println("oooccc" + String(estado) + "ww" + String(omega) + "ww" + incPWM + "ww" + String(millis()) + "ww" + tiempoOmega + "ww" +  String(omegaLP) + "ww###"); //En radianes

}


void Test10iiittt()
{
	   //Test 10: Trata de calcular la aceleración angular máxima.
        estado = 1100;
        incPWM = 170; //Valor inicial del PWM
        tiempoOmega = millis();
        omegaLP = 0; //Valor inicial del filtro LP
	
}
