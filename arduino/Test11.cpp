#include "defines.h"
#include <Arduino.h>
#include <math.h>

extern int estado;

extern float azimut;
extern float azimutRef;

extern float secuencia;

extern int outPD;

//Variables Estado 11 (1110), prueba la función de giro
unsigned long tiempoEstado1110 = 0;
int contadorEstado1110 = 0; //Contador de estado de sobreoscilaciones del PD de giro


void pararMotores();
int movimiento_giro();
int inicia_giro(float azimutGira);


      // Estado 1110, debe girar

void Test11iiiaaa()
{
			  
        if  (abs(azimutRef - azimut) > 0.08) {  //antes 0.04

          movimiento_giro();
          estado = 1110;

        } else {

          contadorEstado1110 += 1;
          if (contadorEstado1110 > 5)
            estado = 0;
          pararMotores();
          secuencia = 0.0;

        }

        secuencia++;
        Serial.println("oooccc" + String(estado) + "ww" + String(azimutRef) + "ww" + String(azimut) + "ww" + String(millis()) + "ww" + String(outPD) + "ww" + String(secuencia) + "ww###"); //En radianes

}

void Test11iiittt()
{
	
	        //Giro
        estado = 1110;
        tiempoEstado1110 = millis();
        contadorEstado1110 = 0; // Contador de sobreoscilación
        inicia_giro((float)  atan2((double)sin(azimut - 3.1416), (double)cos(azimut - 3.1416)));

}
