#include <Arduino.h>
#include "defines.h"
#include "CtrlMotores.h"

#define Kp1050 40 //R2D PENDIENTE Constate proprocional del PD del test 5


extern int estado;

extern float omega;
extern float azimutRef;
extern float outPID;

// Variables Estado 1050, pid en velocidad angular
static unsigned long  tiempoEstado1050=0;


      //////////////////////////////////////////////////////////////////////////
      // Estado 1050, Pruebas PROPORCIONAL sobre VELOCIDAD ANGULAR DEL TYPE_gYROSCOPE
      //////////////////////////////////////////////////////////////////////////



void Test05iiiaaa()
{
		
        if ((tiempoEstado1050 + 10000) > millis()) {
          //Control Proporocional
		  motorIzqPWM (225 + (int)( omega * Kp1050),0);
          motorDchPWM (225 - (int)( omega * Kp1050),0);  //CONTROL PROPOCIONAL SOBRE UNA DE LAS RUEDAS

        } else {
          estado = 0;
          pararMotores();
        }

	
	
}


void Test05iiittt()
{
	       //Realiza un control PD basado en la velocidad angular
        estado = 1050;

        tiempoEstado1050 = millis();
        azimutRef = 100;
        outPID = 0;

		motorIzqPWM (240,0);  //Valores calculados a mano OJO!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        motorDchPWM (230,0); 
	
}
