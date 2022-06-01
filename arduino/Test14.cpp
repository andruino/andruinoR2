#include "defines.h"
#include <Arduino.h>

extern int estado;

extern float incAzimut;
extern float azimut;
extern float cmdVel;
extern float cmdOmega;
extern float ICCR;

extern int  outPID_arco;
extern int estadoPID;

extern float parameters[21];


//Variables Estado 14 (1140), describe media circunferencia
static unsigned long tiempoEstado1140 = 0;
static float azimutEstado1140;


void pararMotores();
int movimiento_arco(int PWM_I, int PWM_D);
int inicia_arco();
float cmd_omegaDer(float cmd_vel, float cmd_ang);
float cmd_omegaIzq(float cmd_vel, float cmd_ang);
int cmd_PWMDer(float omegaDer);
int cmd_PWMIzq(float omegaIzq);



void Test14iiiaaa()
{
	
	      if (incAzimut != azimut) { //Realiza un arco de circunferencia, vel!=0 y omega!=0

          movimiento_arco(cmd_PWMIzq(cmd_omegaIzq(parameters[0] * cmdVel, parameters[1] * cmdOmega)), cmd_PWMDer(cmd_omegaDer(parameters[0] * cmdVel, parameters[1] * cmdOmega)));

          Serial.println("oooccc" + String(estado) + "ww" + String(azimutEstado1140) + "ww" + String(azimut) + "ww" + String(millis() - tiempoEstado1140) + "ww" + String(ICCR) + "ww" + String(outPID_arco) + "ww###"); //En radianes
        }

        if ( ((float)  atan2((double)sin(azimutEstado1140 - azimut), (double)cos(azimutEstado1140 - azimut)) <= (0.00)) && (millis() > tiempoEstado1140 + 500)) {
          pararMotores();
          estado = 0;
          estadoPID = -1;
        }
  
}

void Test14iiittt()
{
        estado = 1140;
        tiempoEstado1140 = millis();
        azimutEstado1140 = azimut;

        cmdVel = 0.15;
        cmdOmega = 0.5;

        ICCR = 0.5 * (parameters[3] / parameters[4]) * DISTANCIA_RUEDAS * ( cmd_omegaDer(parameters[0] * cmdVel, parameters[1] * cmdOmega) + cmd_omegaIzq(parameters[0] * cmdVel, parameters[1] * cmdOmega) ) / ( cmd_omegaDer(parameters[0] * cmdVel, parameters[1] * cmdOmega) - cmd_omegaIzq(parameters[0] * cmdVel, parameters[1] * cmdOmega) );

        inicia_arco();
        Serial.println("ooocccazimutEstado1140wwazimutwwTiempowwICCRwwoutPID_arcoww###");
        movimiento_arco(cmd_PWMIzq(cmd_omegaIzq(parameters[0] * cmdVel, parameters[1] * cmdOmega)), cmd_PWMDer(cmd_omegaDer(parameters[0] * cmdVel, parameters[1] * cmdOmega)));

}
