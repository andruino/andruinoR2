#include "defines.h"
#include <Arduino.h>

extern int estado;
extern float azimut;
extern float azimutRef;
extern float cmdVel;
extern float cmdVelAnterior;

extern String trozo2Str;

extern int valores[10];

extern float parameters[21];

//Variables Estado 1070, calibración en distancia
//Debe recorrer un metro en línea recta (necesita Pid en angulo) medido con el sensor de distancia y mide el tiempo, así calcula velocidad
static int distanciaEstado1070 = 0;
static unsigned long incTiempoEstado1070;
static unsigned long tiempoEstado1070 = 0;
static float velEstado1070 = 0;



void pararMotores();
int inicia_linea_recta();
int movimiento_linea_recta(int PWM_I, int PWM_D);
float cmd_omegaDer(float cmd_vel, float cmd_ang);
float cmd_omegaIzq(float cmd_vel, float cmd_ang);
int cmd_PWMDer(float omegaDer);
int cmd_PWMIzq(float omegaIzq);


      // Estado 1070, se debe mover 0,5 metro en línea recta

void Test07iiiaaa()
{
		
        if (valores[1] > distanciaEstado1070 - DISTANCIA_CALIBRACION) {


          //Vel =0.15 y omega=0
          movimiento_linea_recta((cmd_PWMIzq(cmd_omegaIzq(velEstado1070, 0))), (cmd_PWMDer(cmd_omegaDer(velEstado1070, 0))));
          incTiempoEstado1070 = millis() - tiempoEstado1070;
          float vel_inst1070 = (-(float)valores[1] + (float)distanciaEstado1070) / (float)incTiempoEstado1070;
          Serial.println("oooccc" + String(estado) + "ww" + String(valores[1])  + "ww" + String(incTiempoEstado1070)  + "ww" + String(vel_inst1070)  + "ww"  + String( velEstado1070 / (vel_inst1070 * parameters[0]) ) + "ww###");

        } else {
          pararMotores();
          incTiempoEstado1070 = millis() - tiempoEstado1070;
          float vel_inst1070 = (-(float)valores[1] + (float)distanciaEstado1070) / (float)incTiempoEstado1070;
          Serial.println("oooccc" + String(estado) + "ww" + String(valores[1])  + "ww" + String(incTiempoEstado1070)  + "ww" + String(vel_inst1070)  + "ww"  + String( velEstado1070 / (vel_inst1070 * parameters[0]) ) + "ww###");
          estado = 0;
        }


}

void Test07iiittt()
{
	      //estado que realiza un control PID en el ángulo, y se mueve hasta una determinada distancia
        estado = 1070;
        velEstado1070 = parameters[0] * trozo2Str.toFloat();

        tiempoEstado1070 = millis();
        distanciaEstado1070 = valores[1];

        if (cmdVel != cmdVelAnterior) {
          azimutRef = azimut;
        }

        inicia_linea_recta();
	
}

void Test07Loop()
{
	
	      // Intentando andar en línea recta, con control PID en el ángulo

      if (valores[1] > distanciaEstado1070 - DISTANCIA_CALIBRACION) {

      }

      else {
        pararMotores();
        incTiempoEstado1070 = millis() - tiempoEstado1070;
        float vel_inst1070 = (-(float)valores[1] + (float)distanciaEstado1070) / (float)incTiempoEstado1070;
        Serial.println("oooccc" + String(estado) + "ww" + String(valores[1])  + "ww" + String(incTiempoEstado1070)  + "ww" + String(vel_inst1070)  + "ww"  + String( velEstado1070 / (vel_inst1070 * parameters[0])) + "ww###");


        estado = 0;


      }

}
