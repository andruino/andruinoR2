#include "defines.h"

#include <Arduino.h>

extern int estado;

extern int estadoPID;
extern int salPID;

extern float errorAzimut;
extern float errorAzimutD;
extern float errorAzimutI; 

extern float azimut_odometry_iiikkk;
extern float incAzimut;

extern float distancia;

extern float x;
extern float y;
extern float azimut;
extern float azimutRef;
extern float cmdVel;
extern float cmdVelAnterior;

extern float secuencia;

extern float parameters[21];


//Variables Estado 1060, pid en angulo
static unsigned long  tiempoEstado1060 = 0;


void pararMotores();
int inicia_linea_recta();
int movimiento_linea_recta(int PWM_I, int PWM_D);
float cmd_omegaDer(float cmd_vel, float cmd_ang);
float cmd_omegaIzq(float cmd_vel, float cmd_ang);
int cmd_PWMDer(float omegaDer);
int cmd_PWMIzq(float omegaIzq);




      // Estado 1060, PID sobre el angulo con TYPE_GAME_ORIENTATION_VECTOR, se debe mover en línea recta durante unos sg


void Test06iiiaaa()
{
	  
        
     
        if ((tiempoEstado1060 + 6000) > millis()) {
          //Velocidad fija para 0.15 m/Sg
          cmdVel = 0.1 * parameters[0];
          //Valores a la mitad del rango PWM posible de 180 a 255. Haciendo Wd=-Wd para que camine recto y usando las ecuaciones
          movimiento_linea_recta((cmd_PWMIzq(cmd_omegaIzq(cmdVel, 0))), (cmd_PWMDer(cmd_omegaDer(cmdVel, 0))));
          // lineales que relacionan el w con el PWM, sale que para que ande recto debe ser PWM_I=225 y PWM_D=215
          azimut_odometry_iiikkk += incAzimut;
          secuencia++;
          Serial.print("oooccc");
          Serial.print(estado, 4);
          Serial.print("ww");
          Serial.print(azimut, 4);
          Serial.print ("ww"  + String(millis() - tiempoEstado1060) + "ww");
          Serial.print (azimutRef, 4);
          Serial.print ("ww" + String(estadoPID) +  "ww" + String(salPID) + "ww" + String(errorAzimut) +  "ww" + String(errorAzimutI) + "ww" + String(errorAzimutD) + "ww"); //En radianes
          Serial.print(incAzimut, 4);
          Serial.print("ww");
          Serial.print(azimut_odometry_iiikkk, 4);
          Serial.print("ww");
          Serial.print(distancia, 4);
          Serial.print("ww");
          Serial.print(distancia * cos(azimut_odometry_iiikkk / 10000L), 4);
          Serial.print("ww");
          Serial.print(distancia * sin(azimut_odometry_iiikkk / 10000L), 4);
          Serial.print("ww");
          Serial.print(x, 4);
          Serial.print("ww");
          Serial.print(y, 4);
          Serial.print("ww");
          Serial.print(secuencia, 4);
          Serial.print("ww###");
          Serial.print('\n');

        } else {

          estado = 0;
          pararMotores();
          cmdVel = 0.0;
          secuencia = 0.0;
        }

}


void Test06iiittt()
{
	        //estado que realiza un control PID en el ángulo, moviendose en línea recta
        estado = 1060;
        azimut_odometry_iiikkk = 0.0;
        distancia = 0;
        x = 0.0;
        y = 0.0;

        tiempoEstado1060 = millis();
        if (cmdVel != cmdVelAnterior) {
          azimutRef = azimut;
        }

        inicia_linea_recta();

	
	
}
