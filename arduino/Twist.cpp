#include <Arduino.h>
#include "defines.h"
#include "CtrlMotores.h"

extern int estado;
extern float parameters[21];

extern float cmdVel;
extern float cmdVelAnterior;
extern float cmdOmega;
extern float cmdAzimut;

extern float azimut;
extern float x;
extern float y;
extern float incAzimut;
extern float azimut_odometry;
extern float azimut_odometry_iiikkk; // azimut_odometry ;
extern float azimutRef;

extern float distancia;

extern float ICCR;

extern String inputString,trozo1Str, trozo2Str, trozo3Str;


void Twistiiiaaa()
{
	
	 // Estado 2010, anda recto tras recibir una orden
      if (estado == 2010) {
        azimut_odometry_iiikkk = azimut_odometry_iiikkk + incAzimut;
        movimiento_linea_recta((cmd_PWMIzq(cmd_omegaIzq(parameters[0] * cmdVel, 0))), (cmd_PWMDer(cmd_omegaDer(parameters[0] * cmdVel, 0))));
        //azimut_odometry += 0; //Supone que hace la línea recta y no se desvía ??? podria ser ErrorAzimut*Cos(incAzimut) PENSAR??
        //azimut_odometry = azimut_odometry_iiikkk + errorAzimut; //Considera la desviación del ángulo usada en el PID

      }
      // Fin Estado 2010

      // Estado 2020, giro puro
      if (estado == 2020) {
        azimut_odometry += incAzimut;
        //azimut_odometry =  atan2(sin(azimut_odometry), cos(azimut_odometry)); //Limita a -pi pi . HAcer Funcion !!!
      }
      // Fin Estado 2010


      // Estado 2000, anda haciendo un arco
      if (estado == 2000) {
        //if (incAzimut != azimut) { //Realiza un arco de circunferencia, vel!=0 y omega!=0

          //Considerando que incAzimut es pequeña estimo la velocidad lineal
          //vel_estimada = 1000 * (ICCR * incAzimut) / dtAzimut;

          movimiento_arco(cmd_PWMIzq(cmd_omegaIzq(parameters[0] * cmdVel, parameters[1] * cmdOmega)), cmd_PWMDer(cmd_omegaDer(parameters[0] * cmdVel, parameters[1] * cmdOmega)));

          // Calculo de x y para odometria
          //x +=  ICCR * (sin(azimut_odometry + incAzimut) - sin(azimut_odometry));   COMENTADO POR ALEJANDRO
          //y += ICCR * (cos(azimut_odometry) - cos(azimut_odometry + incAzimut));

          // Actualizacion de azimut para odometria
          azimut_odometry += incAzimut;
          //azimut_odometry =  atan2(sin(azimut_odometry), cos(azimut_odometry)); //Limita a -pi pi . HAcer Funcion !!!

        //}
      }
      // Fin Estado 2000
	
}


void Twistiiikkk()
{
	      //Guarda la velocidad anterior, para saber si tiene que aplicar una nueva referencia de azimut o no
      cmdVelAnterior = cmdVel;

      trozo1Str = inputString.substring(inputString.indexOf("kkk") + 3, inputString.indexOf("ww"));
      trozo2Str = inputString.substring(inputString.indexOf("ww") + 2, inputString.indexOf("vv"));
      trozo3Str = inputString.substring(inputString.indexOf("vv") + 2, inputString.indexOf("ww###"));

      cmdVel = trozo1Str.toFloat();
      cmdOmega = trozo2Str.toFloat();
      cmdAzimut = trozo3Str.toFloat() / 10000.0;

      azimut_odometry_iiikkk = 00.0; // azimut_odometry ;
      incAzimut=0.0;
      distancia = 0;


      if (cmdVel == 0 && cmdOmega == 0) {
        pararMotores();
        estado = 0;
        cmdVelAnterior = 0;
      }
      else if (cmdVel != 0 && cmdOmega == 0) {
        //Anda en línea recta
        estado = 2010;
        if (cmdVel != cmdVelAnterior) {
          azimutRef = cmdAzimut;
          pararMotores();
          //Serial.println("AzimutRef:" +  String(azimutRef) + " " + trozo3Str);
        }
        inicia_linea_recta();

      }
      //Gira en el sitio
      else if (cmdVel == 0 && cmdOmega != 0) {
        estado = 2020;

        motoresPWM(cmd_PWMIzq(cmd_omegaIzq(0.0 , parameters[1] * cmdOmega)), cmd_PWMDer(cmd_omegaDer(0.0 , parameters[1] * cmdOmega)));
        cmdVelAnterior = 0;

      }

      //Anda haciendo un arco de circunferencia
      else if (cmdVel != 0 && cmdOmega != 0) {
        estado = 2000;

        //Calcula el radio de curvatura ICC,para estima x,y
        // de www.doc.ic.ac.uk/~ajd/Robotics Lecture 2
        // R=W(vr+vl)/2(vr-vl)
        ICCR = 0.5 * (parameters[3] / parameters[4]) * DISTANCIA_RUEDAS * ( cmd_omegaDer(parameters[0] * cmdVel, parameters[1] * cmdOmega) + cmd_omegaIzq(parameters[0] * cmdVel, parameters[1] * cmdOmega) ) / ( cmd_omegaDer(parameters[0] * cmdVel, parameters[1] * cmdOmega) - cmd_omegaIzq(parameters[0] * cmdVel, parameters[1] * cmdOmega) );
        //ICCR = 0*5* DISTANCIA_RUEDAS * ( cmd_omegaDer(parameters[0] * cmdVel, 3 * parameters[1] * cmdOmega) + cmd_omegaIzq(parameters[0] * cmdVel, 3* parameters[1] * cmdOmega) ) / ( cmd_omegaDer(parameters[0] * cmdVel, 3 * parameters[1] * cmdOmega) - cmd_omegaIzq(parameters[0] * cmdVel, 3 * parameters[1] * cmdOmega) );

        inicia_arco(); //030116


      }
	
	
}
