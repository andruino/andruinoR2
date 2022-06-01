#include <Arduino.h>
#include "defines.h"
#include "CtrlMotores.h"


extern float parameters[21];
extern int valores[10];
extern float azimut;

extern float incAzimut;
extern float dtAzimut;
extern float cmdOmega;

float outPID = 0;


//PID
float errorAzimut = 0; //error P
float errorAzimutD = 0;  //error D
float errorAzimutI = 0; //error I
float errorAzimutAnterior = 0;
unsigned long tiempoAzimutAnterior;
int estadoPID = -1;
int salPID;

float azimutRef = 0;                    //referencia del azimut para pid
float azimutAnterior = 0; //Usado para el PIS

//PD
int outPD;

//PID_arco
float omega_estimada;
float errorOmega;
int  outPID_arco;


//////////////////////////////////////////////////////////////////////////
// Variables para cálculo de la distancia en tiempo de ejecución en movimiento
// en línea recta, basada en sensor central de ultrasonidos
//////////////////////////////////////////////////////////////////////////
float distanciaUS = -1.0;
float distanciaUS_inicial = -1.0;
float distanciaUS_anterior = -1.0;


void motorDchPWM(int PWMAvance, int PWMRetroceso)
{
  analogWrite(motorDAvance, PWMAvance);
  analogWrite(motorDRetroceso, PWMRetroceso);	
}

void motorIzqPWM(int PWMAvance, int PWMRetroceso)
{
  analogWrite(motorIAvance, PWMAvance);
  analogWrite(motorIRetroceso, PWMRetroceso);	
}

void pararMotores() 
{
  // Para motores
  analogWrite(motorIAvance, 0);
  analogWrite(motorIRetroceso, 0);
  analogWrite(motorDAvance, 0);
  analogWrite(motorDRetroceso, 0);
}

int motoresPWM(int motor_izquierdo, int motor_derecho) {

  int saturacion = 0;

  // saturacion, envía la suma de
  // = 0 si no está saturado
  // = 1, si motor derecho esta por debajo de ((int)parameters[19])
  // = 2, si motor derecho está por enciam de ((int)parameters[18])
  // = 10, si motor izquierdo esta por debajo de ((int)parameters[19])
  // = 20, si motor izquierdo está por enciam de ((int)parameters[18])

  if (abs(motor_derecho) > ((int)parameters[18])) {
    motor_derecho = ((int)parameters[18]) * (motor_derecho / abs(motor_derecho));
    saturacion += 2;
  } else if (abs(motor_derecho) < ((int)parameters[19])) {
    //motor_derecho = 0 ;
    saturacion += 1;
  }

  if (abs(motor_izquierdo) > ((int)parameters[18])) {
    motor_izquierdo = ((int)parameters[18]) * (motor_izquierdo / abs(motor_izquierdo));
    saturacion += 20;
  } else if (abs(motor_izquierdo) < ((int)parameters[19])) {
    //motor_izquierdo = 0 ;
    saturacion += 10;
  }

  // Arranque
  /*
  if (motor_derecho > 0) {
    analogWrite(motorDAvance, 250);
    analogWrite(motorDRetroceso, 0);
  } else {
    analogWrite(motorDAvance, 0);
    analogWrite(motorDRetroceso, 250);
  }

  if (motor_izquierdo > 0) {
    analogWrite(motorIAvance, 250);
    analogWrite(motorIRetroceso, 0);
  } else {
    analogWrite(motorIAvance, 0);
    analogWrite(motorIRetroceso, 250);
  }
  */

  // Sentido de giro
  if (motor_derecho > 0) {
    analogWrite(motorDAvance, abs(motor_derecho));
    analogWrite(motorDRetroceso, 0);
  } else {
    analogWrite(motorDAvance, 0);
    analogWrite(motorDRetroceso, abs(motor_derecho));
  }

  if (motor_izquierdo > 0) {
    analogWrite(motorIAvance, abs(motor_izquierdo));
    analogWrite(motorIRetroceso, 0);
  } else {
    analogWrite(motorIAvance, 0);
    analogWrite(motorIRetroceso, abs(motor_izquierdo));
  }

  if (saturacion != 0 && estadoPID == 0) //Si el PID esta fuuncionando (estadoPID=0) y existe saturacion (saturacion !=0), entonces pasa el estadoPID a 1 (indicando que está saturado)
  {
    estadoPID = 1;
  }
  else {
    if (saturacion == 0 && estadoPID == 1) // Y Si deja de estar saturado deja de estar saturado
    {
      estadoPID = 0;
    }
  }

  return saturacion;
}



//Cinemática inversa uniciclo (PENDIENTE VALIDAR !!!!!!!!!!!!!!!!!!!!!. ESPECIALMENTE EL SIGNO DE cmd_ang!!!)
float cmd_omegaDer(float cmd_vel, float cmd_ang) {

  return (cmd_vel  + (parameters[3] * DISTANCIA_RUEDAS * cmd_ang / 2)) / (parameters[2] * RADIO_RUEDA);

}
float cmd_omegaIzq(float cmd_vel, float cmd_ang) {

  return (cmd_vel - (parameters[3] * DISTANCIA_RUEDAS * cmd_ang / 2)) / (parameters[2] * RADIO_RUEDA);

}


//Relación omea / PWM (PEnDIENTE DE CAMBIAR VALORES FIJOS POR CALIBRACION !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!)
int cmd_PWMDer(float omegaDer) {

  return (int)((parameters[7] * abs(omegaDer)) + parameters[8]) * (omegaDer / abs(omegaDer));
  //return (int)((101.5 * abs(omegaDer)) + 103.41) * (omegaDer / abs(omegaDer));
}

int cmd_PWMIzq(float omegaIzq) {

  return (int)((parameters[5] * abs(omegaIzq)) + parameters[6]) * (omegaIzq / abs(omegaIzq));
  //return (int)((111.5 * abs(omegaIzq)) + 112.41) * (omegaIzq / abs(omegaIzq));
}

int movimiento_arco(int PWM_I, int PWM_D) {

  salPID = pid_arco();

  PWM_I = PWM_I - salPID;
  PWM_D = PWM_D + salPID;

  motoresPWM(PWM_I, PWM_D);
}

int movimiento_linea_recta(int PWM_I, int PWM_D) {

  //int PWM_I = 225;
  //int PWM_D = 215;

  salPID = pid(); //Ejecuta el PID

  PWM_I = PWM_I + salPID;
  PWM_D = PWM_D - salPID;

  //SATURACION.OJO PARAR INTEGRAL del  PID!!!!!!!!!!!!!!!!!
  motoresPWM(PWM_I, PWM_D);


}

int inicia_arco() {

  //pararMotores();//Da problemas en teleoperacion

  estadoPID = -1;

  outPID_arco = 0;

  return 0;

}

int inicia_linea_recta() {

  //pararMotores(); //Da problemas en teleoperacion

  //inicia PID
  estadoPID = -1;

  //Para cálculo de distancia del movimiento
  distanciaUS_inicial = valores[1];
  distanciaUS_anterior = -1;

  outPID = 0;


  azimutAnterior = azimut;


  //Nueva confiuración de PID Azimut
  errorAzimut = 0;  //error P
  errorAzimutD = 0;  //error D
  errorAzimutI = 0;   //error I
  errorAzimutAnterior = 0;
  tiempoAzimutAnterior = millis();
  outPID = 0;

}

int movimiento_giro() {

  int PWM_I;
  int PWM_D;

  outPD = pd(); //Sólo PD, EN REALIDAD PID GIRO

  PWM_I = outPD + (((int)parameters[19]) * (outPD / abs(outPD)));
  PWM_D =  - outPD + (-1) * (((int)parameters[19]) * (outPD / abs(outPD)));

  //SATURACION.OJO PARAR INTEGRAL del  PID!!!!!!!!!!!!!!!!!
  motoresPWM(PWM_I, PWM_D);



}

int inicia_giro(float azimutGira) {

  estadoPID == -1; // Sólo PD para guirar
  outPD = 0;

  //Falta controlar que azimutGira esté entre -pi y pi

  azimutRef = azimutGira; // Establece la referencia del azimut

  outPID = 0;

  azimutAnterior = azimut;

  //Arranque de motores
  /*
  if (azimutRef >= 0) {
    //Arranque de motores
    analogWrite(motorIAvance, 250); //Valores calculados a mano OJO!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    analogWrite(motorIRetroceso, 0);
    analogWrite(motorDAvance, 0);
    analogWrite(motorDRetroceso, 240);
  } else {
    analogWrite(motorIAvance, 0); //Valores calculados a mano OJO!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    analogWrite(motorIRetroceso, 250);
    analogWrite(motorDAvance, 240);
    analogWrite(motorDRetroceso, 0);
  }
  */
}


//PD    EN REALIDAD PID_GIRO
int pd() {


  //P
  errorAzimut = azimutRef - azimut; //Redefine el error
  errorAzimut = (float)  atan2((double)sin(errorAzimut), (double)cos(errorAzimut));
  //outPID =  kpd * (errorAzimut);

  //I
  if (estadoPID == -1) {
    //La primera vez que se ejecuta el PID
    errorAzimutI = 0;
  } else {
    if (estadoPID == 0) {
      //Si no está saturado. estadoPID==1 si alguno de los actuarores está en el máximo (o mínimo)
      errorAzimutI += (errorAzimutAnterior * ((float)(millis() - tiempoAzimutAnterior))) ;
    }
  }


  //D
  // El estadoPID indica si se inició (-1) o si está saturado (1)
  if (estadoPID == -1) {
    estadoPID = 0; // Tras la primera vez que se ejecuta pasa a estado 0, para comenzar a calcular el error D
  } else {
    //if ((millis() - tiempoAzimutAnterior) != 0 && errorAzimut>=0.2)
    //if ((millis() - tiempoAzimutAnterior) != 0 && abs(errorAzimut) >= 0.05)
    if ((millis() - tiempoAzimutAnterior) != 0)
      errorAzimutD =  (1000 * (errorAzimut - errorAzimutAnterior)) / (millis() - tiempoAzimutAnterior);
    else
      errorAzimutD = 0;
  }
  
  outPID =  ((int)parameters[12]) * (errorAzimut) + (((int)parameters[13]) * (errorAzimutI / 1000)) + (((int)parameters[14]) * errorAzimutD);


  errorAzimutAnterior = errorAzimut;
  tiempoAzimutAnterior = millis();


  azimutAnterior = azimut;


  return (int)((outPID));

}


//PID_arco
int pid_arco() {

  //
  omega_estimada = 1000 * incAzimut / dtAzimut;
  errorOmega = cmdOmega - omega_estimada;

  //P
  if (estadoPID == -1) {
    outPID_arco = 0;
    estadoPID = 1;
  } else {
    //outPID_arco =  ((int)parameters[15])o * errorOmega;
    outPID_arco =  ((int)parameters[15]) * errorOmega;
  }

  //I

  //D

  return (int)((outPID_arco));

}

//PID

int pid() {


  //P
  errorAzimut = azimutRef - azimut; //Redefine el error
  errorAzimut = (float)  atan2((double)sin(errorAzimut), (double)cos(errorAzimut));

  //I
  if (estadoPID == -1) {
    //La primera vez que se ejecuta el PID
    errorAzimutI = 0;
  } else {
    if (estadoPID == 0) {
      //Si no está saturado. estadoPID==1 si alguno de los actuarores está en el máximo (o mínimo)
      errorAzimutI += (errorAzimutAnterior * ((float)(millis() - tiempoAzimutAnterior))) ;
    }
  }


  //D
  // El estadoPID indica si se inició (-1) o si está saturado (1)
  if (estadoPID == -1) {
    estadoPID = 0; // Tras la primera vez que se ejecuta pasa a estado 0, para comenzar a calcular el error D
  } else {
    //if ((millis() - tiempoAzimutAnterior) != 0 && errorAzimut>=0.2)
    //if ((millis() - tiempoAzimutAnterior) != 0 && abs(errorAzimut) >= 0.05)
    if ((millis() - tiempoAzimutAnterior) != 0)
      errorAzimutD =  (1000 * (errorAzimut - errorAzimutAnterior)) / (millis() - tiempoAzimutAnterior);
    else
      errorAzimutD = 0;
  }


  //outPID =  (((int)parameters[9]) * errorAzimut) + (((int)parameters[10]) * errorAzimutI) + (((int)parameters[11])/10) * errorAzimutD ;



  //outPID =  ((int)parameters[9]) * (errorAzimut + (2 * errorAzimutI) + (errorAzimutD)/4);
  outPID =  (((int)parameters[9]) * (errorAzimut)) + (((int)parameters[10]) * (errorAzimutI / 1000)) + (((int)parameters[11]) * errorAzimutD);
  //Actualización de valores
  errorAzimutAnterior = errorAzimut;
  tiempoAzimutAnterior = millis();

  azimutAnterior = azimut;

  return (int)((outPID));

}
