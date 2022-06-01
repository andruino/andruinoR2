#include "defines.h"
#include <Arduino.h>

extern int estado;


extern int auxX;

void Test15iiittt()
{
      //Test15: Se orienta hacia la luz sin chocar sin conocimiento previo, usando iot y Machine Learning
        estado = 1150;
        auxX = 0;
}

void Test15Loop()
{
	
	     /*
        //Lee los valores de las LDR y calcula sus diferencias
        LDROffset = 0;
        LDRIzq = analogRead(analogInputIzq);
        LDRDer = analogRead(analogInputDer);
        LDRCen = analogRead(analogInputCen);
        LDRdiff = LDRIzq - LDRDer + LDROffset;
        LDRmedia = (LDRIzq + LDRDer) / 2;

        USOffset = 0;
        USDer = leeUS(echoPin1, trigPin1);
        if (USDer == 0)
          USDer = 3000;
        USCen = leeUS(echoPin2, trigPin2);
        if (USCen == 0)
          USCen = 3000;
        USIzq = leeUS(echoPin3, trigPin3);
        if (USIzq == 0)
          USIzq = 3000;

        USdiff = USIzq - USDer + USOffset;
        USmedia = (USDer + USCen + USIzq) / 3;



        //Genera Movimientos aleatorios de los motores
        moveMI = random(0, 3) - 1;
        moveMD = random(0, 3) - 1;

        //Se mueve
        if (moveMI < 2) {
          if (moveMI == 0) {
            analogWrite(motorIAvance, 0);
            analogWrite(motorIRetroceso, 0);
          }
          if (moveMD == 0) {
            analogWrite(motorDAvance, 0);
            analogWrite(motorDRetroceso, 0);
          }
          if (moveMI == -1) {
            analogWrite(motorIAvance, 0);
            analogWrite(motorIRetroceso, 250);
          }
          if (moveMD == -1) {
            analogWrite(motorDAvance, 0);
            analogWrite(motorDRetroceso, 250);
          }
          if (moveMI == 1) {
            analogWrite(motorIAvance, 250);
            analogWrite(motorIRetroceso, 0);
          }
          if (moveMD == 1) {
            analogWrite(motorDAvance, 250);
            analogWrite(motorDRetroceso, 0);
          }
          //Se mueve durante el tiempo indicado
          delay(200);
        } else if (moveMI == 2 && moveMD == 2) {
          analogWrite(motorDAvance, 250);
          analogWrite(motorIAvance, 250);
          analogWrite(motorDRetroceso, 0);
          analogWrite(motorIRetroceso, 0);


          delay(900); //Cambiar por comparacion con timer para que no pare la recepcion de órdenes!!!!
        }

        //Se detiene
        analogWrite(motorIAvance, 0);
        analogWrite(motorIRetroceso, 0);
        analogWrite(motorDAvance, 0);
        analogWrite(motorDRetroceso, 0);
        auxX = 0;
        delay(200); //Cambiar por comparacion con timer para que no pare la recepcion de órdenes!!!!

        // Verifica resultado de movimiento realizado.
        LDRIzqpost = analogRead(analogInputIzq);
        LDRDerpost = analogRead(analogInputDer);
        LDRdiffpost = LDRIzqpost - LDRDerpost + LDROffset;
        LDRmediapost = (LDRIzqpost + LDRDerpost) / 2;

        USDerpost = leeUS(echoPin1, trigPin1);
        if (USDerpost == 0 || USDer == 3000) //Si alguna de las dos medidas no son válidas, hace los dos valores 3000, para que no afecte al calculo de la media de distancia
          USDerpost = 3000;
        USCenpost = leeUS(echoPin2, trigPin2);
        if (USCenpost == 0 || USCen == 3000)
          USCenpost = 3000;
        USIzqpost = leeUS(echoPin3, trigPin3);
        if (USIzqpost == 0 || USIzq == 3000)
          USIzqpost = 3000;
        USdiffpost = USIzqpost - USDerpost + USOffset;
        USmediapost = (USDerpost + USCenpost + USIzqpost) / 3;

        //Juicio de valor (policy) 1. Mejoro luz. 2 Mejoró US. 3 Mejoró Luz + US
        //if ((abs(LDRdiffpost) - abs(LDRdiff)) < 0 && abs(LDRdiffpost - LDRdiff) > 2 && (LDRmediapost < LDRmedia) ) {
        if ((abs(LDRdiffpost) - abs(LDRdiff)) < 0  && (LDRmediapost < LDRmedia)  && abs(LDRdiffpost - LDRdiff) > 10 ) {
          auxX = 1;
        } else {
          auxX = 0;

        }

        if ((abs(USdiffpost) - abs(USdiff)) > 0  && (USmediapost > USmedia) && abs(USdiffpost - USdiff) > 3 && abs(USCenpost - USCen) > 3) {
          //if ( (USmediapost > USmedia) && abs(USdiffpost - USdiff) > 10) {
          //auxX += 5;
          //auxY += 5;
          auxX += 2;
        } else {
          //auxX = 0;

        }
        // if (auxX>0) {
        Serial.print("oooccc" + String(LDRIzq) + "ww" + String(LDRCen) + "ww" + String(LDRDer) + "ww" +   String(LDRIzqpost) + "ww" + String(LDRCenpost) + "ww" + String(LDRDerpost) + "ww");
        Serial.print(String(USIzq) + "ww" + String(USCen) + "ww" + String(USDer) + "ww" +   String(USIzqpost) + "ww" + String(USCenpost) + "ww" + String(USDerpost) + "ww");
        Serial.print (String(auxX) + "ww" + String(moveMI) + "ww" +  String(moveMD) + "ww" + "###");
        Serial.println("");
        // }

      */
      //Fin de estado 1150, entrenamiento para machine learning
	
	
}
