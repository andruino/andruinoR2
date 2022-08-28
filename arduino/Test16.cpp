#include <Arduino.h>
#include "defines.h"
#include "CtrlMotores.h"

extern int estado;

extern String inputString, trozo1Str;

//Valores de los sensores, LDRdiff y LDRmedia
extern int LDRIzq; //sustituir por valores (3,4,5) !!!!
extern int LDRCen;
extern int LDRDer;
extern int LDRdiff; //diferencia antes del ensayo

extern int LDRdiffpost; // diferencia después del ensayo
extern int LDRmedia; //media antes del ensayo
extern int LDRmediapost; // media despues del ensayo
extern int LDRIzqpost;
extern int LDRDerpost;
extern int LDRCenpost;


extern int LDROffset;


extern int USIzq; //sustituir por valores (3,4,5) !!!!
extern int USCen;
extern int USDer;
extern int USdiff; //diferencia antes del ensayo
//int LDRsigno = 0;
extern int USdiffpost; // diferencia después del ensayo
extern int USmedia; //media antes del ensayo
extern int USmediapost; // media despues del ensayo
extern int USIzqpost;
extern int USDerpost;
extern int USCenpost;
extern int USOffset;

extern int auxX; //sutituir por i


void pararMotores();

int leeUS(int echoPin, int trigPin);



void Test16iiiqqq()
{
	if (estado == 1160) {
        pararMotores();
        //delay(50);

        //Tomo valores actuales de sensores
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


        //Se mueve
        trozo1Str = inputString.substring(inputString.indexOf("qqq") + 3, inputString.indexOf("ww###"));
        /* Movimientos
         * 0 para
         * 1 gira derecha (dextrogiro)
         * 2 gira izquierda
         * 3 avanza
         * 4 retrocede
         * 5 derecha_para, izquierda_adelante
         * 6 derecha_para, izquierda_atras
         * 7 izq_para, derecha_adelante
         * 8 izq_para, derecha_atras
         */

        switch (trozo1Str.toInt()) {
          case 0:
            pararMotores();
            break;
          case 1:
	        motorIzqPWM (0,250);  
            motorDchPWM (250,0); 
            break;
          case 2:
	        motorIzqPWM (250,0);  
            motorDchPWM (0,250); 
            break;
          case 3:
	        motorIzqPWM (250,0);  
            motorDchPWM (250,0); 
            break;
          case 4:
	        motorIzqPWM (0,250);  
            motorDchPWM (0,250); 
            break;
          case 5:
	        motorIzqPWM (250,0);  
            motorDchPWM (0,0); 
            break;
          case 6:
	        motorIzqPWM (0,250);  
            motorDchPWM (0,0); 
            break;
          case 7:
	        motorIzqPWM (0,0);  
            motorDchPWM (250,0); 
            break;
          case 8:
	        motorIzqPWM (0,0);  
            motorDchPWM (0,250); 
            break;
          default:
            pararMotores();
            estado = 0; //Si recibe una accion no esperado sale del modo vsa 1160
        }
        //Se mueve durante el tiempo indicado
        delay(200);

        //Se detiene
        pararMotores();
        auxX = 0;

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

        //Envia los dato al nodo VSA
        Serial.print("oooccc" + String(estado) + "ww" + String(LDRIzq) + "ww" + String(LDRCen) + "ww" + String(LDRDer) + "ww" +   String(LDRIzqpost) + "ww" + String(LDRCenpost) + "ww" + String(LDRDerpost) + "ww");
        Serial.print(String(USIzq) + "ww" + String(USCen) + "ww" + String(USDer) + "ww" +   String(USIzqpost) + "ww" + String(USCenpost) + "ww" + String(USDerpost) + "ww");
        Serial.print ( trozo1Str +  "ww" + "###");
        Serial.println("");

      }


}


void Test16iiittt()
{
 //Test16: Se orienta hacia la luz sin chocar sin conocimiento previo, usando arquitectura VSA
        estado = 1160;
}
