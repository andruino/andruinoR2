#include "defines.h"
#include <Arduino.h>

extern int estado;

// VAriables globales de Test3
int value = 0;
//Movimientos del motor izquierdo y derecho
int moveMI = 0;
int moveMD = 0;
//Valores de los sensores, LDRdiff y LDRmedia
int LDRIzq = 0; 
int LDRCen = 0;
int LDRDer = 0;
int LDRdiff = 0; //diferencia antes del ensayo
int LDRdiffpost = 0; // diferencia después del ensayo
int LDRmedia = 0; //media antes del ensayo
int LDRmediapost = 0; // media despues del ensayo
int LDRIzqpost = 0;
int LDRDerpost = 0;
int LDRCenpost = 0;

int LDROffset = 0;

int USIzq = 0; 
int USCen = 0;
int USDer = 0;
int USdiff = 0; //diferencia antes del ensayo
int USdiffpost = 0; // diferencia después del ensayo
int USmedia = 0; //media antes del ensayo
int USmediapost = 0; // media despues del ensayo
int USIzqpost = 0;
int USDerpost = 0;
int USCenpost = 0;
int USOffset = 0;



int fila; // Fila de la matriz

int BCMD[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0}; 
int BCMI[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

int auxX; 
int auxY; 
int i;
int j;
//Fin de  Test 3



      //Test3: Se orienta hacia la luz sin conocimiento previo

void Test03iiittt()
{
	 estado = 1030;
}


void Test03Loop()
{
      /*
      //Lee los valores de las LDR y calcula sus diferencias
      LDRIzq = analogRead(analogInputIzq);
      LDRDer = analogRead(analogInputDer);
      LDRCen = analogRead(analogInputCen);
      LDRdiff = LDRIzq - LDRDer + LDROffset;
      LDRmedia = (LDRIzq + LDRDer) / 2;

      // Envía señales para depuración

      Serial.println("LDRIzq");
      Serial.print(LDRIzq, DEC);
      Serial.println(" ");
      Serial.println("LDRDer");
      Serial.print(LDRDer, DEC);
      Serial.println(" ");


            if (LDRdiff > 0 && (abs(LDRdiff) > 30)) {
              // Diferencia Positiva, mas luz en LDR derecha
              fila = 0;
              LDRsigno = 1;
              //MOTOR DERECHO
              if ((BCMD[0] > BCMD[1]) && (BCMD[0] > BCMD[2])) {
                moveMD = -1;
              } else if ((BCMD[1] > BCMD[0]) && (BCMD[1] > BCMD[2])) {
                moveMD = 0;
              } else if ((BCMD[2] > BCMD[1]) && (BCMD[2] > BCMD[0])) {
                moveMD = 1;
              }
              else {
                moveMD = random(0, 3) - 1;
              }
              //MOTOR IZQUIERDO
              if ((BCMI[0] > BCMI[1]) && (BCMI[0] > BCMI[1])) {
                moveMI = -1;
              } else if ((BCMI[1] > BCMI[0]) && (BCMI[1] > BCMI[2])) {
                moveMI = 0;
              } else if ((BCMI[2] > BCMI[1]) && (BCMI[2] > BCMI[0])) {
                moveMI = 1;
              } else {
                moveMI = random(0, 3) - 1;
              }
            }

            if (LDRdiff < 0 && (abs(LDRdiff) > 30)) {
              //Más luz en la LDR Izquierda
              fila = 6;
              LDRsigno = -1;
              //MOTOR DERECHO
              if ((BCMD[6] > BCMD[7]) && (BCMD[6] > BCMD[8])) {
                moveMD = -1;
              } else if ((BCMD[7] > BCMD[6]) && (BCMD[7] > BCMD[8])) {
                moveMD = 0;
              } else if ((BCMD[8] > BCMD[6]) && (BCMD[8] > BCMD[7])) {
                moveMD = 1;
              } else {
                moveMD = random(0, 3) - 1;
              }
              //MOTOR IZQUIERDO
              if ((BCMI[6] > BCMI[7]) && (BCMI[6] > BCMI[7])) {
                moveMI = -1;
              } else if ((BCMI[7] > BCMI[6]) && (BCMI[7] > BCMI[8])) {
                moveMI = 0;
              } else if ((BCMI[8] > BCMI[7]) && (BCMI[8] > BCMI[6])) {
                moveMI = 1;
              } else {
                moveMI = random(0, 3) - 1;
              }
            }

            if (abs(LDRdiff) <= 30 and abs(LDRmedia) > 500) {

              LDRsigno = 0;
              fila = 3;

              moveMI = random(0, 3) - 1;

              moveMD = random(0, 3) - 1;

            } else if (
      ) {

              //Si está "orientado" a la luz para
              LDRsigno = 0;
              fila = 3;
              moveMI = 0;
              moveMD = 0;
            }

            if (moveMI < 2) {
              if (moveMI == 0) {
       			motorIzqPWM (0,0);
              }
              if (moveMD == 0) {
                motorDchPWM (0,0);
              }
              if (moveMI == -1) {
				motorIzqPWM (0,250);
              }
              if (moveMD == -1) {
                motorDchPWM (0,250);
     			}
              if (moveMI == 1) {
                motorIzqPWM (250,0);  
              }
              if (moveMD == 1) {
                motorDchPWM (250,0);
				}
              //Se mueve durante el tiempo indicado
              delay(200);
            } else if (moveMI == 2 && moveMD == 2) {
              motorIzqPWM (250,0);
              motorDchPWM (250,0);
              delay(600);
            }



            //Se detiene
            pararMotores();

            // Verifica resultado de movimiento realizado.
            LDRIzq = analogRead(analogInputIzq);
            LDRDer = analogRead(analogInputDer);
            LDRdiffpost = LDRIzq - LDRDer + LDROffset;
            LDRmediapost = (LDRIzq + LDRDer) / 2;


            Serial.println("LDRIzqpost");
            Serial.print(LDRIzq, DEC);
            Serial.println(" ");
            Serial.println("LDRDerpost");
            Serial.print(LDRDer, DEC);
            Serial.println(" ");

            Serial.println("LDRdiffpost");
            Serial.print(LDRdiffpost, DEC);
            Serial.println(" ");
            Serial.println("LDRdiff");
            Serial.print(LDRdiff, DEC);
            Serial.println(" ");

            Serial.println("LDRmediapost");
            Serial.print(LDRmediapost, DEC);
            Serial.println(" ");
            Serial.println("LDRmedia");
            Serial.print(LDRmedia, DEC);
            Serial.println(" ");


            // Si se redujo el valor de la diferencia se considera que el movimiento mejora y se puntua con +5 en la matriz de base de conocimiento
            if ((abs(LDRdiffpost) - abs(LDRdiff)) < 0 && abs(LDRdiffpost - LDRdiff) > 20 && (LDRmediapost < LDRmedia) && fila != 3 && (moveMD != 2 && moveMI != 2)) {
              auxX = (moveMD + 1) + (fila);
              BCMD[auxX] += 5;
              auxY = (moveMI + 1) + (fila);
              BCMI[auxY] += 5;


              Serial.println("Fila");
              Serial.print(fila, DEC);
              Serial.println(" ");
              Serial.println("auxX");
              Serial.print(auxX, DEC);
              Serial.println(" ");
              Serial.println("moveMD");
              Serial.print(moveMD, DEC);
              Serial.println(" ");
              Serial.println("auxY");
              Serial.print(auxY, DEC);
              Serial.println(" ");
              Serial.println("moveMI");
              Serial.print(moveMI, DEC);
              Serial.println(" ");

            }


            Serial.println("Diferencia actual");
            Serial.print(LDRdiffpost, DEC);
            Serial.println(" ");
            Serial.println("Diferencia anterior");
            Serial.print(LDRdiff, DEC);
            Serial.println(" ");
            Serial.println("Mov motor derecho");
            Serial.print(moveMD, DEC);
            Serial.println(" ");
            Serial.println("Mov motor izquierdo");
            Serial.print(moveMI, DEC);
            Serial.println(" ");

            Serial.println(" ");
            Serial.println("Matrices motor derecho");
            for (j = 0; j < 3; j++) {
              for (i = 0; i < 3; i++) {

                Serial.print(BCMD[i + (3 * j)], DEC);
                Serial.print(" ");
              } Serial.println(" ");
            }
            Serial.println("Matrices motor izquierdo");
            for (j = 0; j < 3; j++) {
              for (i = 0; i < 3; i++) {

                Serial.print(BCMI[i + (3 * j)], DEC);
                Serial.print(" ");
              } Serial.println(" ");

            }
      */
    
	
	
	
}
