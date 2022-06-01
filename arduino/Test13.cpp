#include <Arduino.h>
#include "defines.h"
#include "CtrlMotores.h"

extern int estado;

extern int estadoPID;
extern float azimut_odometry;
extern float secuencia;
extern float tiempo1;

static unsigned long  tiempoEstado1130 = 0;
static unsigned long tiempoEstado1131 = 0;
extern int sub_estado;

extern float tiempo2;
extern String inputString;
extern String trozo1Str;
extern String trozo2Str;
extern boolean commandComplete;

void Test13iiiaaa()   
{
    
        //Durante 5 segundos
        if ((tiempoEstado1130 + 5000) > millis()) {   
          secuencia++;
          tiempo1 = millis();
          Serial.print("ooonnn");
          Serial.print(tiempo1, 8);
          Serial.print("ww");
          Serial.print(secuencia, 4);
          Serial.print("ww###");
          Serial.print('\n');

        } else {

          estado = 0;
          secuencia = 0.0;
        }

}

//SINCRONIZACIÓN MENSAJES
void Test13iiittt()
{
          estado = 1130;
          sub_estado = 1130;
          tiempoEstado1130 = millis(); 
          secuencia = 0.0;
//
//          tiempo1 = millis();
//          Serial.print("ooonnn");
//          Serial.print(tiempo1, 8);
//          Serial.print("ww");
//          Serial.print(secuencia, 4);
//          Serial.print("ww###");
//          Serial.print('\n');

}

void Test13Loop(){

        if ((tiempoEstado1130 + 20000) > millis()) {

          if (sub_estado == 1130){
            secuencia++;
            tiempo1 = millis();
//            Serial.print("ooonnn");
//            Serial.print(tiempo1, 8);
//            Serial.print("ww");
//            Serial.print(secuencia);
//            Serial.print(";secuencia"); //Para que Android coloque su propio nº de secuencia
//            Serial.println("ww###");
            Serial.println("ooonnn" + String(tiempo1) + "ww" + String(secuencia)  + ";secuencia" + "ddtiempo" + "ww###");
            Serial.flush();   //Espera hasta que se mande el mensaje por el puerto serie

            sub_estado = 1131;
            tiempoEstado1131 = millis();
          }
          else if (sub_estado == 1131){
            if((tiempoEstado1131 + 1000) > millis())
              sub_estado = 1131;
//              pararMotores();
            else
              sub_estado = 1130;
          }
         }
         else
          estado = 0;
     
}



