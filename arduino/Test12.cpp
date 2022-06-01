#include "defines.h"
#include <Arduino.h>

extern int estado;

extern int estadoPID;
extern float azimut_odometry;
extern float x;
extern float y;
extern float distancia;

void pararMotores();


void Test12iiittt()
{
       //Para el root e inicia las variables de estado
        estado = 0;
        pararMotores();
        distancia = 0;
        x = 0;
        y = 0;
        azimut_odometry = 0;
        estadoPID = -1;
 
}