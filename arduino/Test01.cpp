#include <Arduino.h>
#include "CtrlMotores.h"

extern int estado;
extern int estadoPID;



void Test01iiiaaa()
{
        pararMotores();
		motorIzqPWM (250,0);
        motorDchPWM (250,0);
	
}

      //Test1: Para motores y vuelve al estado 0

void Test01iiittt()
{
	    pararMotores();
        estado = 0;
        estadoPID = -1;
		
}
