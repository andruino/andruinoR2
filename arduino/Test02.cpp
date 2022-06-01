#include <Arduino.h>
#include "defines.h"
#include "CtrlMotores.h"


extern int estado;

extern int valores[10];


      //Test2: Merodea sin chocar usando los ultrasonidos.

void Test02iiiaaa()
{
  pararMotores();
  motorIzqPWM (250,0);
  motorDchPWM (250,0);
	
}

void Test02iiittt()
{
	 estado = 1020;
}

void Test02Loop()
{
	
      if (valores[0] < 200 && valores[0] > 0 && valores [2] >= 200) {
        pararMotores();
		motorIzqPWM (0,250);
        motorDchPWM (250,0);
      }
      if (valores[2] < 200 && valores[2] > 0 && valores [0] >= 200) {
        pararMotores();
        motorIzqPWM (250,0);
        motorDchPWM (0,250);
      }
      if (valores[0] < 100 && valores[1] < 100 && valores[2] < 100 && (valores[1] > 0 || valores [1] > 0 || valores[2] > 0)) {
        pararMotores();
        motorIzqPWM (0,250);
        motorDchPWM (0,250);
        delay(500);
        pararMotores();
        motorIzqPWM (0,250);
        motorDchPWM (250,0);
        delay(600);
        pararMotores();
      }
      if ((valores[0] >= 200 || valores[0] == 0) && (valores [1] >= 100 || valores[1] == 0) && (valores[2] >= 200 || valores[2] == 0)) {
        pararMotores();
		motorIzqPWM (250,0);
        motorDchPWM (250,0);
      }


	
}
