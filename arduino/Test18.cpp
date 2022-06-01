#include "defines.h"
#include <Arduino.h>

extern int estado;

extern float parameters[21];

void Test18iiittt()
{
 //Test18: Envía el valor de todos los parámetros
 estado = 1180;
}

void Test18Loop()
{
		  //Volcamos el vector de parámetros completo:
//Kvel, Komega, Kssr, Kssb, KssIccr, SlopeLeft, OffsetLeft, SlopeRight, OffsetRigh, KpLine, KiLine, KdLine, KpSpin, KiSpin, KdSpin, KpArc, KiArc, KdArc, PWMAX, PWMMIN
      int numparametros=20;

	  Serial.print("oooccc" + String(estado) + "ww"+ String(numparametros)+"ww");
      for(int j=0;j<=numparametros-1;j++)
 		  Serial.print(String(parameters[j])+ "ww");
	  Serial.println("###");
	 
	  estado = 0;
	
	
}
