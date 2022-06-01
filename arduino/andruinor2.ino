/*************************************************************************************************
 * Arduino Firmware for AndruinoR2 low-cost ROS robot, based on Android and Arduino
 * Version:  01.00
 * Date: 2017-04-19
 * Autors: Paco López, Federico Cuesta
 **************************************************************************************************/

#include <math.h>

#include "defines.h"

#include "CtrlMotores.h"
#include "Twist.h"
#include "Test01.h"
#include "Test02.h"
#include "Test03.h"
#include "Test04.h"
#include "Test05.h"
#include "Test06.h"
#include "Test07.h"
#include "Test08.h"
#include "Test09.h"
#include "Test10.h"
#include "Test11.h"
#include "Test12.h"
#include "Test13.h"
#include "Test14.h"
#include "Test15.h"
#include "Test16.h"
#include "Test18.h"




//////////////////////////////////////////////////////////////////////////
// Parámetros
//////////////////////////////////////////////////////////////////////////
float parameters[21];


//////////////////////////////////////////////////////////////////////////
// Definición de variables globales
//////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////
//Variables de estado
//////////////////////////////////////////////////////////////////////////
int estado = 0;
int sub_estado;
float azimut = 0;
float omega = 0;

float x = 0;
float y = 0;
float azimut_odometry = 0;
float azimut_odometry_iiikkk = 0;

float secuencia = 0;
//////////////////////////////////////////////////////////////////////////
//Valores de sensores
//////////////////////////////////////////////////////////////////////////

int valores[10];


//Variables de azimut relaciondas con odometria
float timeAzimut = 0;
float dtAzimut = 0;
float incAzimut = 0;




float distancia = 0;
float tiempo1 = 0;
float tiempo2 = 0;



//Variables de la recepcion de comando en formato Twist
float cmdVel;
float cmdVelAnterior = 0; // En caso de que reciba orden con el mismo valor no actuliza AzimutRef, manteniendo el anterior
float cmdOmega;
float cmdAzimut;
float ICCR; // Radio de curvatura
float incOme; //Incremento de omega/dt, estimado por las velocidades angulesy la geometria del robot.


//Cadenas de entrada de datos
String inputString = "";
String trozo1Str = "";
String trozo2Str = "";
String trozo3Str = "";

// Flag de cadena recibida
boolean commandComplete = false;


//////////////////////////////////////////////////////////////////////////
// Variables de Tiempo d ejecución de loop principal
//////////////////////////////////////////////////////////////////////////
unsigned long timeLoop; //Tiempo de ciclo
unsigned long dtLoop;
unsigned long timeTx; //Tiempo de transmision, para evitar que sea en cada ciclo (y así bajar consumo de CPU en Android)
unsigned long dtTx;



//////////////////////////////////////////////////////////////////////////
// Setup
//////////////////////////////////////////////////////////////////////////

void setup() {

  timeLoop = millis();
  timeTx = millis();

  // Inicia comunicación serie
  Serial.begin(115200);//Serial.begin(9600);
  inputString.reserve(300);
 
  // FCR 01/21: Reservamos espacio para los trozos para evitar problemas de fraccionamiento de memoria
  // Habría que ajustar el tamaño, porque junto al anterior ocupa mucho espacio
  trozo1Str.reserve(100);
  trozo2Str.reserve(100);
  trozo3Str.reserve(100);
  
  Serial.println("AndruinoR2>>>");

  // Prueba motores
  motorIzqPWM (250,0);
  motorDchPWM (250,0);
  delay(300);
  motorIzqPWM (0,250);
  motorDchPWM (0,250);
  delay(300);
  pararMotores();
  // Fin Prueba motores

  // Inicia parametros
  //1-2 Factores de correción entre la velocidad dada en el comando y la que se debe aplicar para que en la práctica se obtenga la deseada en el robot real
  parameters[0] = 0.1; //R2D PENDIENTE //Kvel = 1.1; //1.089 //vel_lineal=parameters[0]*(omega_Der+omega_Izq), valor encontrado experimentalmente en test 7
  parameters[1] = 1.0; //R2D PENDIENTE //Komega = 1.0;

  //3-5 Parámetros del modelo cinemático
  parameters[2] = 1.0; //R2D es diferencial //Kssr = 8.34; //Modificacion del radio de las ruedas, para hacer equivalente el modelo Skid Steer al Diferencial
  parameters[3] = 1.0; //R2D es diferencial //Kssb = 1.65; //Modificacion de la distancia entre ruedas, para hacer equivalente el modelo Skid Steer al Diferencial
  parameters[4] = 1.0; //R2D es diferencial //KssIccr = 1; //Modificacion del radio de curvatura, para hacer equivalente el modelo Skid Steer al Diferencial

  //6-9 Parámetros de la línea entre PWM y omega
  parameters[5] = 83.12;  //R2D PENDIENTE //SlopeLeft = 111.5; // Pendiente de la relación entre PWM y velodidad angular de rueda izquierda
  parameters[6] = 129.55; //R2D PENDIENTE //OffsetLeft = 112.41; // Offset de la relación entre PWM y velodidad angular de rueda izquierda
  parameters[7] = 77.28;  //R2D PENDIENTE //SlopeRight = 101.5;
  parameters[8] = 133.9; //R2D PENDIENTE //OffsetRighet = 103.41;

  //10-12 Parámetros de los PID, para movimiento en línea recta, giro y movimiento en curva

  parameters[9] = 100.0; //R2D PENDIENTE //100.0; //KpLine = 100; //80; //80 Ajustado experimentalmente
  parameters[10] = 3.0; //R2D PENDIENTE //KiLine = 3;
  parameters[11] = 20.0; //R2D PENDIENTE //KdLine = 0 ;

  //13-15 Parámetros de los PID, para movimiento en  giro
  parameters[12] = 20.0; //R2D PENDIENTE //KpSpin = 30;
  parameters[13] = 0.0 ; //R2D PENDIENTE //KiSpin = 0;
  parameters[14] = 5.0; //R2D PENDIENTE //KdSpin = 0;

  //16-18 Parámetros de los PID, para movimiento en arco
  parameters[15] = 10.0; //R2D PENDIENTE //KpArc = 10;
  parameters[16] = 0.0; //R2D PENDIENTE //KiArc = 0;
  parameters[17] = 0.0; //R2D PENDIENTE //KdArc = 0;

  //19-20 Parámetros de los valores de PWM máximos y mínimos que hacen que se mueva el robot

  parameters[18] = 255.0; //R2D PENDIENTE //PWMAX = 255;
  parameters[19] = 125.0; //R2D PENDIENTE //PWMMIN = 155;

  //int PWMMEDIO = (((int)parameters[18]) + ((int)parameters[19])) / 2;

  // Inicia el estado
  estado = 0; //Estado normal esperando algún comando.
  x = 0;
  y = 0;
  azimut_odometry = 0; //Valor inicial

  
}


//////////////////////////////////////////////////////////////////////////
// Loop principal
//////////////////////////////////////////////////////////////////////////

void loop()
{
  /////////////////////////////////////////////////////////////
  //Variables locales
  /////////////////////////////////////////////////////////////

  int i, j;
  float distancia_anterior;

  /////////////////////////////////////////////////////////////
  //Calculo del tiempo de Loop Principal
  /////////////////////////////////////////////////////////////
  dtLoop = millis() - timeLoop;

  //Actualización del tiempo, para la siguiente pasada del bucle
  timeLoop = millis();

  /////////////////////////////////////////////////////////////
  //Sensores Ultrasonidos
  /////////////////////////////////////////////////////////////

  valores[0] = leeUS(echoPin1, trigPin1);
  valores[1] = leeUS(echoPin2, trigPin2);
  valores[2] = leeUS(echoPin3, trigPin3) ;

  /////////////////////////////////////////////////////////////
  // Lectura Sensores LDRs
  /////////////////////////////////////////////////////////////

  valores[3] = analogRead(LDR1); // Leemos el valor de A0.
  valores[4] = analogRead(LDR2); // Leemos el valor de A1.
  valores[5] = analogRead(LDR3); // Leemos el valor de A2.

  /////////////////////////////////////////////////////////////
  // Calculo de coordenadas X,Y
  /////////////////////////////////////////////////////////////

  //Calculo de la distancia recorrida (estimación basado en la velocidad indicada y en el tiempo transcurrido), si se esta moviendo por un comando Twist (Estados 2000s)
  distancia_anterior = distancia;
  if (timeLoop != 0 && (estado >= 2000 && estado < 3000)) {
    //distancia += (cmdVel * (float)dtLoop) / 1000L;
    distancia += (cmdVel * (float)dtLoop); //En mm
  }

  if (timeLoop != 0 && estado == 1060) {

    //distancia += (0.15 * (float)dtLoop) / 1000L;
    distancia += (0.15 * (float)dtLoop); //En mm

  }

  //Estimación (x,y), tras recibir un comando de Twist. Supone que donde se enciende es el 0,0 y el azimut 0 el que indica el giróscopo (Drift!!!!: FALTA Mejorarlo usando los sensores de Ultrasonidos)
  //Calculo de la posición x,y,omega (para valores del estado del rango de 2000's). Basada en la geometría.
  if (estado == 2010 || estado == 1060) { //Anda en línea recta
    //x += (distancia - distancia_anterior) * cos(azimut_odometry);
    //y += (distancia - distancia_anterior) * sin(azimut_odometry);
    x += (distancia - distancia_anterior) * cos((azimut_odometry_iiikkk / 10000L));
    y += (distancia - distancia_anterior) * sin((azimut_odometry_iiikkk / 10000L));
  } else if (estado == 2020) { //Gira
    //No actualiza los valores de x e y.
  } else if (estado == 2000) { //Realiza un arco de circunferencia, vel!=0 y omega!=0
    /* Lo calcula cunado recibe datos de azimut
     x+= -sin(azimut) + ICCR * sin(azimut+(incOme*dtLoop/1000));
     y+= -cos(azimut) - ICCR * cos(azimut+(incOme*dtLoop/1000));
    */
  }

  /////////////////////////////////////////////////////////////
  // Tx valores del sensor al Android
  /////////////////////////////////////////////////////////////

  //Si está calibrando o haciendo test no muestra valores de los sensores

  //if (estado != 1040 && estado != 1041 && estado != 1050 && estado != 1060 && estado != 1070) {

  dtTx = millis() - timeTx;
  if ((estado < 1030 || estado >= 2000) && (dtTx > TEMPOTX)) {

    // TX datos ultasonidos y ldr al Android
    // Estructura de Mesaje: ooosssUS1wwUS2wwUS3wwLDR1wwLDR2wwLDR3wwTiempoLoopwwwwDistanciaRecorridaTwistwwXwwYwwAzimut###
    Serial.print("ooosss");
    for (i = 0; i < 6; i++) {
      Serial.print(valores[i]);
      Serial.print("ww");
    }
    Serial.print(String(dtLoop));
    Serial.print("ww");
    Serial.print((distancia * 1000.0), 4);
    Serial.print("ww");
    Serial.print((x * 1000.0), 4);
    Serial.print("ww");
    Serial.print((y * 1000.0), 4);
    Serial.print("ww");
    Serial.print(azimut_odometry_iiikkk, 4);
    //Serial.print(String((int)(azimut * 1000)));
    //Serial.print(String((int)(azimut_odometry * 1000)));
    //Serial.print( String((int)(1000.0*atan2(sin(azimut_odometry), cos(azimut_odometry)))));
    Serial.print("ww");
    Serial.print(String(estado)); //Cambio septiembre 2016
    Serial.print("ww");
    Serial.println("###"); // Fin de mensaje
    //Serial.println("");
    //Serial.println(String(dtLoop) + "ww" + distancia*1000.0 + "ww" + x*1000.0 + "ww" + y*1000.0 + "ww" + azimut_odometry_iiikkk + "ww" + String(estado) + "ww###");
    Serial.flush();

    timeTx = millis();

  }
  //}

  /////////////////////////////////////////////////////////////
  // RX datos desde Android.
  /////////////////////////////////////////////////////////////

  if (commandComplete) {


    if(inputString.startsWith("iiimmm")){
		                      motoresPWM(inputString.substring(inputString.indexOf("mmm") + 3, inputString.indexOf("ww")).toInt(), inputString.substring(inputString.indexOf("ww") + 2, inputString.indexOf("ww###")).toInt());
													delay(3000);
													pararMotores();
													estado = 0;
    } 
		else if (inputString.startsWith("iiiqqq")){ 
		                      Test16iiiqqq();
		}
		else if (inputString.startsWith("iiiaaa")){
		                    	trozo1Str = inputString.substring(inputString.indexOf("aaa") + 3, inputString.indexOf("ww"));
													trozo2Str = inputString.substring(inputString.indexOf("ww") + 2, inputString.indexOf("ww###"));

													//Calcula el incremento del azimut respecto al valor anterior, antes de refrescar la variable azimut. Y los valores del incremento del tiempo y del tiempo
													incAzimut = trozo1Str.toFloat() - (azimut * 10000.0);
													dtAzimut = millis() - timeAzimut;
													timeAzimut = millis();

													//Nuevo valores de azimut y omega
												  azimut = trozo1Str.toFloat() / 10000.0;
													omega = trozo2Str.toFloat() / 10000.0;

													// Calculamos el estado sin subestado
													int estadof = (int) estado/10;
													switch (estadof){
														case   0: pararMotores(); break;
														case 101: Test01iiiaaa(); break;
														case 102: Test02iiiaaa(); break;
														case 104: Test04iiiaaa(); break;
														case 105: Test05iiiaaa(); break;
														case 106: Test06iiiaaa(); break;
														case 107: Test07iiiaaa(); break;
														case 108: Test08iiiaaa(); break;
														case 110: Test10iiiaaa(); break;
														case 111: Test11iiiaaa(); break;
                            //case 113: Test13iiiaaa(); break;
														case 114: Test14iiiaaa(); break;
														case 200:
														case 201:
														case 202: Twistiiiaaa(); break;
														default: break;
													  };
		}
    else if (inputString.startsWith("iiittt")){
                          trozo1Str = inputString.substring(inputString.indexOf("ttt") + 3, inputString.indexOf("ww"));
													trozo2Str = inputString.substring(inputString.indexOf("ww") + 2, inputString.indexOf("ww###"));

													//Valores por defecto en test
													//No considera el comando de velocidad anterior, en cualquier test
													cmdVelAnterior = 0;
													cmdVel = 0.15;
													cmdOmega = 0.5;

													switch (trozo1Str.toInt()) {
														  case  1: Test01iiittt(); break;
														  case  2: Test02iiittt(); break;
														  case  3: Test03iiittt(); break;
														  case  4: Test04iiittt(); break;
														  case  5: Test05iiittt(); break;
														  case  6: Test06iiittt(); break;
														  case  7: Test07iiittt(); break;
														  case  8: Test08iiittt(); break;
														  case  9: Test09iiittt(); break;
														  case 10: Test10iiittt(); break;
														  case 11: Test11iiittt(); break;
														  case 12: Test12iiittt(); break;
                              case 13: Test13iiittt(); break;
														  case 14: Test14iiittt(); break;
														  case 15: Test15iiittt(); break;
														  case 16: Test16iiittt(); break;
														  case 18: Test18iiittt(); break;
														  default: break;
													};
    }
		else if (inputString.startsWith("iiippp")){
		                      trozo1Str = inputString.substring(inputString.indexOf("ppp") + 3, inputString.indexOf("ww"));
													trozo2Str = inputString.substring(inputString.indexOf("ww") + 2, inputString.indexOf("ww###"));

													int indice = trozo1Str.toInt();
													//Admite hasta 21 parametros
													if (indice >= 0 && indice < 21)
														parameters[indice] = trozo2Str.toFloat();
		}
		else if (inputString.startsWith("iiikkk")){
		                      Twistiiikkk(); //Recibe una orden tipo twist (velocidad lineal / velocidad angular)
		}
		else if (inputString.startsWith("iiibbb")){
		                     	// iiibbbXXXww### donde XXX es la correlación respecto al origen (destino buscado)
													//LDROffSet=beacon; // PRUEBA !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		}

    else if (inputString.startsWith("iiinnn")){ //Recibimos respuesta al test de sincronización
                          tiempo2 = millis();
                          trozo1Str = inputString.substring(inputString.indexOf("nnn") + 3, inputString.indexOf("ww")); //Tomamos tiempo
                          trozo2Str = inputString.substring(inputString.indexOf("ww") + 2, inputString.indexOf("dd")); //Tomamos secuencia
                          trozo3Str = inputString.substring(inputString.indexOf("dd") + 2, inputString.indexOf("ww###")); //Tomamos fecha

                          //retardo = (retardo - trozo1Str.toInt())/2;

//                          Serial.print("oooccc");
//                          Serial.print(String(estado));
//                          Serial.print("ww");
//                          Serial.print(tiempo2, 8);
//                          Serial.print("ww");
//                          Serial.print(trozo1Str);
//                          Serial.print("ww");
//                          Serial.print(trozo2Str);
//                          Serial.println("ww###");
                          Serial.println("oooccc" + String(estado) + "ww" + tiempo2  + "ww" + trozo1Str + "ww" + trozo2Str + "ww" + trozo3Str + "ww###");
                          Serial.flush();
                          //delay(300);
                          sub_estado = 1130;
    }

		//Si el mensaje es incorrecto (ninguno de los anteriores) lo tira
			
    // clear the string:
    inputString = "";
    commandComplete = false;
  }


  //
  // Fin de RX debería ser más limpio y sólo codificar el estado al recibir un mensaje.
  //

  //
  // Ejecución por estado
  //

  switch (estado) {
    case 1020:  Test02Loop(); break;
    case 1030:	Test03Loop(); break;
    case 1070:  Test07Loop(); break;
    case 1130:  Test13Loop(); break;
    case 1150:	Test15Loop(); break;
	  case 1180:  Test18Loop(); break;
    default:    break;
  };

  // Fin de red de petri de ejecución




}




// Ecuación de la recta.
float interpola(float x, float x_min, float y_min, float x_max, float y_max)
{
  //Lo hace a una recta, pero quizás fuese mejor más puntos.MIRAR!!!
  //http://www.matematicasvisuales.com/html/analisis/interpolacion/lagrange.html
  //Debe existir librería
  float y;
  y =  y_min +  ((x - x_min) * (y_max - y_min) / (x_max - x_min)) ;
}



int leeUS(int echoPin, int trigPin) {

  long duration;

  pinMode(trigPin, OUTPUT);
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  //Lectura de echo
  pinMode(echoPin, INPUT);
  duration = pulseIn(echoPin, HIGH, 15000);

  // Conversion a mm
  return (int) microsecondsToMilimeters(duration);//microsecondsToCentimeters(duration);
}


long microsecondsToCentimeters(long microseconds)
{

  return microseconds / 29 / 2;
}

long microsecondsToMilimeters(long microseconds)
{

  return 10 * microseconds / 29 / 2;
}


void serialEvent() {
  // FCR 01/21: Validamos que no se ha recibido un comando completo antes de añadir más caracteres. 
  // De otro modo se podrían seguir añadiendo caracteres al inputstring mientras se está procesando y
  // ejecutando el primer comando, y, como después de procesarlo, se hace inputString="" se podrían perder
  // trozo de los mensajes.  
  //while (Serial.available()) {
  while (!commandComplete && Serial.available()) {

    char inChar = (char)Serial.read();

    inputString += inChar;

    String finCadena = "###";
    //Comienza por iii (entrada desde el Android al Arduino) y termina con ### (fin de mensaje)
    if (inputString.endsWith("###") || (inputString.indexOf("###") > 1) ) {
      commandComplete = true;
    }

  }

}
