/*************************************************************************************************
 * Firmware de Arduino para robot Andruino R2
 * Version: 48 (sin limpiar !!!!!)
 * Autor: Paco López, dirigido por Federico Cuesta
 */

#include <math.h>

#define Kvel 1.089 //10.0 //vel_lineal=Kvel*(omega_Der+omega_Izq), valor encontrado experimentalmente en test 7 (aqui si hay control proporcional!!)
#define Komega 1.0 //20.0 //

#define DISTANCIA_RUEDAS 0.1490
#define RADIO_RUEDA 0.01785

#define SENSIBLE_US 10 //1 cm
#define DISTANCIA_CALIBRACION 500 //50 cm


#define PWMMAX 255
#define PWMMIN 175
#define PWMMEDIO 215

#define omegaMIN 0.6
#define omegaMAX 1.5

#define velMAX 1.9
#define velMIN 0.9

#define trigPin1 11
#define echoPin1 12
#define trigPin2 8
#define echoPin2 7
#define trigPin3 4
#define echoPin3 2


#define LDR1 0
#define LDR2 1
#define LDR3 2
#define analogInputIzq 0
#define analogInputDer 2
#define analogInputCen 1

//Motores

#define motorIAvance 9
#define motorIRetroceso 10
#define motorDAvance 5
#define motorDRetroceso 6

//Led de actividad
#define led 13

//////////////////////////////////////////////////////////////////////////
// Definición de variables globales
//////////////////////////////////////////////////////////////////////////

//Variables es estado
int estado = 0;
float azimut = 0;
float omega = 0;
float x = 0;
float y = 0;

//Valores de sensores
int valores[10];


//float omegaAnterior = 0;
//float omegaLP = 0;
int incPWM = 0;
unsigned long tiempoOmega = 0; //tiempoEstado1080
float mediaOmega = 0;

// Variables Estado 1080
// Representación gráfica (lineal) PWM-Velocidad Angular
float PWM_Omega[8]; // Valores de velocidad angular máximo y mínimos, Orden: D_avanza, I_avanza, D_retrocede, I_retrocede
int numOmegas;      // Numero de omegas para promediar



// Variables Estado 1050, pid en velocidad angular
unsigned long  tiempoEstado1050 = 0;

unsigned long incTErrPWM = 0;
unsigned long tErrPWM = 0;
unsigned long tErrPWMAnt = 0;

float errorPWMAnterior = 0;
float errorPWMSum = 0;
float errorPWMDiff = 0;

float outPID = 0;
int outPIDPWM = 0;

//PID
float errorAzimut = 0;

//Variables de azimut relaciondas con odometria
float timeAzimut = 0;
float dtAzimut = 0;
float incAzimut = 0;

//Variables Estado 1060, pid en angulo
unsigned long  tiempoEstado1060 = 0;
float azimutRef = 0;                    //referencia del azimut para pid
float azimutAnterior = 0; //Usado para el PIS
float distancia = 0;

//Variables Estado 1070, calibración en distancia
//Debe recorrer un metro en línea recta (necesita Pid en angulo) medido con el sensor de distancia y mide el tiempo, así calcula velocidad
int distanciaEstado1070 = 0;
unsigned long incTiempoEstado1070;
unsigned long tiempoEstado1070 = 0;
float velEstado1070 = 0;


//Variables Estado 11 (1110), prueba la función de giro
unsigned long tiempoEstado1110 = 0;
int contadorEstado1110 = 0; //Contador de estado de sobreoscilaciones del PD de giro

//Variables de la recepcion de comando en formato Twist
//String cmdVelStr;
//String cmdOmegaStr;
float cmdVel;
float cmdOmega;

float ICCR; // Radio de curvatura
float incOme; //Incremento de omega/dt, estimado por las velocidades angulesy la geometria del robot.


//Variabes de l Sensor Wifi
/*
int beacon = 0;
int beaconAnterior = 0;
int diffBeacon = 0;
int incOffSet = 0;
int filabeacon = 0;
int LDROffset = 0;
*/



//Cadenas de entrada de datos (posiblemente puede ser reducidas el numero de variables !!!
String inputString = "";
//String auxString = "";
String trozo1Str = "";
String trozo2Str = "";
//String trozo3Str = "";

//String motor1Str = "";
//String motor2Str = "";
//String azimutStr = "";
//String omegaStr = "";
//String trozo1Str = "";
//String paramTestStr = "";
//String cmdBeaconStr = "";


boolean commandComplete = false;
//int orden = 0;
//char Char[8] =";




//Variables para el calculo de velocidad y posición basada en sensor de distancia (odometria fake)
//unsigned long tiempoAntes = -1;
//unsigned long tiempoDespues = -1;
//unsigned long incrementoTiempo;
//unsigned long tiempoFinTest;
//unsigned long auxiliarlong;
//int incrementoDistancia;
//int incDistTiempoAcc;
//int valoresAntes[10];
//int flag_movimiento = 0;

// VAriables globales del programa de 2010
/*
// Variables globales
//int valor = 255;
int value = 0;
//Movimientos del motor izquierdo y derecho
int moveMI = 0;
int moveMD = 0;
//Valores de los sensores, LDRdiff y LDRmedia
int LDRIzq = 0;
int LDRCen = 0;
int LDRDer = 0;
int LDRdiff = 0; //diferencia antes del ensayo
int LDRsigno = 0;
int LDRdiffpost = 0; // diferencia después del ensayo
int LDRmedia = 0; //media antes del ensayo
int LDRmediapost = 0; // media despues del ensayo
//int LDROffSet = 0;
int threshold = 200;
int fila; // Fila de la matriz
int filaMD; // Fila de motor derecho
int filaMI; // Fila de motor izquierdo
//Matrices Entrada / Salida
// Para obtener velocidad variable
//int ESMD[3][3]={{-200,0,200},{-200,0,200},{-200,0,200}};
//int ESMI[3][3]={{-200,0,200},{-200,0,200},{-200,0,200}};
//int ESMD[9] = { -200, 0, 200, -200, 0, 200, -200, 0, 200}; //Matrices sustituidas por vectores por bug en compilador
//int ESMI[9] = { -200, 0, 200, -200, 0, 200, -200, 0, 200};

//Matrices de Base de Conocimiento
//int BD[3][3]={{0,0,0},{0,0,0},{0,0,0}}; //Experencia del motor 1
//int BI[3][3]={{0,0,0},{0,0,0},{0,0,0}}; //Experencia del motor 2
int BCMD[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0}; //Matrices sustituidas por vectores por bug en compilador
int BCMI[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
int BCOffSet[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0}; //Base de conocimiento del offset de ldr

//Fin de variables de 2010
*/

// Tiempo d ejecución de loop principal
unsigned long timeLoop;
unsigned long dtLoop;


//////////////////////////////////////////////////////////////////////////
// Setup
//////////////////////////////////////////////////////////////////////////

void setup() {

  timeLoop = millis();

  // initialize serial communication:
  Serial.begin(115200);//Serial.begin(9600);
  inputString.reserve(200);
  Serial.println("Andruino ROS##Paco Lopez##2015-2016");

  // Prueba motores
  analogWrite(motorIAvance, 250);
  analogWrite(motorIRetroceso, 0);
  analogWrite(motorDAvance, 250);
  analogWrite(motorDRetroceso, 0);
  delay(300);
  analogWrite(motorIAvance, 0);
  analogWrite(motorIRetroceso, 250);
  analogWrite(motorDAvance, 0);
  analogWrite(motorDRetroceso, 250);
  delay(300);
  pararMotores();
  // Fin Prueba motores

  // Inicia el estado
  estado = 0; //Estado normal esperando algún comando.
  x = 0;
  y = 0;

}


//////////////////////////////////////////////////////////////////////////
// Loop principal
//////////////////////////////////////////////////////////////////////////

void loop()
{

  //Variables locales
  int i, j;
  float distancia_anterior;

  //Calculo del tiempo de Loop Principal
  dtLoop = millis() - timeLoop;

  //Calculo de la distancia recorrida, si se esta moviendo por un comando Twist (Estados 2000s)
  distancia_anterior = distancia;
  if (timeLoop != 0 && estado >= 2000 && estado < 3000) {
    distancia += (cmdVel * dtLoop) / 1000;
  }
  //Actualización del tiempo, para la siguiente pasada del bucle
  timeLoop = millis();

  //Estimación (x,y), tras recibir un comando de Twist. Supone que donde se enciende es el 0,0 y el azimut 0 el que indica el giróscopo (Drift!!!!: FALTA Mejorarlo usando los sensores de Ultrasonidos)
  //Calculo de la posición x,y,omega (para valores del estado del rango de 2000's). Basada en la geometría.
  if (estado == 2010) { //Anda en línea recta
    x += (distancia - distancia_anterior) * cos(azimut);
    y += (distancia - distancia_anterior) * sin(azimut);
  } else if (estado == 2020) { //Gira
    //No actualiza los valores de x e y.
  } else if (estado == 2000) { //Realiza un arco de circunferencia, vel!=0 y omega!=0
    /* Lo calcula cunado recibe datos de azimut
     x+= -sin(azimut) + ICCR * sin(azimut+(incOme*dtLoop/1000));
     y+= -cos(azimut) - ICCR * cos(azimut+(incOme*dtLoop/1000));
    */
  }

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
  // Tx valores del sensor al Android
  /////////////////////////////////////////////////////////////

  //Si está calibrando o haciendo test no muestra valores de los sensores
  //if (estado != 1040 && estado != 1041 && estado != 1050 && estado != 1060 && estado != 1070) {
  if (estado < 1040 || estado >= 2000) {
    // TX datos ultasonidos y ldr al Android
    // Estructura de Mesaje: ooosssUS1wwUS2wwUS3wwLDR1wwLDR2wwLDR3wwTiempoLoopwwwwDistanciaRecorridaTwistwwXwwYwwAzimut###
    Serial.print("ooosss");
    for (i = 0; i < 6; i++) {
      Serial.print(valores[i]);
      Serial.print("ww");
    }
    Serial.print(String(dtLoop));
    Serial.print("ww");
    Serial.print(String(distancia));
    Serial.print("ww");
    Serial.print(String(x));
    Serial.print("ww");
    Serial.print(String(y));
    Serial.print("ww");
    Serial.print(String(azimut));
    Serial.print("ww");
    Serial.print("###"); // Fin de mensaje
    Serial.println("");

  }

  /////////////////////////////////////////////////////////////
  // RX datos desde Android.
  /////////////////////////////////////////////////////////////


  if (commandComplete) {
    //Serial.println(inputString); // PAra verificar si el Android manda bien
    //Busca el principio de la cadena y sumo tres carracteres (correspondientes a ese principio de la cadena) y busco el final del primer campo con &&
    //auxString=inputString.substring(inputString.indexOf("@@@")+3,inputString.indexOf("&&"));

    //Depuración devuelve lo que recib@
    //Serial.println("Mensaje Recibido");
    //Serial.println(inputString);
    //Serial.println("###");

    //IMPORTANTE: CAMBIOAR POR SWITH/CASE FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
    if (inputString.startsWith("iiimmm")) {

      //String motor1Str = inputString.substring(inputString.indexOf("mmm") + 3, inputString.indexOf("ww"));
      //String motor2Str = inputString.substring(inputString.indexOf("ww") + 2, inputString.indexOf("ww###"));

      //motoresPWM(motor1Str.toInt(), motor2Str.toInt());
      motoresPWM(inputString.substring(inputString.indexOf("mmm") + 3, inputString.indexOf("ww")).toInt(), inputString.substring(inputString.indexOf("ww") + 2, inputString.indexOf("ww###")).toInt());

      /* ALORA!!!
      if (motor1Str.toInt() > 0) {
        analogWrite(motorDAvance, motor1Str.toInt());
        analogWrite(motorDRetroceso, 0);
      } else {
        analogWrite(motorDAvance, 0);
        analogWrite(motorDRetroceso, abs(motor1Str.toInt()));
      }

      if (motor2Str.toInt() > 0) {
        analogWrite(motorIAvance, motor2Str.toInt());
        analogWrite(motorIRetroceso, 0);
      } else {
        analogWrite(motorIAvance, 0);
        analogWrite(motorIRetroceso, abs(motor2Str.toInt()));
      }
      */


      // Suponemos que si recibe en algun motor un valor por encima de 150 de pwm se mueve (VERIFICAR)
      /*
      if (motor1Str.toInt() >= 150 or motor2Str.toInt() >= 150) {
        flag_movimiento = 1; //Recibio una orden de movimiento, va a calcular la velocidad / posicion
      } else {
        flag_movimiento = 0;
      }
      */
      //Tiempo que mantengo motores funcionando 3 segundos
      //delay(tiempoAccion);
      delay(3000);

      // Para motores e inicia el estado
      pararMotores();
      estado = 0;


      // clear the string:
      inputString = "";
      commandComplete = false;
    }

    else if (inputString.startsWith("iiiaaa")) {

      //azimutStr = inputString.substring(inputString.indexOf("aaa") + 3, inputString.indexOf("ww"));
      //omegaStr = inputString.substring(inputString.indexOf("ww") + 2, inputString.indexOf("ww###"));
      trozo1Str = inputString.substring(inputString.indexOf("aaa") + 3, inputString.indexOf("ww"));
      trozo2Str = inputString.substring(inputString.indexOf("ww") + 2, inputString.indexOf("ww###"));

      //Calcula el incremento del azimut respecto al valor anterior, antes de refrescar la variable azimut. Y los valores del incremento del tiempo y del tiempo
      //incAzimut = trozo1Str.toFloat() - azimut;
      incAzimut= atan2(sin(trozo1Str.toFloat() - azimut), cos(trozo1Str.toFloat() - azimut));
      dtAzimut = millis() - timeAzimut;
      timeAzimut = millis();

      //Nuevo valores de azimut y omega
      azimut = trozo1Str.toFloat();
      omega = trozo2Str.toFloat();

      // Si se desvía, ejecuto PID sobre ángulo
      // radians = (degrees * 71) / 4068


      //errorAzimut = azimutTest - azimut;
      //errorAzimut = atan2(sin(errorAzimut), cos(errorAzimut)); //Para que esté entre -pi  pi
      //degrees = (radians * 4068) / 71
      //errorAzimutGrados = (int)((errorAzimut * 4068) / 71);

      if (estado == 2000 && incAzimut != azimut) { //Realiza un arco de circunferencia, vel!=0 y omega!=0
        x +=  ICCR * (sin(azimut + incAzimut) - sin(azimut));
        y += ICCR * (cos(azimut) - cos(azimut + incAzimut));
        Serial.println("(cmd_ omegaIzq(Kvel * cmdVel, Komega * cmdOmega))");
        Serial.println((cmd_omegaIzq(Kvel * cmdVel, Komega * cmdOmega)));
        Serial.println("(cmd_omegaDer(Kvel * cmdVel, Komega * cmdOmega))");
        Serial.println((cmd_omegaDer(Kvel * cmdVel, Komega * cmdOmega)));
        Serial.println("ICCR");
        Serial.println(ICCR);
        Serial.println("azimut");
        Serial.println(azimut);
        Serial.println("incAzimut");
        Serial.println(incAzimut);
        
      }

      if (estado == 1011 || estado == 1012) {
        /*
        Eliminado
        */
      }


      //////////////////////////////////////////////////////////////////////////
      // Estado 1080 y siguientes 1081,1082,1083:
      // Buscando valores de PWM de velocidades MAX / MIN angulares, para después
      // interpolar por una línea
      // 20 segundos girando y aumentando la velocidad, cambiando de sentido
      // permite obtener de forma gráfica la relacion entre valores del PWM
      // y la velocidad angular dada por giróscopo.
      //////////////////////////////////////////////////////////////////////////


      if (estado >= 1080 && estado <= 1083) {
        if (estado == 1080) {
          analogWrite(motorIAvance, 0);
          analogWrite(motorIRetroceso, 0);
          analogWrite(motorDAvance, incPWM);
          analogWrite(motorDRetroceso, 0);
        } else if (estado == 1081) {
          analogWrite(motorIAvance, incPWM);
          analogWrite(motorIRetroceso, 0);
          analogWrite(motorDAvance, 0);
          analogWrite(motorDRetroceso, 0);
        } else if (estado == 1082) {
          analogWrite(motorIAvance, 0);
          analogWrite(motorIRetroceso, 0);
          analogWrite(motorDAvance, 0);
          analogWrite(motorDRetroceso, incPWM);
        } else if (estado == 1083) {
          analogWrite(motorIAvance, 0);
          analogWrite(motorIRetroceso, incPWM);
          analogWrite(motorDAvance, 0);
          analogWrite(motorDRetroceso, 0);

        }

        // Despues de 10 segundos girando y aumentando el PWM entre la diferencia de PWMMAX y PWMMIN, cambia de sentido



        if (abs(omega) > 0.02) {
          numOmegas += 1;
          mediaOmega += omega;
        }


        if ((tiempoOmega + 20000) < millis() && incPWM <= PWMMAX) {
          int aux;
          pararMotores();
          delay(500);
          tiempoOmega = millis();
          aux = 2 * (estado - 1080) + ((incPWM - PWMMIN) / (PWMMAX - PWMMIN));
          PWM_Omega[aux] = (float)(mediaOmega / numOmegas);
          Serial.println(";" + String(aux) + ";" + incPWM + ";" + String(PWM_Omega[aux]) + ";" + String(mediaOmega) + ";" + String(numOmegas)  + ";" + "###");
          mediaOmega = 0;
          numOmegas = 0;
          incPWM = incPWM + (PWMMAX - PWMMIN);
        } else if (incPWM > PWMMAX && estado == 1083) {
          estado = 0;
        } else if (incPWM > PWMMAX && (estado >= 1080 && estado < 1083)) {
          //Cambio de sentido de giro
          delay(2000);
          estado = estado + 1;
          mediaOmega = 0;
          numOmegas = 0;
          incPWM = PWMMIN; //Valor inicial del PWM
          tiempoOmega = millis();
          //Serial.println("ESTADO @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@");
          //Serial.println(estado);
          //Serial.println("TIEMPO @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@");
          //Serial.println(tiempoOmega);
        }

        // Imprime la tabla PWM / velocidad angular en un CSV para fácil representación y mmanipulación de datos
        Serial.println(";" + String(omega) + ";" + incPWM + ";" + "###"); //En radianes

        //Filtro paso de baja (para quitar el ruido del giróscopo)
        //y[i] := y[i-1] + α * (x[i] - y[i-1])
        //omegaLP = omegaLP + 0.25 * (omega - omegaLP);
        //Serial.println(":"+ String(omegaLP) + ";" + omegaStr + ";" + incPWM + "###");

      }




      //////////////////////////////////////////////////////////////////////////
      // Estado 1040 y siguientes 1041,1042,1043:
      // Creando la tabla de velocidades angulares / PWM
      // 20 segundos girando y aumentando la velocidad, cambiando de sentido
      // permite obtener de forma gráfica la relacion entre valores del PWM
      // y la velocidad angular dada por giróscopo.
      //////////////////////////////////////////////////////////////////////////

      if (estado >= 1040 && estado <= 1043) {
        if (estado == 1040) {
          analogWrite(motorIAvance, 0);
          analogWrite(motorIRetroceso, 0);
          analogWrite(motorDAvance, incPWM);
          analogWrite(motorDRetroceso, 0);
        } else if (estado == 1041) {
          analogWrite(motorIAvance, incPWM);
          analogWrite(motorIRetroceso, 0);
          analogWrite(motorDAvance, 0);
          analogWrite(motorDRetroceso, 0);
        } else if (estado == 1042) {
          analogWrite(motorIAvance, 0);
          analogWrite(motorIRetroceso, 0);
          analogWrite(motorDAvance, 0);
          analogWrite(motorDRetroceso, incPWM);
        } else if (estado == 1043) {
          analogWrite(motorIAvance, 0);
          analogWrite(motorIRetroceso, incPWM);
          analogWrite(motorDAvance, 0);
          analogWrite(motorDRetroceso, 0);
        }
        // Despues de 10 segundos girando y aumentando el PWM en 10, cambia de sentido

        if ((tiempoOmega + 20000) < millis() && incPWM <= 250) {
          pararMotores();
          delay(500);
          tiempoOmega = millis();
          incPWM = incPWM + 10;
        } else if (incPWM > 250 && estado == 1043) {
          estado = 0;
        } else if (incPWM > 250 && (estado >= 1040 && estado < 1043)) {
          //Cambio de sentido de giro
          delay(2000);
          estado = estado + 1;
          incPWM = 150; //Valor inicial del PWM
          tiempoOmega = millis();
          //Serial.println("ESTADO @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@");
          //Serial.println(estado);
          //Serial.println("TIEMPO @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@");
          //Serial.println(tiempoOmega);
        }

        // Imprime la tabla PWM / velocidad angular en un CSV para fácil representación y mmanipulación de datos
        Serial.println(";" + String(omega) + ";" + incPWM + ";" + "###"); //En radianes

        //Filtro paso de baja (para quitar el ruido del giróscopo)
        //y[i] := y[i-1] + α * (x[i] - y[i-1])
        //omegaLP = omegaLP + 0.25 * (omega - omegaLP);
        //Serial.println(":"+ String(omegaLP) + ";" + omegaStr + ";" + incPWM + "###");

      }

      //////////////////////////////////////////////////////////////////////////
      // Estado 1040 y siguientes 1041,1042,1043:
      // Creando la tabla de velocidades angulares / PWM
      // 20 segundos girando y aumentando la velocidad, cambiando de sentido
      // permite obtener de forma gráfica la relacion entre valores del PWM
      // y la velocidad angular dada por giróscopo.
      //////////////////////////////////////////////////////////////////////////



      // Estado 1050, Pruebas PROPORCIONAL sobre VELOCIDAD ANGULAR DEL TYPE_gYROSCOPE
      if (estado == 1050) {
        if ((tiempoEstado1050 + 10000) > millis()) {
          //Control Proporocional
          //auxerrorPWM = omega * 20; //102 es lo que sale en papel
          //errorPWM = (int)auxerrorPWM;
          /*
                    Serial.println("Azimut");
                    Serial.println(String(azimut));  //En radianes
                    Serial.println("Omega");
                    Serial.println(String(omega));  //En radianes
                    Serial.println("errorPWM");
                    Serial.println(errorPWM);  //En radianes
                    Serial.println("###");
          */
          analogWrite(motorIAvance, 240);
          analogWrite(motorIRetroceso, 0);
          analogWrite(motorDAvance, 225 - (int)( omega * 20)) ;//analogWrite(motorDAvance, 240 - outPIDPWM) ; //CONTROL PROPOCIONAL SOBRE UNA DE LAS RUEDAS
          analogWrite(motorDRetroceso, 0);


        } else {
          //Serial.println("Fin de linea!!!!!!!!!!!!!!!!###");
          estado = 0;
          pararMotores();
        }

      }

      // Estado 1060, Pruebas PID sobre el angulo con TYPE_GAME_ORIENTATION_VECTOR, se debe mover en línea recta durante 10 sg

      if (estado == 1060) {
        if ((tiempoEstado1060 + 10000) > millis()) {

          movimiento_linea_recta(225, 215);


        } else {
          //Serial.println("Fin de linea!!!!!!!!!!!!!!!!###");
          estado = 0;
          pararMotores();
        }

      }

      // Fin de Estado 1060

      // Estado 1070, se debe mover 0,5 metro en línea recta

      if (estado >= 1070 && estado <= 1079) {

        if (valores[1] > distanciaEstado1070 - DISTANCIA_CALIBRACION) {


          //movimiento_linea_recta(225,215);

          //Vel =0.15 y omega=0
          movimiento_linea_recta((cmd_PWMIzq(cmd_omegaIzq(velEstado1070, 0))), (cmd_PWMDer(cmd_omegaDer(velEstado1070, 0))));
          /*
                    Serial.println("(cmd_PWMIzq(cmd_omegaIzq(velEsado1070,0))");
                    Serial.println(String((cmd_PWMIzq(cmd_omegaIzq(velEstado1070, 0)))));
                    Serial.println("(cmd_PWMDer(cmd_omegaDer(velEstado1070,0))");
                    Serial.println(String((cmd_PWMDer(cmd_omegaDer(velEstado1070, 0)))));

                    Serial.println("cmd_omegaIzq(velEstado1070,0)");
                    Serial.println(String(cmd_omegaIzq(velEstado1070, 0)));
                    Serial.println("cmd_omegaDer(velEstado1070,0)");
                    Serial.println(String(cmd_omegaDer(velEstado1070, 0)));

                    Serial.println("ditancia");
                    Serial.println(distancia);
                    Serial.println("distanciaEstado1070");
                    Serial.println(distanciaEstado1070);
                    Serial.println("valores[0]");
                    Serial.println(valores[0]);
                    Serial.println("valores[1]");
                    Serial.println(valores[1]);
                    Serial.println("valores[2]");
                    Serial.println(valores[2]);
          */
        } else {
          pararMotores();
          incTiempoEstado1070 = millis() - tiempoEstado1070;
          /*
                    Serial.println("tiempoActual");
                    Serial.println(String(millis()));

                    Serial.println("tiempoEstado1070");
                    Serial.println(tiempoEstado1070);

                    Serial.println("incTiempoEstado1070");
                    Serial.println(incTiempoEstado1070);

                    Serial.println("DISTANCIA_CALIBRACION");
                    Serial.println(DISTANCIA_CALIBRACION);

                    //Serial.println("velocidad");
                    //Serial.println(DISTANCIA_CALIBRACION/incTiempoEstado1070);

                    Serial.println("Fin de linea!!!!!!!!!!!!!!!!###");
                    */
          estado = 0;


        }

      }

      // Fin de Estado 1070

      // Estado 1110, debe girar
      if (estado == 1110) {
        if  (abs(azimutRef - azimut) > 0.02) {

          movimiento_giro(); //AQUI!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!30 abril
          //delay(175);
          //pararMotores();
          //delay(25);
          /*
          Serial.println("azimutRef");
          Serial.println(String(azimutRef));

          Serial.println("azimut");
          Serial.println(String(azimut));

          Serial.println("Sigue girando!!!!!!!!!!!!!!!!###");
          */
          estado = 1110;

        } else {
          /*
          Serial.println("azimutRef");
          Serial.println(String(azimutRef));

          Serial.println("azimut");
          Serial.println(String(azimut));

          Serial.println("Fin de giro!!!!!!!!!!!!!!!!###");
          */
          contadorEstado1110 += 1;
          if (contadorEstado1110 > 5)
            estado = 0;
          pararMotores();

        }

      }

      // Fin de Estado 1070


      // Estado 2010, anda recto tras recibir una orden
      if (estado == 2010) {
        movimiento_linea_recta((cmd_PWMIzq(cmd_omegaIzq(Kvel * cmdVel, 0))), (cmd_PWMDer(cmd_omegaDer(Kvel * cmdVel, 0))));
      }
      //

      if (estado == 0) {
        pararMotores();
      } else if (estado > 1000 && estado < 1030) { //Del estado 1000 al 102o necesita que se paren tambien
        pararMotores();
        analogWrite(motorIAvance, 250);
        analogWrite(motorIRetroceso, 0);
        analogWrite(motorDAvance, 250);
        analogWrite(motorDRetroceso, 0);
      }

      //azimut = 0;

      inputString = "";
      commandComplete = false;
    }

    //Comando de test iiitestXXww###m donde XX indica el número del test
    else if (inputString.startsWith("iiittt")) {

      //Serial.println("Detectó iiitest!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");

      //trozo1Str = inputString.substring(inputString.indexOf("ttt") + 3, inputString.indexOf("ww"));
      //paramTestStr = inputString.substring(inputString.indexOf("ww") + 2, inputString.indexOf("ww###"));
      trozo1Str = inputString.substring(inputString.indexOf("ttt") + 3, inputString.indexOf("ww"));
      trozo2Str = inputString.substring(inputString.indexOf("ww") + 2, inputString.indexOf("ww###"));

      //Serial.println(" ttt " + trozo1Str);

      //Test1: Es avanzar en línea recta medio metro y girar 180 grados.
      if (trozo1Str.toInt() == 1) {
        /*
        if (valores[1] > 200) {
          //Si el sensor de distancia no tiene una pared a menos de dos metros, camina medio metro por tiempo, en función de su experiencia previa

          estado = 1011; //Indico el estado de test 100 + 1
          Serial.println(estado);
          azimutTest = azimut; // Coge  el azimut del momento de recibir una orden como el que debe serguir en la línea recta


          //Determina el tiempo que durara el test para que avance 50 cm, conforme la experiencia previa (incDistTiempoAcc)
          tiempoFinTest = millis();
          auxiliarlong =  (tiempoAccion * 50);
          auxiliarlong =  auxiliarlong / incDistTiempoAcc;
          tiempoFinTest = auxiliarlong + tiempoFinTest;


          //Pongo los motores a moverse
          analogWrite(motorIAvance, 250);
          analogWrite(motorIRetroceso, 0);
          analogWrite(motorDAvance, 250);
          analogWrite(motorDRetroceso, 0);
          //
        } else {
          estado = 1012; //Indico el estado de test 100 + 1 + 2 (uso medida de distancia)
          Serial.println(estado);
          azimutTest = azimut; // Coge  el azimut del momento de recibir una orden como el que debe serguir en la línea recta
          Serial.println("Azimut Test");
          Serial.println(azimutTest);
          //Determina la distanciaInicial
          distanciaTest = valores[1];
          Serial.println("Distancia Test");
          Serial.println(distanciaTest);
          //Pongo los motores a moverse
          analogWrite(motorIAvance, 250);
          analogWrite(motorIRetroceso, 0);
          analogWrite(motorDAvance, 250);
          analogWrite(motorDRetroceso, 0);
        }
        */
      } else if (trozo1Str.toInt() == 2) {
        estado = 1020; // estado de test 1000 ejercicio segundo
        Serial.println(estado);
      } else if (trozo1Str.toInt() == 3) {
        // estado de test que minimiza correlación Wifi
        estado = 1030;
      } else if (trozo1Str.toInt() == 4) {
        //estado que crea tabla w / pwm
        estado = 1040;
        incPWM = 150; //Valor inicial del PWM
        tiempoOmega = millis();
        /*
        Serial.println("ESTADO @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@");
        Serial.println(estado);
        Serial.println("TIEMPO @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@");
        Serial.println(tiempoOmega);
        */

      }
      else if (trozo1Str.toInt() == 5) {
        //estado que realiza un control proporcional en la velocidad angular (Deberia ser PID)
        estado = 1050;

        //Serial.println("ESTADO @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@");
        //Serial.println(estado);
        //Supongo objetivo Omega=0 OJO!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        // OJO al retraso, pues los datos de velocidad angular se generan en el Android y se envían al Arduino,luego hay retraso OJO!!!!!!!!!!!!!!!!!!!!!!!!!

        //Aqui habria que tener tablas o funciones de interpolacion. HAY QUE HACERLO AUTOMATICO, PUES VARÍA MUCHO CON BATERIAS Y SUELO!!!!!!!!!!!!!!!!!

        // De experiencia recogida en hoja de excel.
        // PWMderecho=101,49*w+103,41
        // PWMizquierdo=-111,47*w+132,47

        //Quiero ir a recto, con cada rueda a 1 rad/sg
        //PWMderecho=204
        //PWMizquierdo=243
        // Control derivativo (aprrox. pues no existe una frecuencia de muestreo fija)
        tiempoEstado1050 = millis();
        azimutRef = 100;

        incTErrPWM = 0;
        tErrPWMAnt = 0;
        //errorPWM = 0;
        errorPWMAnterior = 0;
        errorPWMSum = 0;
        errorPWMDiff = 0;
        outPID = 0;
        outPIDPWM = 0;

        analogWrite(motorIAvance, 240); //Valores calculados a mano OJO!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        analogWrite(motorIRetroceso, 0);
        analogWrite(motorDAvance, 230);
        analogWrite(motorDRetroceso, 0);

      }    else if (trozo1Str.toInt() == 6) {
        //estado que realiza un control PID en el ángulo
        estado = 1060;

        //Serial.println("ESTADO @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@");
        //Serial.println(estado);

        tiempoEstado1060 = millis();

        inicia_linea_recta();

      } else if (trozo1Str.toInt() == 7) {
        //estado que realiza un control PID en el ángulo, y se mueve hasta una determinada distancia
        estado = 1070;
        velEstado1070 = Kvel * trozo2Str.toFloat();

        tiempoEstado1070 = millis();
        distanciaEstado1070 = valores[1];

        inicia_linea_recta();

      }  else if (trozo1Str.toInt() == 8) {
        //estado que crea tabla w / pwm. Calibración automática
        estado = 1080;
        incPWM = PWMMIN; //Valor inicial del PWM
        tiempoOmega = millis();
        numOmegas = 0;
        mediaOmega = 0;
        /*
        Serial.println("ESTADO @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@");
        Serial.println(estado);
        Serial.println("TIEMPO @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@");
        Serial.println(tiempoOmega);
        */
      }
      else if (trozo1Str.toInt() == 11) {
        //estado que crea tabla w / pwm. Calibración automática
        estado = 1110;
        tiempoEstado1110 = millis();
        contadorEstado1110 = 0; // Contador de sobreoscilación !!!!!! DEBERIA SER ESO
        //Serial.println("ESTADO @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@");
        //Serial.println(estado);
        inicia_giro(0.33); //VAlor aleatorio de angulo!!! AQUI!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
      }

      inputString = "";
      commandComplete = false;
    }
    //Recibe una orden tipo twist (velocidad lineal / velocidad angular)
    else if (inputString.startsWith("iiikkk")) {
      //Serial.println("Detecta iiiTWIST");
      //cmdVelStr = inputString.substring(inputString.indexOf("kkk") + 3, inputString.indexOf("ww"));
      //cmdOmegaStr = inputString.substring(inputString.indexOf("ww") + 2, inputString.indexOf("ww###"));
      trozo1Str = inputString.substring(inputString.indexOf("kkk") + 3, inputString.indexOf("ww"));
      trozo2Str = inputString.substring(inputString.indexOf("ww") + 2, inputString.indexOf("ww###"));
      cmdVel = trozo1Str.toFloat();
      cmdOmega = trozo2Str.toFloat();
      distancia = 0;
      /*
      Serial.println("cmdVel");
      Serial.println(String(cmdVel));
      Serial.println("cmdOmega");
      Serial.println(String(cmdOmega));
      Serial.println("###");

            Serial.println("cmd_omegaIzq(Kvel*cmdVel, cmdOmega)");
            Serial.println(String(cmd_omegaIzq(Kvel * cmdVel, cmdOmega)));
            Serial.println("cmd_PWMIzq(cmd_omegaIzq(Kvel*cmdVel, cmdOmega)");
            Serial.println(String(cmd_PWMIzq(cmd_omegaIzq(Kvel * cmdVel, cmdOmega))));

            Serial.println("cmd_omegaDer(Kvel*cmdVel, cmdOmega)");
            Serial.println(String(cmd_omegaDer(Kvel * cmdVel, cmdOmega)));
            Serial.println("cmd_PWMDer(cmd_omegaDer(Kvel*cmdVel, cmdOmega)");
            Serial.println(String(cmd_PWMDer(cmd_omegaDer(Kvel * cmdVel, cmdOmega))));
      */
      if (cmdVel == 0 && cmdOmega == 0) {
        pararMotores();
        estado = 0;
      }
      else if (cmdVel != 0 && cmdOmega == 0) {
        //Anda en línea recta
        estado = 2010;
        inicia_linea_recta();
        movimiento_linea_recta((cmd_PWMIzq(cmd_omegaIzq(Kvel * cmdVel, 0.0))), (cmd_PWMDer(cmd_omegaDer(Kvel * cmdVel, 0.0))));
      }
      //Gira en el sitio
      else if (cmdVel == 0 && cmdOmega != 0) {
        estado = 2020;
        /*
        //AQUI!!!! Habría que controlarlo con el valor de omega del giroscopo

        Serial.println("(cmd_omegaIzq(0, cmdOmega))");
        Serial.println((cmd_omegaIzq(0, cmdOmega)));

        Serial.println("(cmd_omegaDer(0, cmdOmega))");
        Serial.println((cmd_omegaDer(0, cmdOmega)));

        Serial.println("(cmd_omegaIzq(0, cmdOmega))");
        Serial.println(cmd_PWMIzq(cmd_omegaIzq(0, cmdOmega)));

        Serial.println("cmd_PWMDer(cmd_omegaDer(0, cmdOmega))");
        Serial.println(cmd_PWMDer(cmd_omegaDer(0, cmdOmega)));

        Serial.println("Debería estar girando###");
        */
        motoresPWM(cmd_PWMIzq(cmd_omegaIzq(0, Komega * cmdOmega)), cmd_PWMDer(cmd_omegaDer(0, Komega * cmdOmega)));


      }

      //Anda haciendo un arco de circunferencia
      else if (cmdVel != 0 && cmdOmega != 0) {
        estado = 2000; //!!!!!!! PRUEBA SIN PID

        //Calcula el radio de curvatura ICC,para estima x,y
        // de www.doc.ic.ac.uk/~ajd/Robotics Lecture 2
        // R=W(vr+vl)/2(vr-vl)
        //ICCR = 0.5 * DISTANCIA_RUEDAS * ( cmd_omegaDer(Kvel * cmdVel, Komega * cmdOmega) + cmd_omegaIzq(Kvel * cmdVel, Komega * cmdOmega) ) / ( cmd_omegaDer(Kvel * cmdVel, Komega * cmdOmega) - cmd_omegaIzq(Kvel * cmdVel, Komega * cmdOmega) );
        ICCR =  DISTANCIA_RUEDAS * ( cmd_omegaDer(Kvel * cmdVel, Komega * cmdOmega) + cmd_omegaIzq(Kvel * cmdVel, Komega * cmdOmega) ) / ( cmd_omegaDer(Kvel * cmdVel, Komega * cmdOmega) - cmd_omegaIzq(Kvel * cmdVel, Komega * cmdOmega) );
        //incOme = ( cmd_omegaDer(Kvel * cmdVel, Komega * cmdOmega) - cmd_omegaIzq(Kvel * cmdVel, Komega * cmdOmega) ) / DISTANCIA_RUEDAS;
        /*
        //Habría que controlarlo con el valor de omega del giroscopo y el tiempo/distancia recorrida
        Serial.println("(cmd_ omegaIzq(Kvel * cmdVel, Komega * cmdOmega))");
        Serial.println((cmd_omegaIzq(Kvel * cmdVel, Komega * cmdOmega)));

        Serial.println("(cmd_omegaDer(Kvel * cmdVel, Komega * cmdOmega))");
        Serial.println((cmd_omegaDer(Kvel * cmdVel, Komega * cmdOmega)));


        Serial.println("cmd_PWMIzq(cmd_omegaIzq(Kvel * cmdVel, Komega * cmdOmega)");
        Serial.println(cmd_PWMIzq(cmd_omegaIzq(Kvel * cmdVel, Komega * cmdOmega)));

        Serial.println("cmd_PWMDer(cmd_omegaDer(Kvel * cmdVel, Komega * cmdOmega)");
        Serial.println(cmd_PWMDer(cmd_omegaDer(Kvel * cmdVel, Komega * cmdOmega)));

        Serial.println("###");
        Serial.println("Debería estar haciendo arco ###");
        */

        motoresPWM(cmd_PWMIzq(cmd_omegaIzq(Kvel * cmdVel, Komega * cmdOmega)), cmd_PWMDer(cmd_omegaDer(Kvel * cmdVel, Komega * cmdOmega)));

        //delay(2000); //Depuracion
      }

      inputString = "";
      commandComplete = false;
    }
    // iiibbbXXXww### donde XXX es la correlación respecto al origen (destino buscado)
    else if (inputString.startsWith("iiibbb")) {

      /* AQUI!!!!!!!!!!!!!!. Parte del LDR Offset por nivel superior
      cmdBeaconStr = inputString.substring(inputString.indexOf("bbb") + 3, inputString.indexOf("ww"));
      beacon = cmdBeaconStr.toInt();


            if (beaconAnterior < 0){
              diffBeacon = 0; // la primera vez, pues no hay beacon anterior. FALTA, ver que pasa si beacon es valor no VALIDO !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
              incOffSet = 0; //random(0, 3) - 1;
            }
            else {
              diffBeacon = beaconAnterior - beacon;
               Serial.println("Beacon");
       Serial.println(beacon);
       Serial.println("Beacon Anterior");
       Serial.println(beaconAnterior);

       Serial.println("diffBeacon");
       Serial.println(diffBeacon);




              if (diffBeacon > 0 && (beacon < 500) && (beacon > 50)) {
                filabeacon = 0;
                //Matriz de "conocimiento" de Variación de Beacons Wifi (Input) y incremento/decremento LDROffset (Output)
                if ((BCOffSet[0] > BCOffSet[1]) && (BCOffSet[0] > BCOffSet[2])) {
                  incOffSet = -1;
                } else if ((BCOffSet[1] > BCOffSet[0]) && (BCOffSet[1] > BCOffSet[2])) {
                  incOffSet = 0;
                } else if ((BCOffSet[2] > BCOffSet[1]) && (BCOffSet[2] > BCOffSet[0])) {
                  incOffSet = 1;
                }
                else {
                  incOffSet = random(0, 3) - 1;
                }
              }
              if (diffBeacon < 0 && (beacon < 500) && (beacon > 50) ) {
                       filabeacon = 6;
                if ((BCOffSet[6] > BCOffSet[7]) && (BCOffSet[6] > BCOffSet[8])) {
                  incOffSet = -1;
                } else if ((BCOffSet[7] > BCOffSet[6]) && (BCOffSet[7] > BCOffSet[8])) {
                  incOffSet = 0;
                } else if ((BCOffSet[8] > BCOffSet[6]) && (BCOffSet[8] > BCOffSet[7])) {
                  incOffSet = 1;
                } else {
                  incOffSet = random(0, 3) - 1;
                }
              }

              if (beacon >= 500) {
                //Si el valor de beacon es muy alto genera valor aleatorio

                incOffSet = random(0, 3) - 1;

              } else if ( beacon < 50) {

                //Si el valor de beacon es muy alto genera valor aleatorio ya hemos llegado.

                incOffSet = 0;

              }




            }
            //Movimiento ????
            Serial.println("LDROffSet Antes");
            Serial.println(LDROffset);

            LDROffset = LDROffset + (20*incOffSet) ;

            Serial.println("LDROffSet Despues");
            Serial.println(LDROffset);

            Serial.println("incOffSet");
            Serial.println(incOffSet);

           //Jucio de valor: Si se redujo el valor del beacon se refuerza el movimiento anterior.
            if (diffBeacon > 0 && (beacon < 500) && (beacon > 50)) {
              auxX = (incOffSet + 1) + (filabeacon);
              BCOffSet[auxX] += 5;
            }

       Serial.println("Matrices Beacon");
            for (i = 0; i < 3; i++) {
              for (j = 0; j < 3; j++) {
                Serial.print(BCOffSet[i + 3 * j], DEC);
                Serial.print(" ");
              } Serial.println(" ");
            }

            beaconAnterior = beacon; // Actualiza el valor para la siguiente vez que reciba un beacon


      */

      //LDROffSet=beacon; // PRUEBA !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
      inputString = "";
      commandComplete = false;

    }


    // Si el mensaje es incorrecto lo tira
    else {
      inputString = "";
      commandComplete = false;
    }




  }
  //
  // Fin de RX debería ser más limpio y sólo codificar el estado al recibir un mensaje.
  //

  //
  // Ejecución por estado
  //

  switch (estado) {
    case 0:
      //do something when var equals 1
      break;
    case 1011:
      //Esta andando durante un metro, si llega al tiempo final lo paro
      // Eliminado
      break;
    case 1012:
      //Esta andando durante un metro, si a un metro o a un obstáculo se para
      // Eliminado
      break;
    case 1020:
      if (valores[2] < 250 && valores[2] < 250) {
        if (abs(valores[0] - valores[2]) < 20) {
          pararMotores();
          estado = 0;
        } else if (valores[0] > valores[2]) {
          pararMotores();
          analogWrite(motorIAvance, 0);
          analogWrite(motorIRetroceso, 250);
          analogWrite(motorDAvance, 250);
          analogWrite(motorDRetroceso, 0);
          //for (i = 0; i < errorAzimutGrados/36; i++)
          delay(100);
          pararMotores();
        } else {
          pararMotores();
          analogWrite(motorIAvance, 250);
          analogWrite(motorIRetroceso, 0);
          analogWrite(motorDAvance, 0);
          analogWrite(motorDRetroceso, 250);
          //for (i = 0; i < errorAzimutGrados/36; i++)
          delay(100);
          pararMotores();
        }
      }
      //delay(1000);

      break;

    // 1030 - ejecuta el algoritmo de 2010 !!!
    case 1030:
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
        //Programa del 2010
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

        } else if ( abs(LDRdiff) <= 30 and abs(LDRmedia) <= 500) {

          //Si está "orientado" a la luz avanza
          LDRsigno = 0;
          fila = 3;
          moveMI = 2; //0
          moveMD = 2; //0
        }

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

          delay(600);
        }



        //Se detiene
        analogWrite(motorIAvance, 0);
        analogWrite(motorIRetroceso, 0);
        analogWrite(motorDAvance, 0);
        analogWrite(motorDRetroceso, 0);

        //ELIMINAR. Sólo para verificar
        delay(2000);

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
        //Fin del programa de 2010
      */
      break;

    case 1040:
      // Representacion w (omega) / PWM. Derecha adelate.
      break;
    case 1041:
      // Representacion w (omega) / PWM. Izquierda adelante.
      break;
    case 1050:
      // Intentando andar en línea recta, con control proporcional en la velocidad angular
      break;
    case 1060:
      // Intentando andar en línea recta, con control PID en el ángulo
      break;
    case 1070:

      // Intentando andar en línea recta, con control PID en el ángulo

      if (valores[1] > distanciaEstado1070 - DISTANCIA_CALIBRACION) {

        /*

                Serial.println("ditancia");
                Serial.println(distancia);
                Serial.println("distanciaEstado1070");
                Serial.println(distanciaEstado1070);
                Serial.println("valores[0]");
                Serial.println(valores[0]);
                Serial.println("valores[1]");
                Serial.println(valores[1]);
                Serial.println("valores[2]");
                Serial.println(valores[2]);
        */
      }

      else {
        pararMotores();
        incTiempoEstado1070 = millis() - tiempoEstado1070;
        /*
              Serial.println("tiempoActual");
              Serial.println(String(millis()));

              Serial.println("tiempoEstado1070");
              Serial.println(tiempoEstado1070);

              Serial.println("incTiempoEstado1070");
              Serial.println(incTiempoEstado1070);

              Serial.println("DISTANCIA_CALIBRACION");
              Serial.println(DISTANCIA_CALIBRACION);


              Serial.println("ditancia");
              Serial.println(distancia);
              Serial.println("distanciaEstado1070");
              Serial.println(distanciaEstado1070);
              Serial.println("valores[0]");
              Serial.println(valores[0]);
              Serial.println("valores[1]");
              Serial.println(valores[1]);
              Serial.println("valores[2]");
              Serial.println(valores[2]);

              Serial.println("velocidad");
              Serial.println(String(DISTANCIA_CALIBRACION / incTiempoEstado1070));





              Serial.println("Fin de linea!!!!!!!!!!!!!!!!###");
              */

        estado = 0;


      }
      break;

    case 1080:
      // Calculando automaticamente los valores de la linea PWM velocidad angular
      break;

    case 1110:
      // Se orienta a un angulo
      break;

    default:
      break;
  }

  // Fin de red de petri de ejecución



}



void pararMotores() {
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
  // = 1, si motor derecho esta por debajo de PWMMIN
  // = 2, si motor derecho está por enciam de PWMMAX
  // = 10, si motor izquierdo esta por debajo de PWMMIN
  // = 20, si motor izquierdo está por enciam de PWMMAX

  if (abs(motor_derecho) > PWMMAX) {
    motor_derecho = PWMMAX * (motor_derecho / abs(motor_derecho));
    saturacion += 2;
  } else if (abs(motor_derecho) < PWMMIN) {
    motor_derecho = 0 ;
    saturacion += 1;
  }

  if (abs(motor_izquierdo) > PWMMAX) {
    motor_izquierdo = PWMMAX * (motor_izquierdo / abs(motor_izquierdo));
    saturacion += 20;
  } else if (abs(motor_izquierdo) < PWMMIN) {
    motor_izquierdo = 0 ;
    saturacion += 10;
  }


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

  return saturacion;
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


//Cinemática inversa uniciclo (PENDIENTE VALIDAR !!!!!!!!!!!!!!!!!!!!!. ESPECIALMENTE EL SIGNO DE cmd_ang!!!)
float cmd_omegaDer(float cmd_vel, float cmd_ang) {
  // FALTA DEFINIR LOS LÍMITES DE VELOCIDADES MÁXIMOS y MINIMOS !!!!!!!!!!!!!!!!!!
  //return (cmd_vel + (DISTANCIA_RUEDAS*cmd_ang/2))/RADIO_RUEDA;
  //return (cmd_vel/7 + (DISTANCIA_RUEDAS * cmd_ang / 2)) / ( RADIO_RUEDA); //Para línea recta OK, con Kvel=1
  //return (cmd_vel  + (DISTANCIA_RUEDAS * cmd_ang / 2)) / (80 * RADIO_RUEDA);
  return (cmd_vel  + (DISTANCIA_RUEDAS * cmd_ang / 2)) / (8.34 * RADIO_RUEDA);
}
float cmd_omegaIzq(float cmd_vel, float cmd_ang) {
  //return (cmd_vel - (DISTANCIA_RUEDAS*cmd_ang/2))/RADIO_RUEDA;
  //return (cmd_vel/7  - (DISTANCIA_RUEDAS * cmd_ang / 2)) / ( RADIO_RUEDA); //Para línea recta OK, co KVel=1. No considera omega
  //return (cmd_vel - (DISTANCIA_RUEDAS * cmd_ang / 2)) / (80 * RADIO_RUEDA);
  return (cmd_vel - (DISTANCIA_RUEDAS * cmd_ang / 2)) / (8.34 * RADIO_RUEDA);
}


//Relación omea / PWM (PEnDIENTE DE CAMBIAR VALORES FIJOS POR CALIBRACION !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!)
int cmd_PWMDer(float omegaDer) {

  if (abs(omegaDer) > omegaMAX) {
    omegaDer = omegaMAX * (omegaDer / abs(omegaDer));
  }
  if (abs(omegaDer) < omegaMIN) {
    omegaDer = omegaMIN * (omegaDer / abs(omegaDer));
  }


  return (int)((101.5 * abs(omegaDer)) + 103.41) * (omegaDer / abs(omegaDer));
}

int cmd_PWMIzq(float omegaIzq) {

  if (abs(omegaIzq) > omegaIzq) {
    omegaIzq = omegaMAX * (omegaIzq / abs(omegaIzq));
  }
  if (abs(omegaIzq) < omegaMIN) {
    omegaIzq = omegaMIN * (omegaIzq / abs(omegaIzq));
  }

  //return (-111.5*omegaIzq) + 132.41;
  //return (111.5*omegaIzq) + 132.41;

  return (int)((111.5 * abs(omegaIzq)) + 112.41) * (omegaIzq / abs(omegaIzq));
}

int movimiento_linea_recta(int PWM_I, int PWM_D) {

  //int PWM_I = 225;
  //int PWM_D = 215;

  outPIDPWM = pid(); //Ejecuta el PID (error en ángulo!!!), por ahora solo proporcional!!!!!!!

  PWM_I = PWM_I + outPIDPWM;
  PWM_D = PWM_D - outPIDPWM;

  //SATURACION.OJO PARAR INTEGRAL del  PID!!!!!!!!!!!!!!!!!
  motoresPWM(PWM_I, PWM_D);

  /*
  Serial.println("PWM_I");
  Serial.println(String(PWM_I));

  Serial.println("PWM_D");
  Serial.println(String(PWM_D));

  Serial.println("###");
   */
}

int inicia_linea_recta() {

  incTErrPWM = 0;
  tErrPWMAnt = 0;
  //errorPWM = 0;
  errorPWMAnterior = 0;
  errorPWMSum = 0;
  errorPWMDiff = 0;
  outPID = 0;
  outPIDPWM = 0;


  azimutRef = azimut;
  azimutAnterior = azimut;
  distancia = 0;
  errorPWMAnterior = 0;
  tErrPWMAnt = millis(); //

  errorPWMSum = 0;
  errorPWMDiff = 0;

  /*
    Serial.println("Azimut");
    Serial.println(azimut);
    Serial.println("Definiendo la referencia de Azimut !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5");
    Serial.println(azimutRef);
    Serial.println("###");
  */

  //Arranque de motores
  analogWrite(motorIAvance, 250); //Valores calculados a mano OJO!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  analogWrite(motorIRetroceso, 0);
  analogWrite(motorDAvance, 240);
  analogWrite(motorDRetroceso, 0);
}

int movimiento_giro() {

  int PWM_I = 225;
  int PWM_D = 215;

  outPIDPWM = pid(); //Ejecuta el PID (error en ángulo!!!), y por ahora solo proporcional!!!!!!!

  PWM_I =  (outPIDPWM) + (PWMMIN * (outPIDPWM / abs(outPIDPWM)));
  PWM_D =  (outPIDPWM * (-1)) - (PWMMIN * (outPIDPWM / abs(outPIDPWM)));

  //SATURACION.OJO PARAR INTEGRAL del  PID!!!!!!!!!!!!!!!!!
  motoresPWM(PWM_I, PWM_D);


  /*
    Serial.println("PWM_I");
    Serial.println(String(PWM_I));

    Serial.println("PWM_D");
    Serial.println(String(PWM_D));

    Serial.println("azimutRef");
    Serial.println(String(azimutRef));

    Serial.println("azimut");
    Serial.println(azimut);


    Serial.println("###");
  */

}

int inicia_giro(float azimutGira) {

  //Falta controlar que azimutGira esté entre -pi y pi FALTA!!!!!!!!!!!!!!!!!!!!!!!!!!!

  azimutRef = azimutGira; // Establece la referencia del azimut
  incTErrPWM = 0;
  tErrPWMAnt = 0;
  // errorPWM = 0;
  errorPWMAnterior = 0;
  errorPWMSum = 0;
  errorPWMDiff = 0;
  outPID = 0;
  outPIDPWM = 0;

  azimutAnterior = azimut;
  distancia = 0;
  errorPWMAnterior = 0;
  tErrPWMAnt = millis(); //

  errorPWMSum = 0;
  errorPWMDiff = 0;

  /*
   Serial.println("Azimut");
   Serial.println(azimut);
   Serial.println("Definiendo la referencia de Azimut !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5");
   Serial.println(azimutRef);
   Serial.println("###");
  */

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
}


int pid() {
  int kp = 80; //Por el momento sólo PROPORCIONAL
  int ki = 0;
  int kd = 0;

  //P
  errorAzimut = azimutRef - azimut; //Redefine el error
  //Serial.println("Error azimut");
  //Serial.println(String(errorAzimut));

  errorAzimut = (float)  atan2((double)sin(errorAzimut), (double)cos(errorAzimut));

  //Serial.println("Error entre pi y pi");
  //Serial.println(String(errorAzimut));

  //I
  tErrPWM = millis();
  incTErrPWM = (float)(tErrPWM - tErrPWMAnt);
  //errorPWMSum += ((errorPWM * incTErrPWM) / 1000);

  //D
  //errorPWMDiff = 1000 * (errorPWM - errorPWMAnterior) / incTErrPWM;



  //outPID = kp * errorAzimut + ki * errorPWMSum + kd * errorPWMDiff; //PID SOBRE VELOCIDAD ANGULAR
  outPID = kp * errorAzimut;


  azimutAnterior = azimut;
  // 0.135 es la velocidad en metros por segundo paraPWM = 225 (sacada linealmente de extramolar entre 255=0.17m/sg y 200=0.1m/sg)
  //distancia += (0.135 * incTErrPWM) / 1000.0;

  /*
  Serial.println("Azimut");
  Serial.println(String(azimut));

  Serial.println("AzimutRef");
  Serial.println(String(azimutRef));

  Serial.println("Error azimut");
  Serial.println(String(errorAzimut));

  Serial.println("Error entre pi y pi");
  Serial.println(String(errorAzimut));

  Serial.println("errorPWMSum");
  Serial.println(errorPWMSum);

  Serial.println("errorPWMDiff");
  Serial.println(errorPWMDiff);

  Serial.println("outPIDPWM");
  Serial.println(outPIDPWM);

  Serial.println("distancia");
  Serial.println(distancia);


  Serial.println("tErrPWM");
  Serial.println(String(tErrPWM));


  Serial.println("incTErrPWM");
  Serial.println(String(incTErrPWM));

  */

  //Actualiza los valores para la siguiente vuelta
  //tErrPWMAnt =  tErrPWM;
  //errorPWMAnterior = errorPWM;

  return (int)((outPID));

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
  while (Serial.available()) {

    char inChar = (char)Serial.read();

    inputString += inChar;

    String finCadena = "###";
    //Comienza por iii (entrada desde el Android al Arduino) y termina con ### (fin de mensaje)
    if (inputString.endsWith("###") || (inputString.indexOf("###") > 1) ) {
      commandComplete = true;
    }

  }

}



