/*************************************************************************************************
 * Arduino Firmware for Andruino R2 ROS robot
 * Version:  99
 * Date: 2017-04-19
 * Autors: Paco López, dirigido por Federico Cuesta
 **************************************************************************************************/

#include <math.h>


// Constantes a convertir en parámetros si es necesario

#define DISTANCIA_RUEDAS 0.1490
#define RADIO_RUEDA 0.01785

#define SENSIBLE_US 10 
#define DISTANCIA_CALIBRACION 500 


#define omegaMIN 1.5 //Cuando realiza un giro puro
#define omegaMAX 2.5

#define velMAX 0.2 // Obtenida de repetir el test 7 para diferentes velocidades
#define velMIN 0.1 //0.14

#define Kp1050 40 //Constate proprocional del PD del test 5

#define TEMPOTX 100 //Transmite cada 100 milisegundos.

//////////////////////////////////////////////////////////////////////////
// Pines de Arduino (Placa Andruino R2)
//////////////////////////////////////////////////////////////////////////

// Sensores Utrasonidos
#define trigPin1 11
#define echoPin1 12
#define trigPin2 8
#define echoPin2 7
#define trigPin3 4
#define echoPin3 2

// LDRs
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
float azimut = 0;
float omega = 0;

float x = 0;
float y = 0;
float azimut_odometry = 0;
float azimut_odometry_iiikkk = 0;
//////////////////////////////////////////////////////////////////////////
//Valores de sensores
//////////////////////////////////////////////////////////////////////////

int valores[10];

int incPWM = 0;
unsigned long tiempoOmega = 0; //tiempoEstado1080
float mediaOmega = 0;
unsigned long  tiempoEstado1040 = 0;

// Variables Estado 1080
// Representación gráfica (lineal) PWM-Velocidad Angular
float PWM_Omega[8]; // Valores de velocidad angular máximo y mínimos, Orden: D_avanza, I_avanza, D_retrocede, I_retrocede
int numOmegas;      // Numero de omegas para promediar

// Variables Estado 1050, pid en velocidad angular
unsigned long  tiempoEstado1050 = 0;

float outPID = 0;
int outPIDPWM = 0;

//PID
float errorAzimut = 0; //error P
float errorAzimutD = 0;  //error D
float errorAzimutI = 0; //error I
float errorAzimutAnterior = 0;
unsigned long tiempoAzimutAnterior;
int estadoPID = -1;
int salPID;

//PD
int outPD;

//Variables de azimut relaciondas con odometria
float timeAzimut = 0;
float dtAzimut = 0;
float incAzimut = 0;

//PID_arco
float omega_estimada;
float errorOmega;
int  outPID_arco;

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

//Variables -estao 1100
float omegaLP;

//Variables Estado 11 (1110), prueba la función de giro
unsigned long tiempoEstado1110 = 0;
int contadorEstado1110 = 0; //Contador de estado de sobreoscilaciones del PD de giro


//Variables Estado 14 (1140), describe media circunferencia
unsigned long tiempoEstado1140 = 0;
float azimutEstado1140;

//Variables de la recepcion de comando en formato Twist
float cmdVel;
float cmdVelAnterior = 0; // En caso de que reciba orden con el mismo valor no actuliza AzimutRef, manteniendo el anterior
float cmdOmega;
float cmdAzimut;
float ICCR; // Radio de curvatura
float incOme; //Incremento de omega/dt, estimado por las velocidades angulesy la geometria del robot.

//Variabes de l Sensor Wifi
int beacon = 0;
int beaconAnterior = -1;
int diffBeacon = 0;
int incOffSet = 0;
int filabeacon = 0;
int LDROffset = 0;

//Cadenas de entrada de datos
String inputString = "";
String trozo1Str = "";
String trozo2Str = "";
String trozo3Str = "";

// Flag de cadena recibida
boolean commandComplete = false;

// VAriables globales de Test3
int value = 0;
//Movimientos del motor izquierdo y derecho
int moveMI = 0;
int moveMD = 0;
//Valores de los sensores, LDRdiff y LDRmedia
int LDRIzq = 0; //sustituir por valores (3,4,5) !!!!
int LDRCen = 0;
int LDRDer = 0;
int LDRdiff = 0; //diferencia antes del ensayo
//int LDRsigno = 0;
int LDRdiffpost = 0; // diferencia después del ensayo
int LDRmedia = 0; //media antes del ensayo
int LDRmediapost = 0; // media despues del ensayo
int LDRIzqpost = 0;
int LDRDerpost = 0;
int LDRCenpost = 0;

int USIzq = 0; //sustituir por valores (3,4,5) !!!!
int USCen = 0;
int USDer = 0;
int USdiff = 0; //diferencia antes del ensayo
//int LDRsigno = 0;
int USdiffpost = 0; // diferencia después del ensayo
int USmedia = 0; //media antes del ensayo
int USmediapost = 0; // media despues del ensayo
int USIzqpost = 0;
int USDerpost = 0;
int USCenpost = 0;
int USOffset = 0;

int fila; // Fila de la matriz

int BCMD[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0}; //Matrices sustituidas por vectores por bug en compilador
int BCMI[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

int auxX; //sutituir por i
int auxY; //sustitir por j
int i;
int j;
//Fin de  Test 3

int indice = -1;


//////////////////////////////////////////////////////////////////////////
// Variables de Tiempo d ejecución de loop principal
//////////////////////////////////////////////////////////////////////////
unsigned long timeLoop; //Tiempo de ciclo
unsigned long dtLoop;
unsigned long timeTx; //Tiempo de transmision, para evitar que sea en cada ciclo (y así bajar consumo de CPU en Android)
unsigned long dtTx;

//////////////////////////////////////////////////////////////////////////
// Variables para cálculo de la distancia en tiempo de ejecución en movimiento
// en línea recta, basada en sensor central de ultrasonidos
//////////////////////////////////////////////////////////////////////////
float distanciaUS = -1.0;
float distanciaUS_inicial = -1.0;
float distanciaUS_anterior = -1.0;

//////////////////////////////////////////////////////////////////////////
// Setup
//////////////////////////////////////////////////////////////////////////

void setup() {

  timeLoop = millis();
  timeTx = millis();

  // Inicia comunicación serie
  Serial.begin(115200);//Serial.begin(9600);
  inputString.reserve(300);
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

  // Inicia parametros
  //1-2 Factores de correción entre la velocidad dada en el comando y la que se debe aplicar para que en la práctica se obtenga la deseada en el robot real
  parameters[0] = 1.1; //Kvel = 1.1; //1.089 //vel_lineal=parameters[0]*(omega_Der+omega_Izq), valor encontrado experimentalmente en test 7
  parameters[1] = 1.0; //Komega = 1.0;

  //3-5 Parámetros del modelo cinemático
  parameters[2] = 8.34; //Kssr = 8.34; //Modificacion del radio de las ruedas, para hacer equivalente el modelo Skid Steer al Diferencial
  parameters[3] = 1.65;//Kssb = 1.65; //Modificacion de la distancia entre ruedas, para hacer equivalente el modelo Skid Steer al Diferencial
  parameters[4] = 1; //KssIccr = 1; //Modificacion del radio de curvatura, para hacer equivalente el modelo Skid Steer al Diferencial

  //6-9 Parámetros de la línea entre PWM y omega
  parameters[5] = 111.5; //SlopeLeft = 111.5; // Pendiente de la relación entre PWM y velodidad angular de rueda izquierda
  parameters[6] = 112.41; //OffsetLeft = 112.41; // Offset de la relación entre PWM y velodidad angular de rueda izquierda
  parameters[7] = 101.5; //SlopeRight = 101.5;
  parameters[8] = 103.41; //OffsetRighet = 103.41;

  //10-12 Parámetros de los PID, para movimiento en línea recta, giro y movimiento en curva

  parameters[9] = 200.0; //100.0; //KpLine = 100; //80; //80 Ajustado experimentalmente
  parameters[10] = 3.0; //KiLine = 3;
  parameters[11] = 0.0; //KdLine = 0 ;

  //13-15 Parámetros de los PID, para movimiento en  giro
  parameters[12] = 30.0; //KpSpin = 30;
  parameters[13] = 0.0 ; //KiSpin = 0;
  parameters[14] = 0.0; //KdSpin = 0;

  //16-18 Parámetros de los PID, para movimiento en arco
  parameters[15] = 10.0; //KpArc = 10;
  parameters[16] = 0.0; //KiArc = 0;
  parameters[17] = 0.0; //KdArc = 0;

  //19-20 Parámetros de los valores de PWM máximos y mínimos que hacen que se mueva el robot

  parameters[18] = 255.0; //PWMAX = 255;
  parameters[19] = 155.0; //PWMMIN = 155;

  //int PWMMEDIO = (((int)parameters[18]) + ((int)parameters[19])) / 2;

  // Inicia el estado
  estado = 0; //Estado normal esperando algún comando.
  x = 0;
  y = 0;
  azimut_odometry = 0; //Valor inicial

  // Inicia matrix de conocimiento de test 3 (estado 1030)

  for (j = 0; j < 3; j++) {
    for (i = 0; i < 3; i++) {

      BCMI[i + (3 * j)] = 0;
      BCMD[i + (3 * j)] = 0;
    }

  }

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
    Serial.print("###"); // Fin de mensaje
    Serial.println("");

    timeTx = millis();

  }
  //}

  /////////////////////////////////////////////////////////////
  // RX datos desde Android.
  /////////////////////////////////////////////////////////////

  if (commandComplete) {

    if (inputString.startsWith("iiimmm")) {

      motoresPWM(inputString.substring(inputString.indexOf("mmm") + 3, inputString.indexOf("ww")).toInt(), inputString.substring(inputString.indexOf("ww") + 2, inputString.indexOf("ww###")).toInt());
      delay(3000);
      pararMotores();
      estado = 0;

      // clear the string:
      inputString = "";
      commandComplete = false;
    }

    //VSA
    else if (inputString.startsWith("iiiqqq")) {

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
            analogWrite(motorIAvance, 0);
            analogWrite(motorIRetroceso, 250);
            analogWrite(motorDAvance, 250);
            analogWrite(motorDRetroceso, 0);
            break;
          case 2:
            analogWrite(motorIAvance, 250);
            analogWrite(motorIRetroceso, 0);
            analogWrite(motorDAvance, 0);
            analogWrite(motorDRetroceso, 250);
            break;
          case 3:
            analogWrite(motorIAvance, 250);
            analogWrite(motorIRetroceso, 0);
            analogWrite(motorDAvance, 250);
            analogWrite(motorDRetroceso, 0);
            break;
          case 4:
            analogWrite(motorIAvance, 0);
            analogWrite(motorIRetroceso, 250);
            analogWrite(motorDAvance, 0);
            analogWrite(motorDRetroceso, 250);
            break;
          case 5:
            analogWrite(motorIAvance, 250);
            analogWrite(motorIRetroceso, 0 );
            analogWrite(motorDAvance, 0);
            analogWrite(motorDRetroceso, 0);
            break;
          case 6:
            analogWrite(motorIAvance, 0);
            analogWrite(motorIRetroceso, 250);
            analogWrite(motorDAvance, 0);
            analogWrite(motorDRetroceso, 0);
            break;
          case 7:
            analogWrite(motorIAvance, 0);
            analogWrite(motorIRetroceso, 0);
            analogWrite(motorDAvance, 250);
            analogWrite(motorDRetroceso, 0);
            break;
          case 8:
            analogWrite(motorIAvance, 0);
            analogWrite(motorIRetroceso, 0);
            analogWrite(motorDAvance, 0);
            analogWrite(motorDRetroceso, 250);
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
        //delay(50); //Cambiar por comparacion con timer para que no pare la recepcion de órdenes!!!!

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
        Serial.print("oooccc" + String(LDRIzq) + "ww" + String(LDRCen) + "ww" + String(LDRDer) + "ww" +   String(LDRIzqpost) + "ww" + String(LDRCenpost) + "ww" + String(LDRDerpost) + "ww");
        Serial.print(String(USIzq) + "ww" + String(USCen) + "ww" + String(USDer) + "ww" +   String(USIzqpost) + "ww" + String(USCenpost) + "ww" + String(USDerpost) + "ww");
        Serial.print ( trozo1Str +  "ww" + "###");
        Serial.println("");

      }

      // clear the string:
      inputString = "";
      commandComplete = false;
    }
    //FIN DE VSA

    else if (inputString.startsWith("iiiaaa")) {

      trozo1Str = inputString.substring(inputString.indexOf("aaa") + 3, inputString.indexOf("ww"));
      trozo2Str = inputString.substring(inputString.indexOf("ww") + 2, inputString.indexOf("ww###"));

      //Calcula el incremento del azimut respecto al valor anterior, antes de refrescar la variable azimut. Y los valores del incremento del tiempo y del tiempo
      incAzimut = trozo1Str.toFloat() - (azimut * 10000.0);
      dtAzimut = millis() - timeAzimut;
      timeAzimut = millis();

      //Nuevo valores de azimut y omega
      azimut = trozo1Str.toFloat() / 10000.0;
      omega = trozo2Str.toFloat() / 10000.0;

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

        // Despues de 10 segundos girando y aumentando el PWM entre la diferencia de ((int)parameters[18]) y ((int)parameters[19]), cambia de sentido

        if (abs(omega) > 0.02) {
          numOmegas += 1;
          mediaOmega += omega;
        }


        if ((tiempoOmega + 20000) < millis() && incPWM <= ((int)parameters[18])) {
          int aux;
          pararMotores();
          delay(500);
          tiempoOmega = millis();
          aux = 2 * (estado - 1080) + ((incPWM - ((int)parameters[19])) / (((int)parameters[18]) - ((int)parameters[19])));
          PWM_Omega[aux] = (float)(mediaOmega / numOmegas);
          //Serial.println(";" + String(aux) + ";" + incPWM + ";" + String(PWM_Omega[aux]) + ";" + String(mediaOmega) + ";" + String(numOmegas)  + ";" + "###");
          Serial.println("oooccc" + String(aux) + "ww" + incPWM + "ww" + String(PWM_Omega[aux]) + "ww" + String(mediaOmega) + "ww" + String(numOmegas)  + "ww" + "###");
          mediaOmega = 0;
          numOmegas = 0;
          incPWM = incPWM + (((int)parameters[18]) - ((int)parameters[19]));
        } else if (incPWM > ((int)parameters[18]) && estado == 1083) {
          estado = 0;
        } else if (incPWM > ((int)parameters[18]) && (estado >= 1080 && estado < 1083)) {
          //Cambio de sentido de giro
          delay(2000);
          estado = estado + 1;
          mediaOmega = 0;
          numOmegas = 0;
          incPWM = ((int)parameters[19]); //Valor inicial del PWM
          tiempoOmega = millis();
        }

        // Imprime la tabla PWM / velocidad angular en un CSV para fácil representación y mmanipulación de datos
        Serial.println("oooccc" + String(omega) + "ww" + incPWM + "ww"  + "###");

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

        if ((tiempoOmega + 5000) < millis() && incPWM <= ((int)parameters[18])) {
          //pararMotores();
          //delay(500);
          tiempoOmega = millis();
          incPWM = incPWM + 10;
        } else if (incPWM > ((int)parameters[18]) && estado == 1043) {
          estado = 0;
          pararMotores();
        } else if (incPWM > ((int)parameters[18]) && (estado >= 1040 && estado < 1043)) {
          //Cambio de sentido de giro
          pararMotores();
          delay(500);
          estado = estado + 1;
          incPWM = ((int)parameters[19]); //Valor inicial del PWM
          tiempoOmega = millis();
          tiempoEstado1040 = millis() ;
          omegaLP = 0;
          //Arranque de motores
          if (estado == 1040) {
            analogWrite(motorIAvance, 0);
            analogWrite(motorIRetroceso, 0);
            analogWrite(motorDAvance, 255);
            analogWrite(motorDRetroceso, 0);
          } else if (estado == 1041) {
            analogWrite(motorIAvance, 255);
            analogWrite(motorIRetroceso, 0);
            analogWrite(motorDAvance, 0);
            analogWrite(motorDRetroceso, 0);
          } else if (estado == 1042) {
            analogWrite(motorIAvance, 0);
            analogWrite(motorIRetroceso, 0);
            analogWrite(motorDAvance, 0);
            analogWrite(motorDRetroceso, 255);
          } else if (estado == 1043) {
            analogWrite(motorIAvance, 0);
            analogWrite(motorIRetroceso, 255);
            analogWrite(motorDAvance, 0);
            analogWrite(motorDRetroceso, 0);
          }

        }
        //Filtro paso de baja (para quitar el ruido del giróscopo)
        if (omegaLP == 0)
          omegaLP = omega;
        omegaLP = omegaLP + 0.1 * (omega - omegaLP);

        // Imprime la tabla PWM / velocidad angular en un CSV para fácil representación y mmanipulación de datos.
        // Solo valores válidos después tres segundo espues del inicio(elimina transitorios) y tres segundos antes del fin
        if ((millis() > (tiempoOmega + 500)) && (millis() < tiempoOmega + 4500)   ) {
          Serial.println("oooccc" + String(omegaLP) + "ww" + incPWM + "ww" + String(millis()) + "ww" + String(omega) + "ww###"); //En radianes
        }



      }

      //////////////////////////////////////////////////////////////////////////
      // FIN Estado 1040 y siguientes 1041,1042,1043:
      // Creando la tabla de velocidades angulares / PWM
      // 20 segundos girando y aumentando la velocidad, cambiando de sentido
      // permite obtener de forma gráfica la relacion entre valores del PWM
      // y la velocidad angular dada por giróscopo.
      //////////////////////////////////////////////////////////////////////////


      //////////////////////////////////////////////////////////////////////////
      // Estado 1100 y siguientes 1101,1102,1103....:
      // Trata de calcular la acelararcion angular máxima pasando de PWMe 180 a PWM de 250
      // y viendo el tiempo
      //////////////////////////////////////////////////////////////////////////

      if (estado >= 1100 && estado <= 1101) {
        if (estado == 1100) {
          incPWM = 180;
          analogWrite(motorIAvance, 0);
          analogWrite(motorIRetroceso, 0);
          analogWrite(motorDAvance, incPWM);
          analogWrite(motorDRetroceso, 0);
        } else if (estado == 1101) {
          incPWM = 250;
          analogWrite(motorIAvance, 0);
          analogWrite(motorIRetroceso, 0);
          analogWrite(motorDAvance, incPWM);
          analogWrite(motorDRetroceso, 0);
        }


        if ((tiempoOmega + 20000) < millis()) {
          estado = estado + 1;
          tiempoOmega = millis();
        }
        //
        if (estado >= 1102) {
          pararMotores();
          delay(500);
          estado = 0;

        }

        //Filtro paso de baja (para quitar el ruido del giróscopo)
        //y[i] := y[i-1] + α * (x[i] - y[i-1])
        omegaLP = omegaLP + 0.25 * (omega - omegaLP);

        // Imprime la tabla PWM / velocidad angular en un CSV para fácil representación y mmanipulación de datos.
        Serial.println("oooccc" + String(omega) + "ww" + incPWM + "ww" + String(millis()) + "ww" + tiempoOmega + "ww" +  String(omegaLP) + "ww###"); //En radianes


      }

      //////////////////////////////////////////////////////////////////////////
      // Estado 1050, Pruebas PROPORCIONAL sobre VELOCIDAD ANGULAR DEL TYPE_gYROSCOPE
      //////////////////////////////////////////////////////////////////////////

      if (estado == 1050) {
        if ((tiempoEstado1050 + 10000) > millis()) {
          //Control Proporocional

          analogWrite(motorIAvance, 225 + (int)( omega * Kp1050));
          analogWrite(motorIRetroceso, 0);
          analogWrite(motorDAvance, 225 - (int)( omega * Kp1050)) ; //CONTROL PROPOCIONAL SOBRE UNA DE LAS RUEDAS
          analogWrite(motorDRetroceso, 0);

        } else {
          estado = 0;
          pararMotores();
        }

      }

      // Estado 1060, Pruebas PID sobre el angulo con TYPE_GAME_ORIENTATION_VECTOR, se debe mover en línea recta durante 10 sg

      if (estado == 1060) {
        //Durante dos segundos
        if ((tiempoEstado1060 + 2000) > millis()) {
          //OJO ! Velocidad fija para 0.15 m/Sg
          cmdVel = 0.15 * parameters[0];
          //movimiento_linea_recta(225, 215); //Valores a la mitad del rango PWM posible de 180 a 255. Haciendo Wd=-Wd para que camine recto y usando las ecuaciones
          movimiento_linea_recta((cmd_PWMIzq(cmd_omegaIzq(cmdVel, 0))), (cmd_PWMDer(cmd_omegaDer(cmdVel, 0))));
          // lineales que relacionan el w con el PWM, sale que para que ande recto debe ser PWM_I=225 y PWM_D=215

          azimut_odometry_iiikkk += incAzimut;

          //Serial.print("oooccc" + String(azimut) + "ww"  + String(millis() - tiempoEstado1060) + "ww"  + String(azimutRef) + "ww" + String(estadoPID) +  "ww" + String(salPID) + "ww" + String(errorAzimut) +  "ww" + String(errorAzimutI) + "ww" + String(errorAzimutD) + "ww"); //En radianes
          Serial.print("oooccc");
          Serial.print(azimut, 4);
          Serial.print ("ww"  + String(millis() - tiempoEstado1060) + "ww");
          Serial.print (azimutRef, 4);
          Serial.print ("ww" + String(estadoPID) +  "ww" + String(salPID) + "ww" + String(errorAzimut) +  "ww" + String(errorAzimutI) + "ww" + String(errorAzimutD) + "ww"); //En radianes
          Serial.print(incAzimut, 4);
          Serial.print("ww");
          Serial.print(azimut_odometry_iiikkk, 4);
          Serial.print("ww");
          Serial.print(distancia, 4);
          Serial.print("ww");
          Serial.print(distancia * cos(azimut_odometry_iiikkk / 10000L), 4);
          Serial.print("ww");
          Serial.print(distancia * sin(azimut_odometry_iiikkk / 10000L), 4);
          Serial.print("ww");
          Serial.print(x, 4);
          Serial.print("ww");
          Serial.print(y, 4);
          Serial.print("ww###");
          Serial.print('\n');

        } else {

          estado = 0;
          pararMotores();
          cmdVel = 0.0;
        }

      }

      // Fin de Estado 1060

      // Estado 1070, se debe mover 0,5 metro en línea recta

      if (estado >= 1070 && estado <= 1079) {

        if (valores[1] > distanciaEstado1070 - DISTANCIA_CALIBRACION) {


          //Vel =0.15 y omega=0
          movimiento_linea_recta((cmd_PWMIzq(cmd_omegaIzq(velEstado1070, 0))), (cmd_PWMDer(cmd_omegaDer(velEstado1070, 0))));
          incTiempoEstado1070 = millis() - tiempoEstado1070;
          float vel_inst1070 = (-(float)valores[1] + (float)distanciaEstado1070) / (float)incTiempoEstado1070;
          Serial.println("oooccc" + String(valores[1])  + "ww" + String(incTiempoEstado1070)  + "ww" + String(vel_inst1070)  + "ww"  + String( velEstado1070 / (vel_inst1070 * parameters[0]) ) + "ww###");

        } else {
          pararMotores();
          incTiempoEstado1070 = millis() - tiempoEstado1070;
          float vel_inst1070 = (-(float)valores[1] + (float)distanciaEstado1070) / (float)incTiempoEstado1070;
          Serial.println("oooccc" + String(valores[1])  + "ww" + String(incTiempoEstado1070)  + "ww" + String(vel_inst1070)  + "ww"  + String( velEstado1070 / (vel_inst1070 * parameters[0]) ) + "ww###");
          estado = 0;
        }

      }

      // Fin de Estado 1070

      // Estado 1110, debe girar
      if (estado == 1110) {
        if  (abs(azimutRef - azimut) > 0.04) {

          movimiento_giro();
          estado = 1110;

        } else {

          contadorEstado1110 += 1;
          if (contadorEstado1110 > 5)
            estado = 0;
          pararMotores();

        }


        Serial.println("oooccc" + String(azimutRef) + "ww" + String(azimut) + "ww" + String(millis()) + "ww" + String(outPD) + "ww###"); //En radianes

      }

      // Fin de Estado 1070

      // Estado 1140

      if (estado == 1140) {
        if (incAzimut != azimut) { //Realiza un arco de circunferencia, vel!=0 y omega!=0

          movimiento_arco(cmd_PWMIzq(cmd_omegaIzq(parameters[0] * cmdVel, parameters[1] * cmdOmega)), cmd_PWMDer(cmd_omegaDer(parameters[0] * cmdVel, parameters[1] * cmdOmega)));

          Serial.println("oooccc" + String(azimutEstado1140) + "ww" + String(azimut) + "ww" + String(millis() - tiempoEstado1140) + "ww" + String(ICCR) + "ww" + String(outPID_arco) + "ww###"); //En radianes
        }

        if ( ((float)  atan2((double)sin(azimutEstado1140 - azimut), (double)cos(azimutEstado1140 - azimut)) <= (0.00)) && (millis() > tiempoEstado1140 + 500)) {
          pararMotores();
          estado = 0;
          estadoPID = -1;
        }
      }

      // Fin Estado 1140


      // Estado 2010, anda recto tras recibir una orden
      if (estado == 2010) {
        azimut_odometry_iiikkk = azimut_odometry_iiikkk + incAzimut;
        movimiento_linea_recta((cmd_PWMIzq(cmd_omegaIzq(parameters[0] * cmdVel, 0))), (cmd_PWMDer(cmd_omegaDer(parameters[0] * cmdVel, 0))));
        //azimut_odometry += 0; //Supone que hace la línea recta y no se desvía ??? podria ser ErrorAzimut*Cos(incAzimut) PENSAR??
        //azimut_odometry = azimut_odometry_iiikkk + errorAzimut; //Considera la desviación del ángulo usada en el PID

      }
      // Fin Estado 2010

      // Estado 2020, giro puro
      if (estado == 2020) {
        azimut_odometry += incAzimut;
        //azimut_odometry =  atan2(sin(azimut_odometry), cos(azimut_odometry)); //Limita a -pi pi . HAcer Funcion !!!
      }
      // Fin Estado 2010


      // Estado 2000, anda haciendo un arco
      if (estado == 2000) {
        if (incAzimut != azimut) { //Realiza un arco de circunferencia, vel!=0 y omega!=0

          //Considerando que incAzimut es pequeña estimo la velocidad lineal
          //vel_estimada = 1000 * (ICCR * incAzimut) / dtAzimut;

          movimiento_arco(cmd_PWMIzq(cmd_omegaIzq(parameters[0] * cmdVel, parameters[1] * cmdOmega)), cmd_PWMDer(cmd_omegaDer(parameters[0] * cmdVel, parameters[1] * cmdOmega)));

          // Calculo de x y para odometria
          x +=  ICCR * (sin(azimut_odometry + incAzimut) - sin(azimut_odometry));
          y += ICCR * (cos(azimut_odometry) - cos(azimut_odometry + incAzimut));

          // Actualizacion de azimut para odometria
          azimut_odometry += incAzimut;
          //azimut_odometry =  atan2(sin(azimut_odometry), cos(azimut_odometry)); //Limita a -pi pi . HAcer Funcion !!!

        }
      }
      // Fin Estado 2000

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

      trozo1Str = inputString.substring(inputString.indexOf("ttt") + 3, inputString.indexOf("ww"));
      trozo2Str = inputString.substring(inputString.indexOf("ww") + 2, inputString.indexOf("ww###"));

      //Valores por defecto en test
      //No considera el comando de velocidad anterior, en cualquier test
      cmdVelAnterior = 0;
      cmdVel = 0.15;
      cmdOmega = 0.5;

      //Test1: Para motores y vuelve al estado 0
      if (trozo1Str.toInt() == 1) {
        pararMotores();
        estado = 0;
        estadoPID = -1;
      }
      //Test2: Merodea sin chocar usando los ultrasonidos.
      else if (trozo1Str.toInt() == 2) {
        estado = 1020;
      }
      //Test3: Se orienta hacia la luz sin conocimiento previo
      else if (trozo1Str.toInt() == 3) {
        estado = 1030;
      }
      //Test4:
      else if (trozo1Str.toInt() == 4) {
        //estado que crea tabla w / pwm
        estado = 1040;
        incPWM = ((int)parameters[19]); //Valor inicial del PWM
        tiempoOmega = millis();
        tiempoEstado1040 = 140;
        omegaLP = 0;
        // Arranque de motores
        analogWrite(motorIAvance, 0);
        analogWrite(motorIRetroceso, 0);
        analogWrite(motorDAvance, 255);
        analogWrite(motorDRetroceso, 0);

      }
      else if (trozo1Str.toInt() == 5) {
        //Realiza un control PD basado en la velocidad angular
        estado = 1050;

        tiempoEstado1050 = millis();
        azimutRef = 100;
        outPID = 0;
        outPIDPWM = 0;

        analogWrite(motorIAvance, 240); //Valores calculados a mano OJO!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        analogWrite(motorIRetroceso, 0);
        analogWrite(motorDAvance, 230);
        analogWrite(motorDRetroceso, 0);

      }    else if (trozo1Str.toInt() == 6) {
        //estado que realiza un control PID en el ángulo, moviendose en línea recta
        estado = 1060;
        azimut_odometry_iiikkk = 0.0;
        distancia = 0;
        x = 0.0;
        y = 0.0;

        tiempoEstado1060 = millis();
        if (cmdVel != cmdVelAnterior) {
          azimutRef = azimut;
        }

        inicia_linea_recta();


      } else if (trozo1Str.toInt() == 7) {
        //estado que realiza un control PID en el ángulo, y se mueve hasta una determinada distancia
        estado = 1070;
        velEstado1070 = parameters[0] * trozo2Str.toFloat();

        tiempoEstado1070 = millis();
        distanciaEstado1070 = valores[1];

        if (cmdVel != cmdVelAnterior) {
          azimutRef = azimut;
        }

        inicia_linea_recta();

      }  else if (trozo1Str.toInt() == 8) {
        //estado que crea tabla w / pwm. Calibración automática
        estado = 1080;
        incPWM = ((int)parameters[19]); //Valor inicial del PWM
        tiempoOmega = millis();
        numOmegas = 0;
        mediaOmega = 0;

      }
      else if (trozo1Str.toInt() == 9) {
        //Trata de regresar al origen empleando los puntos Wifi
        estado = 1090;

      }
      else if (trozo1Str.toInt() == 10) {
        //Test 10: Trata de calcular la aceleración angular máxima.
        estado = 1100;
        incPWM = 170; //Valor inicial del PWM
        tiempoOmega = millis();
        omegaLP = 0; //Valor inicial del filtro LP
      }
      else if (trozo1Str.toInt() == 11) {
        //Giro
        estado = 1110;
        tiempoEstado1110 = millis();
        contadorEstado1110 = 0; // Contador de sobreoscilación
        inicia_giro((float)  atan2((double)sin(azimut - 3.1416), (double)cos(azimut - 3.1416)));
      }
      else if (trozo1Str.toInt() == 12) {
        //Para el root e inicia las variables de estado
        estado = 0;
        pararMotores();
        distancia = 0;
        x = 0;
        y = 0;
        azimut_odometry = 0;
        estadoPID = -1;
      }
      else if (trozo1Str.toInt() == 14) {
        estado = 1140;
        tiempoEstado1140 = millis();
        azimutEstado1140 = azimut;

        cmdVel = 0.15;
        cmdOmega = 0.5;

        ICCR = 0.5 * (parameters[3] / parameters[4]) * DISTANCIA_RUEDAS * ( cmd_omegaDer(parameters[0] * cmdVel, parameters[1] * cmdOmega) + cmd_omegaIzq(parameters[0] * cmdVel, parameters[1] * cmdOmega) ) / ( cmd_omegaDer(parameters[0] * cmdVel, parameters[1] * cmdOmega) - cmd_omegaIzq(parameters[0] * cmdVel, parameters[1] * cmdOmega) );

        inicia_arco();
        Serial.println("ooocccazimutEstado1140wwazimutwwTiempowwICCRwwoutPID_arcoww###");
        movimiento_arco(cmd_PWMIzq(cmd_omegaIzq(parameters[0] * cmdVel, parameters[1] * cmdOmega)), cmd_PWMDer(cmd_omegaDer(parameters[0] * cmdVel, parameters[1] * cmdOmega)));

      }
      //Test15: Se orienta hacia la luz sin chocar sin conocimiento previo, usando iot y Machine Learning
      else if (trozo1Str.toInt() == 15) {
        estado = 1150;
        auxX = 0;

      }
      //Test16: Se orienta hacia la luz sin chocar sin conocimiento previo, usando arquitectura VSA
      else if (trozo1Str.toInt() == 16) {
        estado = 1160;

      }


      inputString = "";
      commandComplete = false;
    }
    //Recibe una orden para modificar los parámetros
    else if (inputString.startsWith("iiippp")) {
      trozo1Str = inputString.substring(inputString.indexOf("ppp") + 3, inputString.indexOf("ww"));
      trozo2Str = inputString.substring(inputString.indexOf("ww") + 2, inputString.indexOf("ww###"));

      indice = trozo1Str.toInt();
      //Admite hasta 21 parametros
      if (indice >= 0 && indice < 21)
        parameters[indice] = trozo2Str.toFloat();


      inputString = "";
      commandComplete = false;
    }

    //Recibe una orden tipo twist (velocidad lineal / velocidad angular)
    else if (inputString.startsWith("iiikkk")) {

      //Guarda la velocidad anterior, para saber si tiene que aplicar una nueva referencia de azimut o no
      cmdVelAnterior = cmdVel;

      trozo1Str = inputString.substring(inputString.indexOf("kkk") + 3, inputString.indexOf("ww"));
      trozo2Str = inputString.substring(inputString.indexOf("ww") + 2, inputString.indexOf("vv"));
      trozo3Str = inputString.substring(inputString.indexOf("vv") + 2, inputString.indexOf("ww###"));

      cmdVel = trozo1Str.toFloat();
      cmdOmega = trozo2Str.toFloat();
      cmdAzimut = trozo3Str.toFloat() / 10000.0;

      azimut_odometry_iiikkk = 00.0; // azimut_odometry ;

      distancia = 0;


      if (cmdVel == 0 && cmdOmega == 0) {
        pararMotores();
        estado = 0;
        cmdVelAnterior = 0;
      }
      else if (cmdVel != 0 && cmdOmega == 0) {
        //Anda en línea recta
        estado = 2010;
        if (cmdVel != cmdVelAnterior) {
          azimutRef = cmdAzimut;
          pararMotores();
          //Serial.println("AzimutRef:" +  String(azimutRef) + " " + trozo3Str);
        }
        inicia_linea_recta();

      }
      //Gira en el sitio
      else if (cmdVel == 0 && cmdOmega != 0) {
        estado = 2020;

        motoresPWM(cmd_PWMIzq(cmd_omegaIzq(0.0 , parameters[1] * cmdOmega)), cmd_PWMDer(cmd_omegaDer(0.0 , parameters[1] * cmdOmega)));
        cmdVelAnterior = 0;

      }

      //Anda haciendo un arco de circunferencia
      else if (cmdVel != 0 && cmdOmega != 0) {
        estado = 2000;

        //Calcula el radio de curvatura ICC,para estima x,y
        // de www.doc.ic.ac.uk/~ajd/Robotics Lecture 2
        // R=W(vr+vl)/2(vr-vl)
        ICCR = 0.5 * (parameters[3] / parameters[4]) * DISTANCIA_RUEDAS * ( cmd_omegaDer(parameters[0] * cmdVel, parameters[1] * cmdOmega) + cmd_omegaIzq(parameters[0] * cmdVel, parameters[1] * cmdOmega) ) / ( cmd_omegaDer(parameters[0] * cmdVel, parameters[1] * cmdOmega) - cmd_omegaIzq(parameters[0] * cmdVel, parameters[1] * cmdOmega) );
        //ICCR = 0*5* DISTANCIA_RUEDAS * ( cmd_omegaDer(parameters[0] * cmdVel, 3 * parameters[1] * cmdOmega) + cmd_omegaIzq(parameters[0] * cmdVel, 3* parameters[1] * cmdOmega) ) / ( cmd_omegaDer(parameters[0] * cmdVel, 3 * parameters[1] * cmdOmega) - cmd_omegaIzq(parameters[0] * cmdVel, 3 * parameters[1] * cmdOmega) );

        inicia_arco(); //030116


      }

      inputString = "";
      commandComplete = false;
    }

    // iiibbbXXXww### donde XXX es la correlación respecto al origen (destino buscado)
    else if (inputString.startsWith("iiibbb")) {

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

      if (valores[0] < 200 && valores[0] > 0 && valores [2] >= 200) {
        pararMotores();
        analogWrite(motorIAvance, 0);
        analogWrite(motorIRetroceso, 250);
        analogWrite(motorDAvance, 250);
        analogWrite(motorDRetroceso, 0);
      }
      if (valores[2] < 200 && valores[2] > 0 && valores [0] >= 200) {
        pararMotores();
        analogWrite(motorIAvance, 250);
        analogWrite(motorIRetroceso, 0);
        analogWrite(motorDAvance, 0);
        analogWrite(motorDRetroceso, 250);
      }
      if (valores[0] < 100 && valores[1] < 100 && valores[2] < 100 && (valores[1] > 0 || valores [1] > 0 || valores[2] > 0)) {
        pararMotores();
        analogWrite(motorIAvance, 0);
        analogWrite(motorIRetroceso, 250);
        analogWrite(motorDAvance, 0);
        analogWrite(motorDRetroceso, 250);
        delay(500);
        pararMotores();
        analogWrite(motorIAvance, 0);
        analogWrite(motorIRetroceso, 250);
        analogWrite(motorDAvance, 250);
        analogWrite(motorDRetroceso, 0);
        delay(600);
        pararMotores();
      }
      if ((valores[0] >= 200 || valores[0] == 0) && (valores [1] >= 100 || valores[1] == 0) && (valores[2] >= 200 || valores[2] == 0)) {
        pararMotores();
        analogWrite(motorIAvance, 250);
        analogWrite(motorIRetroceso, 0);
        analogWrite(motorDAvance, 250);
        analogWrite(motorDRetroceso, 0);
      }



      break;

    case 1150:
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
      break;

    case 1160:
      //VSA
      break;
    case 1030:
      //Programa del 2010
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
      //Fin del programa de 2010

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

      }

      else {
        pararMotores();
        incTiempoEstado1070 = millis() - tiempoEstado1070;
        float vel_inst1070 = (-(float)valores[1] + (float)distanciaEstado1070) / (float)incTiempoEstado1070;
        Serial.println("oooccc" + String(valores[1])  + "ww" + String(incTiempoEstado1070)  + "ww" + String(vel_inst1070)  + "ww"  + String( velEstado1070 / (vel_inst1070 * parameters[0])) + "ww###");


        estado = 0;


      }
      break;

    case 1080:
      // Calculando automaticamente los valores de la linea PWM velocidad angular
      break;

    case 1090:
      // Se mueve hasta el origen usando los puntos de acceso proximos.
      break;

    case 1100:
      // Pasa de PWM 180 a 250, tratando de calcular la velocidad angular máxima. (Para posteriormente filtrar)
      break;

    case 1110:
      // Se orienta a un angulo
      break;

    case 1140:
      //Describe media circunferencia
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
  // = 1, si motor derecho esta por debajo de ((int)parameters[19])
  // = 2, si motor derecho está por enciam de ((int)parameters[18])
  // = 10, si motor izquierdo esta por debajo de ((int)parameters[19])
  // = 20, si motor izquierdo está por enciam de ((int)parameters[18])

  if (abs(motor_derecho) > ((int)parameters[18])) {
    motor_derecho = ((int)parameters[18]) * (motor_derecho / abs(motor_derecho));
    saturacion += 2;
  } else if (abs(motor_derecho) < ((int)parameters[19])) {
    //motor_derecho = 0 ;
    saturacion += 1;
  }

  if (abs(motor_izquierdo) > ((int)parameters[18])) {
    motor_izquierdo = ((int)parameters[18]) * (motor_izquierdo / abs(motor_izquierdo));
    saturacion += 20;
  } else if (abs(motor_izquierdo) < ((int)parameters[19])) {
    //motor_izquierdo = 0 ;
    saturacion += 10;
  }

  // Arranque
  /*
  if (motor_derecho > 0) {
    analogWrite(motorDAvance, 250);
    analogWrite(motorDRetroceso, 0);
  } else {
    analogWrite(motorDAvance, 0);
    analogWrite(motorDRetroceso, 250);
  }

  if (motor_izquierdo > 0) {
    analogWrite(motorIAvance, 250);
    analogWrite(motorIRetroceso, 0);
  } else {
    analogWrite(motorIAvance, 0);
    analogWrite(motorIRetroceso, 250);
  }
  */

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

  if (saturacion != 0 && estadoPID == 0) //Si el PID esta fuuncionando (estadoPID=0) y existe saturacion (saturacion !=0), entonces pasa el estadoPID a 1 (indicando que está saturado)
  {
    estadoPID = 1;
  }
  else {
    if (saturacion == 0 && estadoPID == 1) // Y Si deja de estar saturado deja de estar saturado
    {
      estadoPID = 0;
    }
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

  return (cmd_vel  + (parameters[3] * DISTANCIA_RUEDAS * cmd_ang / 2)) / (parameters[2] * RADIO_RUEDA);

}
float cmd_omegaIzq(float cmd_vel, float cmd_ang) {

  return (cmd_vel - (parameters[3] * DISTANCIA_RUEDAS * cmd_ang / 2)) / (parameters[2] * RADIO_RUEDA);

}


//Relación omea / PWM (PEnDIENTE DE CAMBIAR VALORES FIJOS POR CALIBRACION !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!)
int cmd_PWMDer(float omegaDer) {

  return (int)((parameters[7] * abs(omegaDer)) + parameters[8]) * (omegaDer / abs(omegaDer));
  //return (int)((101.5 * abs(omegaDer)) + 103.41) * (omegaDer / abs(omegaDer));
}

int cmd_PWMIzq(float omegaIzq) {

  return (int)((parameters[5] * abs(omegaIzq)) + parameters[6]) * (omegaIzq / abs(omegaIzq));
  //return (int)((111.5 * abs(omegaIzq)) + 112.41) * (omegaIzq / abs(omegaIzq));
}

int movimiento_arco(int PWM_I, int PWM_D) {

  salPID = pid_arco();

  PWM_I = PWM_I - salPID;
  PWM_D = PWM_D + salPID;

  motoresPWM(PWM_I, PWM_D);
}

int movimiento_linea_recta(int PWM_I, int PWM_D) {

  //int PWM_I = 225;
  //int PWM_D = 215;

  salPID = pid(); //Ejecuta el PID

  PWM_I = PWM_I + salPID;
  PWM_D = PWM_D - salPID;

  //SATURACION.OJO PARAR INTEGRAL del  PID!!!!!!!!!!!!!!!!!
  motoresPWM(PWM_I, PWM_D);


}

int inicia_arco() {

  //pararMotores();//Da problemas en teleoperacion

  estadoPID = -1;

  outPID_arco = 0;

  return 0;

}

int inicia_linea_recta() {

  //pararMotores(); //Da problemas en teleoperacion

  //inicia PID
  estadoPID = -1;

  //Para cálculo de distancia del movimiento
  distanciaUS_inicial = valores[1];
  distanciaUS_anterior = -1;

  outPID = 0;


  azimutAnterior = azimut;


  //Nueva confiuración de PID Azimut
  errorAzimut = 0;  //error P
  errorAzimutD = 0;  //error D
  errorAzimutI = 0;   //error I
  errorAzimutAnterior = 0;
  tiempoAzimutAnterior = millis();
  outPID = 0;

}

int movimiento_giro() {

  int PWM_I;
  int PWM_D;

  outPD = pd(); //Sólo PD

  //PWM_I =  (outPIDPWM) + (((int)parameters[19]) * (outPIDPWM / abs(outPIDPWM)));
  //PWM_D =  (outPIDPWM * (-1)) - (((int)parameters[19]) * (outPIDPWM / abs(outPIDPWM)));

  PWM_I = outPD + (((int)parameters[19]) * (outPD / abs(outPD)));
  PWM_D =  - outPD + (-1) * (((int)parameters[19]) * (outPD / abs(outPD)));

  //SATURACION.OJO PARAR INTEGRAL del  PID!!!!!!!!!!!!!!!!!
  motoresPWM(PWM_I, PWM_D);



}

int inicia_giro(float azimutGira) {

  estadoPID == -1; // Sólo PD para guirar
  outPD = 0;

  //Falta controlar que azimutGira esté entre -pi y pi

  azimutRef = azimutGira; // Establece la referencia del azimut

  outPID = 0;
  outPIDPWM = 0;

  azimutAnterior = azimut;

  //Arranque de motores
  /*
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
  */
}


//PD
int pd() {


  //P
  errorAzimut = azimutRef - azimut; //Redefine el error
  errorAzimut = (float)  atan2((double)sin(errorAzimut), (double)cos(errorAzimut));
  //outPID =  kpd * (errorAzimut);
  outPID =  ((int)parameters[12]) * (errorAzimut);


  errorAzimutAnterior = errorAzimut;
  tiempoAzimutAnterior = millis();


  azimutAnterior = azimut;


  return (int)((outPID));

}


//PID_arco
int pid_arco() {

  //
  omega_estimada = 1000 * incAzimut / dtAzimut;
  errorOmega = cmdOmega - omega_estimada;

  //P
  if (estadoPID == -1) {
    outPID_arco = 0;
    estadoPID = 1;
  } else {
    //outPID_arco =  ((int)parameters[15])o * errorOmega;
    outPID_arco =  ((int)parameters[15]) * errorOmega;
  }

  //I

  //D

  return (int)((outPID_arco));

}

//PID

int pid() {


  //P
  errorAzimut = azimutRef - azimut; //Redefine el error
  errorAzimut = (float)  atan2((double)sin(errorAzimut), (double)cos(errorAzimut));

  //I
  if (estadoPID == -1) {
    //La primera vez que se ejecuta el PID
    errorAzimutI = 0;
  } else {
    if (estadoPID == 0) {
      //Si no está saturado. estadoPID==1 si alguno de los actuarores está en el máximo (o mínimo)
      errorAzimutI += (errorAzimutAnterior * ((float)(millis() - tiempoAzimutAnterior))) ;
    }
  }


  //D
  // El estadoPID indica si se inició (-1) o si está saturado (1)
  if (estadoPID == -1) {
    estadoPID = 0; // Tras la primera vez que se ejecuta pasa a estado 0, para comenzar a calcular el error D
  } else {
    //if ((millis() - tiempoAzimutAnterior) != 0 && errorAzimut>=0.2)
    //if ((millis() - tiempoAzimutAnterior) != 0 && abs(errorAzimut) >= 0.05)
    if ((millis() - tiempoAzimutAnterior) != 0)
      errorAzimutD =  (1000 * (errorAzimut - errorAzimutAnterior)) / (millis() - tiempoAzimutAnterior);
    else
      errorAzimutD = 0;
  }


  //outPID =  (((int)parameters[9]) * errorAzimut) + (((int)parameters[10]) * errorAzimutI) + (((int)parameters[11])/10) * errorAzimutD ;



  //outPID =  ((int)parameters[9]) * (errorAzimut + (2 * errorAzimutI) + (errorAzimutD)/4);
  outPID =  (((int)parameters[9]) * (errorAzimut)) + (((int)parameters[10]) * (errorAzimutI / 1000)) + (((int)parameters[11]) * errorAzimutD);
  //Actualización de valores
  errorAzimutAnterior = errorAzimut;
  tiempoAzimutAnterior = millis();

  azimutAnterior = azimut;

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



