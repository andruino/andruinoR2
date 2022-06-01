// Constantes a convertir en parámetros si es necesario

#define DISTANCIA_RUEDAS 0.129   //R2D a punto medio de ruedas
#define RADIO_RUEDA 0.03175		 //R2D 1.25 pulgadas

#define SENSIBLE_US 10 
#define DISTANCIA_CALIBRACION 500 


#define omegaMIN 1.5 //R2D PENDIENTE Cuando realiza un giro puro
#define omegaMAX 2.5 //R2D PENDIENTE 

#define velMAX 0.2 //R2D PENDIENTE Obtenida de repetir el test 7 para diferentes velocidades
#define velMIN 0.1 //R2D PENDIENTE 0.14

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
