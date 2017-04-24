#!/usr/bin/python
'''
Autor: Paco Lopez, dentro de proyecto/tesis Andruino diriguido por Federico Cuesta (ESI, US)
Basado en trabajo previo de Ross Gayler en relacion a la VSA
Licencia Creative Commons Atribucion, Derivadas no comercial (CC BY-NC)
version:92
'''
import random
import rospy
import rosgraph.masterapi
import numpy
import os
import csv

from std_msgs.msg import String

# Constantes de ROS
Umbral_LDR = 10 # Valor minimo para que sea considerado mejora en el nivel medio de luz

#Valores minimos y maximos para los sensores de luz (OJO!!!!SOLO CONSIDERA UN SENSOR, HABRA QUE CAMBIAR ESTO POR LISTA CUANDO SEAN VARIOS SENSORES!!!!!!!!!!!!)
SENMIN = 0
SENMAX = 900

# Constantes de VSA

SEM = 2     #Numero de sensores
DIVRANSEN = 2 #Division del rango de sensores
 # 4 #20 

SENRANGEVALUE = 20
SENRANGESIGN = 4 
SENRANGE = SENRANGEVALUE + SENRANGESIGN
 
ACTRANGE = 5 # Numero de posibles acciones, o de rango de valores dentro de una accion

debug=1


num_mov=0 #numero de movimientos, cada experimento debe iniciarse a 0

'''
Funciones y Objetos VSA
'''

		

#Producto de lista
def dotprod(a,b):
	d = 0
	respuesta=0
    	for k in range (0,len(a)):
    		d += a[k] * b[k]
    	if (d < -1*len(a)/2): 
        	respuesta = -1
    	else:
		if(d > len(a)/2):
    			respuesta=1
	#print "distancia:  " + str(d)
	return respuesta

def dotprod3(a,b,c):
	d = 0
	respuesta=0
    	for k in range (0,len(a)):
    		d += a[k] * b[k] *c[k]
    	if (d < -1*len(a)/2): 
        	respuesta = -1
    	else:
		if(d > len(a)/2):
    			respuesta=1
	#print "distancia:  " + str(d)
	return respuesta
	 

def distance(a,b):
	d = 0
    	for k in range (0,len(a)):
    		d += a[k] * b[k]
	return d

def distance3(a,b,c):
	d = 0
    	for k in range (0,len(a)):
    		d += a[k] * b[k]*c[k]
	return d


class vsa(object):
	s = []
	a = []
	c = []

	ds =[] 	#Contiene los vectores que definen la secuencia ("precedido por", "seguido de")
	seq= [] #Vector que contiene la secuencia de estados para realizar predicciones

	epsilon=0.15 #% de las acciones son aleatorias puras
	gamma = 1

	sensors0=[]
	sensors1=[]
	values0=[]
	values1=[]
	actuators=[]
	

	def __init__(self,NDIMS):

		#Verificacion de si existen ficheros para cargar
		if  os.path.exists('sensors.dat') and os.path.exists('actuators.dat') and os.path.exists('controller.dat') and os.path.exists('sequence.dat') and os.path.exists('defseq.dat'):
			No_ficheros=False
		else:
			No_ficheros=True

		#Sensores

		for i in range(0,SENRANGE):
    			subs= []
			if No_ficheros==True :
	    			for j in range(0,NDIMS):
					subs.append(random.randrange(-1,2,2))
	    			self.s.append(subs)
			self.sensors0.append(-1)
			self.sensors1.append(-1)
			self.values0.append(-1)
			self.values1.append(-1)

		if No_ficheros==True :
			numpy.savetxt('sensors.dat',self.s)
		else:
			self.s=numpy.loadtxt('sensors.dat',dtype=int)
			if debug==1:
				print "Cargando el fichero de sensores"
		#print "Sensors s-----------------------------------"
		#print self.s		
		#Actuadores
		for i in range(0,ACTRANGE):
			suba=[]
			if No_ficheros==True :
				for j in range(0,NDIMS):
					suba.append(random.randrange(-1,2,2))
	    			self.a.append(suba)
			self.actuators.append(0)
		
		if No_ficheros==True :
			numpy.savetxt('actuators.dat',self.a)
		else:
			self.a=numpy.loadtxt('actuators.dat',dtype=int)
			if debug==1:
				print "Cargando el fichero de actuadores"

		#Controlador
		if No_ficheros==True :
			for i in range(0,NDIMS):			
				self.c.append(0)
		else:
			self.c=numpy.loadtxt('controller.dat',dtype=int)
			if debug==1:
				print "Cargando el fichero de controladores"

		
		#Definiciones de secuencias ds (antes de , despues de [pensar si se necesitan mas])
		for i in range(0,ACTRANGE):
			subds=[]
			if No_ficheros==True :
				for j in range(0,NDIMS):
					subds.append(random.randrange(-1,2,2))
	    			self.ds.append(subds)
			
		
		if No_ficheros==True :
			numpy.savetxt('defseq.dat',self.ds)
		else:
			self.ds=numpy.loadtxt('defseq.dat',dtype=int)
			if debug==1:
				print "Cargando el fichero de definicion de secuencias"

		#Secuencia
		if No_ficheros==True :
			for i in range(0,NDIMS):			
				self.seq.append(0)
		else:
			self.seq=numpy.loadtxt('sequence.dat',dtype=int)
			if debug==1:
				print "Cargando el fichero de secuencias"
		'''
		for j in (0,SENRANGE):
			self.sensors0.append(0)
		for j in (0,SENRANGE):
			self.sensors1.append(0)
		for j in (0,ACTRANGE):
			self.actuators.append(0)
		'''
	def save_controller(self):
		numpy.savetxt('controller.dat',self.c)	

	def save_sequence(self):
		numpy.savetxt('sequence.dat',self.seq)	

	#Cambiar los cuatro metodos siguientes por uno solo
	def set_state0(self,sensors):
		for j in range(0,SENRANGE):
			self.sensors0[j]=sensors[j]

	def set_state1(self,sensors):
		for j in range(0,SENRANGE):
			self.sensors1[j]=sensors[j]

	def set_values0(self,sensors):
		for j in range(0,SENRANGE):
			self.values0[j]=sensors[j]

	def set_values1(self,sensors):
		for j in range(0,SENRANGE):
			self.values1[j]=sensors[j]

	def get_state0(self):
		return self.sensors0

	def get_state1(self):
		return self.sensors1
	

	def lumax(self,sensors):
		result=[]
		s_k=[]
		dis=[]

		for l in range(0,SENRANGE):
			s_k.append(0)
		for l in range(0,ACTRANGE):
			dis.append(0)

		for k in range (0,len(self.c)):
			result_k = self.c[k]
			for l in range(0,SENRANGE):

				if sensors[l]==1:
					s_k[l]= self.s[l][k]
				else:
					s_k[l]= 1

				result_k=result_k*s_k[l]

			result.append(result_k)
			
		for j in range (0,ACTRANGE):
			dis[j]=distance(result, self.a[j])

		#Normalizacion al maximo
				
		maxdis=max(dis)

		for j in range (0,ACTRANGE):
			dis[j]=int(dis[j]/(0.8*len(self.c)))
		
		return dis

	# Uso del vsa, dado un estado (expresado como valor de sensores) me da la accion
	def lookup(self, sensors):
		result=[]
		s_k=[]
		mot=[]

		for l in range(0,SENRANGE):
			s_k.append(0)
		for l in range(0,ACTRANGE):
			mot.append(0)

		for k in range (0,len(self.c)):
			result_k = self.c[k]
			for l in range(0,SENRANGE):

				if sensors[l]==1:
					s_k[l]= self.s[l][k]
				else:
					s_k[l]= 1

				result_k=result_k*s_k[l]

			result.append(result_k)
			
		for j in range (0,ACTRANGE):
			mot[j]=dotprod(result, self.a[j])
		
		return mot
	
	#DesAprendizaje
	def unenter(self, sensors, actuators):
		s_k=[]
		a_k=[]
		for l in range(0,SENRANGE):
			s_k.append(0)
		for l in range(0,ACTRANGE):
			a_k.append(0)

		for k in range (0,len(self.c)):
			#Sensores
			s_k_parcial = 1
			for l in range(0,SENRANGE):

				if sensors[l]==1:
					s_k[l]= self.s[l][k]
				else:
					s_k[l]= 1

				s_k_parcial=(-1)*s_k_parcial*s_k[l]
			

			#Actuadores
			a_k_parcial = 0
			for l in range(0,ACTRANGE):

				if actuators[l]==1:
					a_k[l]= self.a[l][k]
				else:
					a_k[l]=0

				a_k_parcial=a_k_parcial+a_k[l]
			
			#Actualizacion controlador
			
			self.c[k] = int(self.c[k]) + int(s_k_parcial * a_k_parcial) 
		
		#Cada nueva regla salva el controlador
		#numpy.savetxt('controller.dat',self.c)
	
	#Aprendizaje
	def enter(self, sensors, actuators):
		s_k=[]
		a_k=[]
		for l in range(0,SENRANGE):
			s_k.append(0)
		for l in range(0,ACTRANGE):
			a_k.append(0)

		for k in range (0,len(self.c)):
			#Sensores
			s_k_parcial = 1
			for l in range(0,SENRANGE):

				if sensors[l]==1:
					s_k[l]= self.s[l][k]
				else:
					s_k[l]= 1

				s_k_parcial=s_k_parcial*s_k[l]
			
			
			#Actuadores
			a_k_parcial = 0
			for l in range(0,ACTRANGE):

				if actuators[l]==1:
					a_k[l]= self.a[l][k]
				else:
					a_k[l]=0

				a_k_parcial=a_k_parcial+a_k[l]
			
			#Actualizacion controlador
			
			self.c[k] = int(self.c[k]) + int(s_k_parcial * a_k_parcial) 
		
		#Cada nueva regla salva el controlador
		#numpy.savetxt('controller.dat',self.c)

	#Aprendizaje de secuencias
	def enterseq(self, values0, values1):
		e0_k=[] #s_k=[] Estado en t=0
		e1_k=[] #a_k=[] Estado en t=1
		for l in range(0,SENRANGE):
			e0_k.append(0)
		for l in range(0,SENRANGE):
			e1_k.append(0)

		for k in range (0,len(self.seq)):
			#Valores de estado (sensores) en t= 0 
			e0_k_parcial = 1
			for l in range(0,SENRANGE):

				if values0[l]==1:
					e0_k[l]= self.s[l][k]
				else:
					e0_k[l]= 1

				e0_k_parcial=e0_k_parcial*e0_k[l]

			#Valores de estado (sensores) en t= 1 
			e1_k_parcial = 1
			for l in range(0,SENRANGE):

				if values1[l]==1:
					e1_k[l]= self.s[l][k]
				else:
					e1_k[l]= 1

				e1_k_parcial=e1_k_parcial*e1_k[l]
		
		
			#Actualizacion secuencia
			
		
			self.seq[k] = int(self.seq[k]) + int(e0_k_parcial * e1_k_parcial) 
		#print "IMPRIMO SECUENCIA"
		#print self.seq

	# Prevision de estado futuro, dado el estado actual
	def lookupseq(self, values0):
		result=[]
		result2=[]
		e0_k=[]
		prev=[]
		aux=[]

		for k in range (0,len(self.seq)):
			result2.append(0)

		for l in range(0,SENRANGE):
			e0_k.append(0)

		for l in range(0,SENRANGE):
			prev.append(0)
			aux.append(0)

		for k in range (0,len(self.seq)):
			result_k = self.seq[k]
			for l in range(0,SENRANGE):

				if values0[l]==1:
					e0_k[l]= self.s[l][k]
				else:
					e0_k[l]= 1

				result_k=result_k*e0_k[l]

			result.append(result_k)
		#print "result"
		#print result
		
		#Solo para el caso de dos entradas activadas a la vez (dos sensores de luz, donde solo una puede estar activa)
		for j in range (4,SENRANGE-1):
			if values0[j]==1:
				for k in range (0,len(self.seq)):
					result2[k] = result[k]*self.s[j][k]
				#print "result2"
				#print result2
				#print "s"
				#print self.s[j]


				for m in range (0,SENRANGE):
					prev[m]=prev[m]+dotprod(result2, self.s[m])
					#print "prev"
					#print prev	
					#print "s"
					#print self.s[m]	
		return prev
	
		
'''
Fin de Funciones y Objetos VSA
'''

#Crea el vector vsa, que contendra la base de conocimiento
andruino_vsa=vsa(20000)

'''
Nodo ROS
'''
# Mensaje desde el topic /andruinoXXX/distance, que trae los datos del sensores
# Formato de datos sera 
#      LDR_Izq_0, LDR_Cen_0; LDR_Der_0; LDR_Izq_1; LDR_Cen_1; LDR_Der_1, ACCION_ejecutada(???); [COMA AL FINAL, OJO!!!]
#      lista(0) , lista(1),  lista[2],  lista[3] , lista[4] , lista[5], lista[6],
def callback(msg):
	#print msg.data.replace(';',',')[0:len(msg.data)-1]
	cadenaaux=msg.data.replace(';',',')[0:len(msg.data)-1]
	
	maxa=[]
	sensors0=[]
	sensors1=[]
	values0=[]
	values1=[]
	actuators=[]
	
	#inicio listas
	for j in range(0,ACTRANGE):
		maxa.append(0)
	
	for j in range(0,SENRANGE):
		sensors0.append(0)
	
	for j in range(0,SENRANGE):
		sensors1.append(0)

	for j in range(0,SENRANGE):
		values0.append(0)
	
	for j in range(0,SENRANGE):
		values1.append(0)

	for j in range(0,ACTRANGE):
		actuators.append(0)


	#Convierto de csv a lista, cambiando ; por , y quitando la ; del final
	list = msg.data.replace(';',',')[0:len(msg.data)-1].split(",")

	if len(list)==13:
		if debug==1: 
			print "RESULTADO:--------------------------------------------------"			
			print list

		LDRdiff=int(list[0])-int(list[2])
		LDRdiffpost=int(list[3])-int(list[5])
		
		LDRmedia=(int(list[0]) + int(list[2])/2)
		LDRmediapost=(int(list[3]) + int(list[5])/2)
		if debug==1:
			print "PUNTUACION GLOBAL:--------------------------------------------------"			
			print LDRmediapost
			print "MEJORA"
			print LDRmediapost-LDRmedia

		#Variables para definir el estado en base a la diferencia de luz y la media de la luz (SIGNO)
		
		#Define los estados "antes"
		if (LDRdiff > 0) and (abs(LDRdiff)> 50):
			sensors0[0]=1
		if (LDRdiff < 0) and (abs(LDRdiff) > 50):
			sensors0[1]=1
		if (abs(LDRdiff)<=50 and abs(LDRmedia)>250):
			sensors0[2]=1
		if (abs(LDRdiff)<=50 and abs(LDRmedia)<=250):
			sensors0[3]=1
			print "fin del episodio************************************************+"
			#pub.publish("iiittt012ww###")
			#pub.publish("iiittt012ww###")
			
		#Define los estados "despues"
		if (LDRdiffpost > 0) and (abs(LDRdiffpost) > 50):
			sensors1[0]=1
		if (LDRdiffpost < 0) and (abs(LDRdiffpost) > 50):
			sensors1[1]=1
		if (abs(LDRdiffpost)<=50 and abs(LDRmediapost)>250):
			sensors1[2]=1
		if (abs(LDRdiffpost)<=50 and abs(LDRmediapost)<=250):
			sensors1[3]=1
		
		actuators[int(list[12])]=1 


		#Variables para definir el estado en base a los valores de los sensores de luz (VALOR)
		
		values0[SENRANGESIGN + int(int(list[0])/100)]=1
		values0[SENRANGESIGN+10+int(int(list[2])/100)]=1
		values1[SENRANGESIGN+int(int(list[3])/100)]=1
		values1[SENRANGESIGN+10+int(int(list[5])/100)]=1

		LDRdiff=int(list[0])-int(list[2])
		LDRdiffpost=int(list[3])-int(list[5])
		
		LDRmedia=int(list[0]) + int(list[2])
		LDRmediapost=int(list[3]) + int(list[5])
		
		if debug==1:
			print "sensors0"
			print sensors0
			print "sensors1"
			print sensors1
			print "values0"
			print values0
			print "values1"
			print values1
	
	

		if (LDRmediapost < LDRmedia): 

			#Aprende la secuencia
			andruino_vsa.enterseq(values0,values1)
			
			#Q-Learning, mira primero el futuro
			for p in range(0,ACTRANGE-1):
				maxa[p]=0		
			#Mira si en el furuto hay mejora por esa actuacion
			maxa=andruino_vsa.lumax(values1)
			if max(maxa)>1:
				if debug==1:
					print "DEBERIA APRENDER Q-Learning"
				#for h in range(0,int(max(maxa)/2)):
				andruino_vsa.enter(values0,actuators)
					

			if debug==1:
				print "DEBERIA APRENDER POR RECOMPENSA INSTANTANEA"
			#for h in range(0,abs(LDRmediapost-LDRmedia)):
			andruino_vsa.enter(sensors0,actuators)
			# Y aprende en valor
			andruino_vsa.enter(values0,actuators) 
				

				#andruino_vsa.enter(sensors0,actuators)
			#Si llego a un lugar iluminado con luz y centrado mete la regla 25 veces, para fin del espisodio
			'''
			if abs(LDRmediapost)<=250 and abs(LDRdiffpost)<=50:
				for i in range(0,3):
					andruino_vsa.enter(sensors0,actuators)
					andruino_vsa.enter(values0,actuators)	
				if debug==1:
					print "OBJETIVO ALCANZADO!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"					
			'''					
		
			if debug==1: 
				print "Q(s,a,r,s')"
				print "AprendioOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOo"
				print "sensors0"
				print sensors0
				print "values0"
				print values0
				print "actuators"
				print actuators
				print "maxa de values1"
				print maxa
				print "sensors1"
				print sensors1
				print "values1"
				print values1
				print "FIN Q(s,a,r,s')"

		else:
			#Desaprende
			andruino_vsa.unenter(values0,actuators) 
			#recompensa_instantanea = 0

		#Guarda los signos
		andruino_vsa.set_state0(sensors0)
		andruino_vsa.set_state1(sensors1)
			
		#Guarda los valores
		andruino_vsa.set_values0(values0)
		andruino_vsa.set_values1(values1)
	
		#print "sensors1 guardados"
		#print andruino_vsa.get_state1()
		

#Listar todos los topics, para ver cuantos topics distance hay en ese master 
master = rosgraph.masterapi.Master('/rostopic')
list_of_lists= master.getPublishedTopics('/') 
andruinos_lista=[]
for list in list_of_lists:
    for x in list:
        #print x
	if "distance" in x: 
    		#x=x.replace("/andruino","andruino")
		#x=x.replace("/distance",",distance")
		andruinos_lista.append(x)
		print andruinos_lista


#Crea el nodo
rospy.init_node('vsa')

# Va a publicar en topic /cmd y recibir en topic /andruinoXXX/distance
# (Supone que solo hay un robot)

pub = rospy.Publisher('cmd',String)

for list in andruinos_lista:
	sub= rospy.Subscriber(list,String,callback)

# Envia el comando de test para colocar el andruino en modo de VSA, comando iiittt16ww###
if debug==1:
	rate=rospy.Rate(.5)
else:
	rate=rospy.Rate(1)

count=0
while not rospy.is_shutdown():
	if count<2:
		pub.publish("iiittt016ww###")
		count+=1
	else:
		if debug==1:
			print "ACCION:-------------------------------------------------"
		#Segun indique el parametro ,escogera una accion aleatoria o si son los primeros 10 movimientos...
		
		if random.random() < andruino_vsa.epsilon or num_mov<17:
			
			if num_mov>17:
				action = random.randint(1,ACTRANGE-1) #Solo las acciones de guiro, para empezar a aprender
				if debug==1:
					print "ACCION aleatoria pura"
				
			else:
				
				if num_mov < 3:
					action=3
				if (num_mov>=3 and num_mov<=5):
					action=4
				if (num_mov>=6 and num_mov<=8):
					action=1
				if (num_mov>=9 and num_mov<=14):
					action=2
				if (num_mov>=15 and num_mov<=17):
					action=1
				if debug==1:
					print "ACCION POR MOVIMIENTOS INICIALES. ESCUELA"
			
			num_mov+=1
	
		else:
			#El estado actual es el estado tras la ultima accion
			num_mov+=1	
			#Q en signo (q)
			q=andruino_vsa.lookup(andruino_vsa.sensors1)
			q_values=andruino_vsa.lumax(andruino_vsa.sensors1)
			#Q en valores (qv)
			qv=andruino_vsa.lookup(andruino_vsa.values1)
			qv_values=andruino_vsa.lumax(andruino_vsa.values1)

			max_val = max(q_values)
			maxv_val= max(qv_values)

			if debug==1:
					print "sensors"
					print andruino_vsa.sensors1
					print "q_values"
					print q_values
					print "max_val"
					print max_val
					print "values"
					print andruino_vsa.values1
					print "qv_values"
					print qv_values
					print "maxv_val"
					print maxv_val
						
								
		    	
			
					

			#Si no sabe nada movimiento aleatorio, si tiene conocimiento en q se mueve por q
			if max_val==0 and maxv_val==0:
				action = random.randint(1,ACTRANGE-1)
				if debug==1:
					print "ACCION aleatoriA NO VALOR EN Q"
				
			else:
				action_candidates_q=[]

				if maxv_val==0:
					
					for i,x in enumerate(q_values):
						if x==max_val:
							action_candidates_q.append(i)
				
        				action=random.choice(action_candidates_q)

					if debug==1:
						print "ACCION por Q-SIGNO++++++++++++++++++++++++++++++++++"
						

				else:
					
					if max_val==0:
						for i,x in enumerate(qv_values):
							if x==maxv_val:
								action_candidates_q.append(i)
				
						action=random.choice(action_candidates_q)

						if debug==1:
							print "ACCION por Q-VALUE++++++++++++++++++++++++++++++++++"
							
					else:
												
						if random.random() < 0.33:
							for i,x in enumerate(q_values):
								if x==max_val:
									action_candidates_q.append(i)

							action=random.choice(action_candidates_q)
							
							if debug==1:
								print "ACCION por Q-SIGNO (AUNQUE HABIA ALTERNATIVA DE VALOR)"
							
						else:
							for i,x in enumerate(qv_values):
								if x==maxv_val:
									action_candidates_q.append(i)
				
							action=random.choice(action_candidates_q)

							if debug==1:
								print "ACCION por Q-VALUE(AUNQUE HABIA ALTERNATIVA DE SIGNO)"
		
		#Cada 25 ensayos almacena el controlador		
		if (num_mov % 25 ==1):
			andruino_vsa.save_controller()	
			andruino_vsa.save_sequence()

		#Trata de predecir el siguiente estado
		preduccion=[]
		prediccion=andruino_vsa.lookupseq(andruino_vsa.values1)
		if debug==1:
			print "PREDICCION DE ESTADO??????????????????"		
			print prediccion
			print "??????????????????"
	
		#Manda a andruino ejecutar la acci0n	 
		pub.publish("iiiqqq"+str(action)+"ww###")
		'''
		 /* Movimientos = action
			 * 0 para
			 * 1 gira derecha (dextrogiro)
			 * 2 gira izquierda
			 * 3 avanza
			 * 4 retrocede
			 * 10.... para
			 */
		'''
    
	

	
	#rospy.spin()
	#rospy.sleep(1.)
	
	rate.sleep()


#Espera un segundo
#rospy.sleep(1.)





'''

#Aprende
entrada=[0,0,1,1,0,0]
salida=[0,0,0,0,1]
andruino_vsa.enter(entrada,salida)
entrada=[1,0,0,1,0,0]
salida=[1,0,0,0,0]
andruino_vsa.enter(entrada,salida)
entrada=[1,0,0,0,1,0]
salida=[1,1,1,1,1]
andruino_vsa.enter(entrada,salida)
entrada=[0,0,1,0,0,1]
salida=[0,0,0,0,1]
andruino_vsa.enter(entrada,salida)
entrada=[0,0,1,1,0,0]
salida=[0,0,0,0,1]
andruino_vsa.enter(entrada,salida)
entrada=[0,0,1,1,0,0]
salida=[0,0,0,0,1]
andruino_vsa.enter(entrada,salida)

#Usa lo aprendido
entrada=[1,0,0,0,1,0]
aux=andruino_vsa.lookup(entrada)
print aux
'''

