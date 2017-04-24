#!/usr/bin/python

import random

SENRANGE = 9 # Valores de sensores diferenciados, bien por ser diferente sensor, bien por ser un rango de valores dentro del sensor
ACTRANGE = 5 # Numero de posibles acciones, o de rango de valores dentro de una accion

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

def distance(a,b):
	d = 0
    	for k in range (0,len(a)):
    		d += a[k] * b[k]
	return d


class vsa(object):
	s = []
	a = []
	c = []

	def __init__(self,NDIMS):

		#Sensores
		for i in range(0,SENRANGE):
    			subs= []
    			for j in range(0,NDIMS):
        			subs.append(random.randrange(-1,2,2))
    			self.s.append(subs)

		#Actuadores
		for i in range(0,ACTRANGE):
			suba=[]
			for j in range(0,NDIMS):
        			suba.append(random.randrange(-1,2,2))
    			self.a.append(suba)

		#Controlador
		for i in range(0,NDIMS):			
			self.c.append(0)
		

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
		'''		
		maxdis=max(dis)

		for j in range (0,ACTRANGE):
			dis[j]=int(dis[j]/(0.8*len(self.c)))
		'''
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
	#generalizar
	def generalize(self, sensors):
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
	#Aprendizaje
	#def enter(self, obst, lgt_l, lgt_r, mot_l, mot_r, mot_a, mot_b, mot_s):
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
		#print self.c 
			
#Crea el vector
andruino_vsa=vsa(10000)

#Aprende
'''
entrada=[0,0,1,0,0,0,0,0,0]
salida=[0,0,0,0,1]
andruino_vsa.enter(entrada,salida)
entrada=[0,0,1,0,0,1,0,0,0]
salida=[0,0,0,0,1]
andruino_vsa.enter(entrada,salida)
entrada=[0,0,1,0,0,1,0,0,1]
salida=[0,0,0,0,1]
andruino_vsa.enter(entrada,salida)
entrada=[0,0,1,0,0,1,0,0,0]
salida=[0,0,0,0,1]
andruino_vsa.enter(entrada,salida)
entrada=[0,0,1,0,0,0,0,0,1]
salida=[0,0,0,0,1]
andruino_vsa.enter(entrada,salida)
entrada=[0,0,1,0,0,0,0,0,0]
salida=[0,0,0,0,1]
andruino_vsa.enter(entrada,salida)
entrada=[0,0,1,0,0,1,0,0,1]
salida=[0,0,0,0,1]
andruino_vsa.enter(entrada,salida)
'''

entrada=[0,0,0,0,0,0,1,0,0]
salida=[1,0,0,0,0]
andruino_vsa.enter(entrada,salida)
entrada=[0,0,0,0,0,0,0,1,0]
salida=[0,1,0,0,0]
andruino_vsa.enter(entrada,salida)
entrada=[0,0,0,0,0,0,0,0,1]
salida=[0,0,1,0,0]
andruino_vsa.enter(entrada,salida)

entrada=[1,0,0,0,0,0,1,0,0]
salida=[1,0,0,0,0]
andruino_vsa.enter(entrada,salida)
entrada=[1,0,0,0,0,0,0,1,0]
salida=[0,1,0,0,0]
andruino_vsa.enter(entrada,salida)
entrada=[1,0,0,0,0,0,0,0,1]
salida=[0,0,1,0,0]
andruino_vsa.enter(entrada,salida)


'''
entrada=[0,0,0,0,0,0,0,1,0]
salida=[0,0,0,0,1]
andruino_vsa.enter(entrada,salida)
entrada=[1,0,0,0,0,0,0,0,1]
salida=[0,0,0,0,1]
andruino_vsa.enter(entrada,salida)
entrada=[0,1,0,0,0,0,0,0,1]
salida=[0,0,0,0,1]
andruino_vsa.enter(entrada,salida)
entrada=[0,0,1,0,0,0,0,0,1]
salida=[0,0,0,0,1]
andruino_vsa.enter(entrada,salida)


#Usa lo aprendido
entrada=[0,0,0,0,0,0,0,1,1]
aux=andruino_vsa.lookup(entrada)
print aux
aux=andruino_vsa.lumax(entrada)
print aux


entrada=[1,1,0,0,0,0,0,0,1]
aux=andruino_vsa.lookup(entrada)
print aux
aux=andruino_vsa.lumax(entrada)
print aux
'''
entrada=[1,0,0,0,0,0,0,0,1]
aux=andruino_vsa.lookup(entrada)
print aux
aux=andruino_vsa.lumax(entrada)
print aux
entrada=[1,0,0,0,0,0,0,0,0]
aux=andruino_vsa.lookup(entrada)
print aux
aux=andruino_vsa.lumax(entrada)
print aux



