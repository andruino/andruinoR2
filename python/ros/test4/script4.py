#! /usr/bin/python

# Para generar correctamente antes debe obtener los datos ejecutando el comando
# rostopic echo /andruinoXXX/distance > pwmomega.csv

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Prepocesamiento deberia usar Panda...MEJORA FUTURA

# abrimos el archivo solo de lectura
f = open("pwmomega.csv","r")
 
# Creamos una lista con cada una de sus lineas
lineas = f.readlines()
 
# cerramos el archivo
f.close()
 
# abrimos el archivo pero vacio
f = open("pwmomega2.csv","w")
 
# recorremos todas las lineas quitando los elementos no necesarios para la representacion xy y la regresion lineal.
for linea in lineas:
 	linea=linea.replace('data: ','')		
 	if "---" not in linea:
		f.write(linea)
 
# cerramos el archivo
f.close()

# Carga de valores usado panda
datos = pd.read_csv('pwmomega2.csv', header=None, sep=';', usecols=[1,0])
# Limpia los valores NaN
datos.dropna(how="all", inplace=True) 

#print datos.values

fig, ax = plt.subplots()

fit = np.polyfit(datos.values[:,0], datos.values[:,1], 1)
print fit
ax.plot(datos.values[:,0], fit[0] * datos.values[:,0] + fit[1], color='red')
ax.scatter(datos.values[:,0],datos.values[:,1])

plt.ylabel('PWM values')
plt.xlabel('Angular velocity (rad/seg)')
plt.title('From Angular velocity to PWM')

#fig.show()
plt.show()
