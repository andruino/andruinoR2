# Representa grafica pra ZieglerNichols
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Para obtener los datos ejecutar el comando
# Terminal 1: rostopic pub /cmd std_msgs/String iiittt006ww###
# Terminal2: rostopic echo /andruinoXXX/distance > znomega.csv

# Prepocesamiento (deberia haberlo hecho con panda)...MEJORA FUTURA

# abrimos el archivo solo de lectura
f = open("znomega.csv","r")
 
# Creamos una lista con cada una de sus lineas
lineas = f.readlines()
 
# cerramos el archivo
f.close()
 
# abrimos el archivo pero vacio
f = open("znomega2.csv","w")
 
# recorremos todas las lineas quitando los elementos no necesarios para la representacion xy y la regresion lineal.
for linea in lineas:
 	linea=linea.replace('data: ','')		
 	if "---" not in linea:
		f.write(linea)
 
# cerramos el archivo
f.close()



# Carga de valores usado panda
datos = pd.read_csv('znomega2.csv',header=None, sep=';', usecols=[1,0,2] )
datos2 = pd.read_csv('znomega2.csv', header=None, sep=';', usecols=[1,2] )
# Limpia los valores NaN
datos.dropna(how="all", inplace=True) 
datos2.dropna(how="all", inplace=True)

#datos[0] = datos[0].astype('float64')


#print datos.values

fig, ax = plt.subplots()

#fit = np.polyfit(datos.values[:,0], datos.values[:,1], 1)
#print fit
#ax.plot(datos.values[:,0], fit[0] * datos.values[:,0] + fit[1], color="red")
plt.plot(datos.values[:,1], datos.values[:,0], color='r')
plt.plot(datos.values[:,1], datos.values[:,2], color='b')
#ax.plot(datos2.values[:,1],datos2.values[:,0])

plt.ylabel('Azimuth(rad)')
plt.xlabel('Time (ms)')
plt.title('P Controller')
plt.axis([0, 6200, 0, 1.5])

#fig.show()
plt.show()
