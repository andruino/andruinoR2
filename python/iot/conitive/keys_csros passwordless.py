#!/usr/bin/env python
# BEGIN ALL
import rospy
import std_srvs.srv
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import base64
import json
import requests

key_mapping = { 'w': [ 0, 1], 'x': [0, -1], 
                'a': [-1, 0], 'd': [1,  0], 
                's': [ 0, 0] }
g_last_twist = None 
g_vel_scales = [2, 1] # default to very slow 

image_filename="foto.jpg"
api_key = "XXX"
data_result=[]
data_string=[]
lista_texto=[]

def keys_cb(msg, twist_pub):
	global g_last_twist, g_vel_scales
	if len(msg.data) > 0 and key_mapping.has_key(msg.data[0]):
	
		vels = key_mapping[msg.data[0]]
		g_last_twist.angular.z = vels[0] * g_vel_scales[0]
		g_last_twist.linear.x  = vels[1] * g_vel_scales[1]
		twist_pub.publish(g_last_twist)

	else:
		if msg.data=='p':
			#print msg.data
			call_image_saver()
			post_url="XXX"
			image_data = open(image_filename, 'rb').read()
			result = requests.post(post_url, data=image_data, headers={'Content-Type': 'application/octet-stream'})
			result.raise_for_status
			#print result.text["description"]	
			data_result = json.loads(result.text)
			lista_texto=str(data_result.get('description').get('captions')).split("u\'")
			texto="iiittt0017ww"+"\""+lista_texto[2][:-3]+"\""+"ww###"
			print texto
			cmd_pub.publish(texto)

			
			lista_texto=str(data_result.get('tags')).split("u\'")
			print lista_texto
			
			
		else:
			return # unknown key.


	

if __name__ == '__main__':
  rospy.init_node('keys_csros')
  rospy.wait_for_service('/image_saver/save') #Espera que se inicie el servicio de captura de imagenes
  #twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
  twist_pub = rospy.Publisher('cmd_vel', Twist)
  cmd_pub = rospy.Publisher('cmd', String)
  rospy.Subscriber('keys', String, keys_cb, twist_pub)
  g_last_twist = Twist() # initializes to zero
  call_image_saver=rospy.ServiceProxy('/image_saver/save', std_srvs.srv.Empty())
  # BEGIN PARAM
  if rospy.has_param('~linear_scale'):
    g_vel_scales[1] = rospy.get_param('~linear_scale')
  else:
    rospy.logwarn("linear scale not provided; using %1f" %\
                  g_vel_scales[1])
  # END PARAM

  if rospy.has_param('~angular_scale'):
    g_vel_scales[0] = rospy.get_param('~angular_scale')
  else:
    rospy.logwarn("angular scale not provided; using %2f" %\
                  g_vel_scales[0])

  #rate = rospy.Rate(10)
  rate = rospy.Rate(5)
  while not rospy.is_shutdown():
    #twist_pub.publish(g_last_twist)
    rate.sleep()
# END ALL
