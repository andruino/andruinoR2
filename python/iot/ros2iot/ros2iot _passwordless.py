#!/usr/bin/python

import paho.mqtt.publish as publish
import paho.mqtt.subscribe as subscribe
import paho.mqtt.client as mqtt
import ssl
import os
import socket
import json

import rospy
from std_msgs.msg import String
import rosgraph.masterapi

andruinos_lista=[]



auth = {
  'username':"XXX.azure-devices.net/XXX",
  'password':"XXX"
}

tls = {
  'ca_certs':"/etc/ssl/certs/ca-certificates.crt",
  'tls_version':ssl.PROTOCOL_TLSv1
}




def on_connect(client, userdata, rc):
    print "Connected with result code: %s" % rc
    client.subscribe("devices/XXX/messages/devicebound/#")
  
def on_disconnect(client, userdata, rc):
    print "Disconnected with result code: %s" % rc  

def on_message(client, userdata, msg):
    print " --------------------- ".join((msg.topic, str(msg.payload)))
    #client.publish("devices/XXX/messages/events", "REPLY", qos=1)

def on_publish(client, userdata, mid):
    print "Sent message"



client = mqtt.Client(client_id="XXX", protocol=mqtt.MQTTv311)


client.on_connect = on_connect
client.on_message = on_message
client.on_disconnect = on_disconnect
client.on_publish = on_publish


client.username_pw_set(username="XXX.azure-devices.net/XXX",
  password="XXX")
client.tls_set(ca_certs="/etc/ssl/certs/ca-certificates.crt",tls_version=ssl.PROTOCOL_TLSv1)
#client.tls_insecure_set(True)
client.connect("XXX.azure-devices.net", port=8883)
#client.loop_forever()

cont=0;

def callback(msg):
	msg_json=""
	msg_list=[]
	now = rospy.Time.now()
	#print (msg.data.replace(';',','))[0:len(msg.data)-1]
	msg_list=((msg.data.replace(';',','))[0:len(msg.data)-1]).split(',')
	print msg_list	
	topico= msg._connection_header['topic']
	#topico=topico.replace("/andruino","andruino")
	#topico=topico.replace("/distance",",distance")	
	msg_json={"rostopic":topico,"us1":int(msg_list[1])}
	print msg_json
	client.publish("devices/XXX/messages/events/",payload=json.dumps(msg_json))


#Listar todos los topics, para ver cuantos topics distance hay en ese master 
master = rosgraph.masterapi.Master('/rostopic')
list_of_lists= master.getPublishedTopics('/') 

for list in list_of_lists:
    for x in list:
	if "distance" in x: 
		andruinos_lista.append(x)
		print andruinos_lista


rospy.init_node('ros2iot')

for list in andruinos_lista:
	sub= rospy.Subscriber(list,String,callback)
rospy.spin()
#client.loop_forever()