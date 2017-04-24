#!/usr/bin/env python

""" 
      
"""

import roslib 
import rospy
from geometry_msgs.msg import Twist, Quaternion
from std_msgs.msg import Int32
'''
from nav_msgs.msg import Odometry
import tf
from math import radians, copysign
from transform_utils import quat_to_angle, normalize_angle
'''

rospy.init_node('gira_crea_mapa')

r = rospy.Rate(10)

pub = rospy.Publisher('/andruino/cmd_vel', Twist)
#pub = rospy.Publisher('counter', Int32)     
count=0 

while not rospy.is_shutdown():

	 move_cmd = Twist()
         move_cmd.angular.z = 0
         pub.publish(move_cmd)
	 move_cmd.angular.z = 0
         pub.publish(move_cmd)
	 r.sleep()
         move_cmd.angular.z = 1
         pub.publish(move_cmd)
	 move_cmd.angular.z = 1
         pub.publish(move_cmd)
	 r.sleep()
	


'''
	pub.publish(count)
	count += 1
	r.sleep()

'''

