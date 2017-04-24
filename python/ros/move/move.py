#!/usr/bin/env python

import roslib 
import rospy
from geometry_msgs.msg import Twist, Quaternion
from std_msgs.msg import Int32


rospy.init_node('move')

r = rospy.Rate(10)

pub = rospy.Publisher('/andruino144/cmd_vel', Twist)
#pub = rospy.Publisher('counter', Int32)     
count=0 

while not rospy.is_shutdown():

	 move_cmd = Twist()
         move_cmd.angular.z = 0
	 move_cmd.linear.x  = 0
         pub.publish(move_cmd)
	 r.sleep()
	 move_cmd.angular.z = 0
	 move_cmd.linear.x  = 0.15
         pub.publish(move_cmd)
	 r.sleep()
	 move_cmd.angular.z = 0
	 move_cmd.linear.x  = 0
         pub.publish(move_cmd)
	 r.sleep()
         move_cmd.angular.z = 1
	 move_cmd.linear.x  = -0.15
         pub.publish(move_cmd)
	 move_cmd.angular.z = 0
	 move_cmd.linear.x  = 0
         pub.publish(move_cmd)
	 r.sleep()
