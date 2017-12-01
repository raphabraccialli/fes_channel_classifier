#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy

#usar
#rostopic echo /ema_tao/kneeAngle/data > kneeAngle.txt
#rostopic echo /ema_tao/upperLeg/data > upperLeg.txt

#import ros msgs
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion

# import utilities
from math import pi
from tf import transformations

	
##################################################
##### Funções de Callback ########################
##################################################

def bodyAngle_callback(data):
	
	pubBody.publish(data.orientation)




##################################################
##### Loop do ROS ################################
##################################################

def recorder():
	global pubBody

	rospy.init_node('recorder', anonymous = True)
	pubBody = rospy.Publisher('bodyAngle', Quaternion, queue_size = 10)
	rospy.Subscriber('imu/pedal', Imu, callback = bodyAngle_callback)

	while not rospy.is_shutdown():
		pass



if __name__ == '__main__':
	recorder()