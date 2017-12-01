#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy

#usar
#rostopic echo /ema_tao/kneeAngle/data > kneeAngle.txt
#rostopic echo /ema_tao/upperLeg/data > upperLeg.txt

#import ros msgs
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64

# import utilities
from math import pi
from tf import transformations

	
##################################################
##### Funções de Callback ########################
##################################################

def lowerLegAngle_callback(data):
	global lowerLegAngle
	#global kneeAngle

	qx,qy,qz,qw = data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w
	euler = transformations.euler_from_quaternion([qx, qy, qz, qw], axes='syxz')

	lowerLegAngle = euler[0]
	lowerLegAngle = lowerLegAngle * (180/pi)
 
	#kneeAngle  = upperLegAngle - lowerLegAngle



def upperLegAngle_callback(data):
	global upperLegAngle
	global lowerLegAngle
	global footAngle
	global bodyAngle
	#global kneeAngle
	#global pubKnee
	global pubUpperLeg
	global pubLowerLeg
	global pubFoot
	global pubBody

	qx,qy,qz,qw = data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w
	euler = transformations.euler_from_quaternion([qx, qy, qz, qw], axes='syxz')

	upperLegAngle = euler[0]
	upperLegAngle = upperLegAngle * (180/pi)

	#kneeAngle  = upperLegAngle - lowerLegAngle

	#pubKnee.publish(kneeAngle)
	pubBody.publish(bodyAngle)
	pubUpperLeg.publish(upperLegAngle)
	pubLowerLeg.publish(lowerLegAngle)
	pubFoot.publish(footAngle)
	#pubLowerLeg.publish(lowerLegAngle)
	#pubFoot.publish(footAngle)

def footAngle_callback(data):
	global footAngle

	qx,qy,qz,qw = data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w
	euler = transformations.euler_from_quaternion([qx, qy, qz, qw], axes='syxz')

	footAngle = euler[0]
	footAngle = footAngle * (180/pi)

def bodyAngle_callback(data):
	global bodyAngle

	qx,qy,qz,qw = data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w
	euler = transformations.euler_from_quaternion([qx, qy, qz, qw], axes='syxz')

	bodyAngle = euler[0]
	bodyAngle = bodyAngle * (180/pi)



##################################################
##### Iniciação de variáveis globais ############
##################################################
footAngle = -1
lowerLegAngle = -1
upperLegAngle = -1
kneeAngle = -1
bodyAngle = -1


##################################################
##### Loop do ROS ################################
##################################################

def recorder():
	global state
	#global pubKnee
	global pubUpperLeg
	global pubLowerLeg
	global pubFoot
	global pubBody

	rospy.init_node('recorder', anonymous = True)
	#pubKnee = rospy.Publisher('kneeAngle', Float64, queue_size = 10)
	pubUpperLeg = rospy.Publisher('upperLegAngle', Float64, queue_size = 10)
	pubLowerLeg = rospy.Publisher('lowerLegAngle', Float64, queue_size = 10)
	pubFoot = rospy.Publisher('footAngle', Float64, queue_size = 10)
	pubBody = rospy.Publisher('bodyAngle', Float64, queue_size = 10)
	rospy.Subscriber('imu/foot', Imu, callback = footAngle_callback)
	rospy.Subscriber('imu/lowerLeg', Imu, callback = lowerLegAngle_callback)
	rospy.Subscriber('imu/upperLeg', Imu, callback = upperLegAngle_callback)
	rospy.Subscriber('imu/body', Imu, callback = bodyAngle_callback)

	while not rospy.is_shutdown():
		pass



if __name__ == '__main__':
	recorder()