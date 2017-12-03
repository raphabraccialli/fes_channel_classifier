#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy

#import ros msgs
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from std_msgs.msg import Int8
from std_msgs.msg import Int8MultiArray
from ema_common_msgs.msg import Stimulator

# import utilities
from math import pi
from tf import transformations
import time

#import channelScanner class
from channelScanner import ChannelScanner


class FESProtocol1:
	'''
	'''
	def __init__(self):
			print('####################################### hello __init__')
			# inicia nó
			rospy.init_node('fes_protocol_1', anonymous = True)

			self.init_variables()

			# SUBSCRIBERS
			# Inscreve nos canais de pulibação de ângulos de cada sensor
			rospy.Subscriber('imu/angle', Imu, callback = self.angle_callback, queue_size=1)
			# Recebe valores dos canais para começar protocolo
			self.subInput = rospy.Subscriber('selectedChannels', Int8MultiArray, callback = self.channel_input, queue_size=10)
			
			# PUBLISHERS			
			# Publica ângulos  para mostrar em gráfico
			self.plotAngle = rospy.Publisher('angle', Float64, queue_size = 10)
			# Publica valores de eletroestimulação a serem lidos pelo estimulador
			self.pubStim = rospy.Publisher('stimulator/ccl_update', Stimulator, queue_size=10)


	def init_variables(self):
		print('####################################### hello init_variables')
		''' Iniciação das variaveis da classe '''
		self.channelCurrent = []


	def channel_input(self, data):
		print('####################################### hello callback')
		self.channelCurrent = data.data
		print(self.channelCurrent)
		self.subInput.unregister()

		
	def get_angle(self, data):
		qx,qy,qz,qw = data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w
		euler = transformations.euler_from_quaternion([qx, qy, qz, qw], axes='syxz')
		self.angle = euler[0] * (180/pi)
		return self.angle

	def stimulation_routine(self):
		pass

	def angle_callback(self, data):
		self.get_angle(data)
		self.plotAngle.publish(self.angle)

	def channelScanner(self):
		'''
		'''

		#espera terminar calibragem
		rospy.sleep(5)
		rate = rospy.Rate(1)

		while not rospy.is_shutdown():
			rate.sleep()

		# Zera canais de estimulação antes de desligar
		#self.stimMsg.pulse_current = [0]
		#self.stimMsg.pulse_width = [0]
		#self.pubStim.publish(self.stimMsg)

		rospy.sleep(1)

if __name__ == '__main__':
	fes_protocol = FESProtocol1()
	fes_protocol.channelScanner()
