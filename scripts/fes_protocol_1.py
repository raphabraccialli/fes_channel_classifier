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


class FESProtocol1:
	'''
	'''
	def __init__(self):
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
		''' Iniciação das variaveis da classe '''
		self.channelCurrent = {}
		self.channelCurrentReference = []
		self.start = False
		self.counter = 0
		self.activeChannels = []
		self.activeCurrents = []
		self.activePulseWidths = []


	def channel_input(self, data):
		# Unsubscribes from topic
		self.subInput.unregister()

		# Saves input for possible future reference
		self.channelCurrentReference = data.data

		# The recieved message is a list where the channel is accessed by the index, thus it shall not change.
		# The following piece of code transforms it into dictionary, so it can ignore zeros (off channels) 
		# and pop values without changing the index (channel) of the related currents.
		for c in range(len(self.channelCurrentReference)):
			if self.channelCurrentReference[c] > 0:
				self.channelCurrent[c+1] = self.channelCurrentReference[c]

		# Prints channels as a dictionary
		print('Protocol started with values: ' + str(self.channelCurrent))
		
		# Enables protocol to start
		#self.start = True

		print('\n')
		while len(self.channelCurrent) > 0:
			self.add_min_channel()
		

	def add_min_channel(self):
		# adds channel of minimum current

		# finds first occurence of minimum current
		minCurrent = min(self.channelCurrent.values())
		# finds it's index in the values list
		indexOfMinCurrent = list(self.channelCurrent.values()).index(minCurrent)
		# with it's index, finds the key in the key list
		channelOfMinCurrent = list(self.channelCurrent.keys())[indexOfMinCurrent]

		self.channelCurrent.pop(channelOfMinCurrent)

		#self.activeChannels.append(channelOfMinCurrent)
		#self.activeCurrents.append(minCurrent)
		#self.activePulseWidths.append(500)

		self.activeChannels = channelOfMinCurrent
		self.activeCurrents = minCurrent
		self.activePulseWidths = 500

		print('Channels to be added: ' + str(self.channelCurrent))
		print('Active channels: ' + str(self.activeChannels))
		print('Active currents: ' + str(self.activeCurrents))
		print('Active currents: ' + str(self.activePulseWidths))
		print('\n')


	def add_max_channel(self):
		# adds channel of maximum current

		# finds first occurence of maximum current
		maxCurrent = max(self.channelCurrent.values())
		# finds it's index in the values list
		indexOfMaxCurrent = list(self.channelCurrent.values()).index(maxCurrent)
		# with it's index, finds the key in the key list
		channelOfMaxCurrent = list(self.channelCurrent.keys())[indexOfMaxCurrent]

		self.channelCurrent.pop(channelOfMaxCurrent)

		#self.activeChannels.append(channelOfMaxCurrent)
		#self.activeCurrents.append(maxCurrent)
		#self.activePulseWidths.append(500)

		self.activeChannels = channelOfMaxCurrent
		self.activeCurrents = maxCurrent
		self.activePulseWidths = 500

		print('Channels to be added: ' + str(self.channelCurrent))
		print('Active channels: ' + str(self.activeChannels))
		print('Active currents: ' + str(self.activeCurrents))
		print('Active currents: ' + str(self.activePulseWidths))
		print('\n')

		
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

	def fes_protocol_1(self):
		'''
		'''

		#espera terminar calibragem
		rospy.sleep(5)
		rate = rospy.Rate(1)

		while not rospy.is_shutdown():
			if self.start:
				pass
			rate.sleep()

		# Zera canais de estimulação antes de desligar
		#self.stimMsg.pulse_current = [0]
		#self.stimMsg.pulse_width = [0]
		#self.pubStim.publish(self.stimMsg)

		rospy.sleep(1)

if __name__ == '__main__':
	fes_protocol = FESProtocol1()
	fes_protocol.fes_protocol_1()
