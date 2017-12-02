#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy

#import ros msgs
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from std_msgs.msg import Int8
from ema_common_msgs.msg import Stimulator

# import utilities
from math import pi
from tf import transformations
import time


####### TO DO #######
# - remover do imu.yaml streamming de dados que não esão sendo usados
# - remover o publish do loop da main(). Está publicando acima da taxa de atualização das imus e distorcendo o gráfico


class ChannelScanner:
	''' Classe scanner de canais '''
	def __init__(self):
			# inicia nó
			rospy.init_node('channelScanner', anonymous = True)
			self.init_variables()
			# Inscreve nos canais de pulibação de ângulos de cada sensor
			rospy.Subscriber('imu/angle', Imu, callback = self.angle_callback, queue_size=1)
			# Publica ângulos  para mostrar em gráfico
			self.plotAngle = rospy.Publisher('angle', Float64, queue_size = 10)
			# Publica valores de eletroestimulação a serem lidos pelo estimulador
			self.pubStim = rospy.Publisher('stimulator/ccl_update', Stimulator, queue_size=10)

	def init_variables(self):
		''' Iniciação das variaveis da classe '''
		self.scannedChannels = [1, 3, 5, 7]
		self.stimMsg = Stimulator()
		self.stimMsg.channel = [1]
		self.stimMsg.mode = ['single']
		self.counter = 0
		self.channelIndex = 0
		self.maxAngle = []
		self.breaker = False
		self.channelCurrent = {}
		for c in self.scannedChannels:
			self.channelCurrent[c] = 0

	def get_angle(self, data):
		qx,qy,qz,qw = data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w
		euler = transformations.euler_from_quaternion([qx, qy, qz, qw], axes='syxz')
		self.angle = euler[0] * (180/pi)
		return self.angle

	def stimulation_routine(self):
		print('Estimulando canal ' + str(self.scannedChannels[self.channelIndex]))
		self.stimMsg.pulse_current = [8]
		self.stimMsg.pulse_width = [500]
		self.stimMsg.channel = [self.scannedChannels[self.channelIndex]]
		self.pubStim.publish(self.stimMsg)

		print(self.counter)

		if self.counter < 200:
			if self.angle > self.maxAngle[self.channelIndex-1]:
				self.maxAngle[self.channelIndex-1] = self.angle
			self.counter += 1
		else:
			self.stimMsg.pulse_current = [0]
			self.stimMsg.pulse_width = [0]
			self.pubStim.publish(self.stimMsg)

			rospy.sleep(5)

			self.counter = 0
			self.channelIndex += 1

	def angle_callback(self, data):		
		self.get_angle(data)
		self.plotAngle.publish(self.angle)

	def channelScanner(self):
		self.maxAngle = [0] * len(self.scannedChannels)

		#espera terminar calibragem
		rospy.sleep(5)
		rate = rospy.Rate(50)

		while not rospy.is_shutdown():
			if self.channelIndex < len(self.scannedChannels):
				if self.channelCurrent[self.scannedChannels[self.channelIndex]] == 0:
					self.stimulation_routine()
				else:
					self.channelIndex += 1
			else:
				break

			#else:
			#	if 'passou por todos canais' | 'corrente acima do limite':
			#			break

			rate.sleep()

		# Zera canais de estimulação antes de desligar
		self.stimMsg.pulse_current = [0]
		self.stimMsg.pulse_width = [0]
		self.pubStim.publish(self.stimMsg)

		print(self.maxAngle)


if __name__ == '__main__':
	channel_scan = ChannelScanner()
	channel_scan.channelScanner()
