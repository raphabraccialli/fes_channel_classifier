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
		self.angleThreshold = 10
		self.min_current = 8
		self.max_current = 20
		self.current_step = 2
		self.current = self.min_current

	def get_angle(self, data):
		qx,qy,qz,qw = data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w
		euler = transformations.euler_from_quaternion([qx, qy, qz, qw], axes='syxz')
		self.angle = euler[0] * (180/pi)
		return self.angle

	def stimulation_routine(self):
		if self.counter == 0:
			print('Estimulando canal ' 
				  + str(self.scannedChannels[self.channelIndex]) 
				  + '\nEstimulation current: ' + str(self.current))
		self.stimMsg.pulse_current = [self.current]
		self.stimMsg.pulse_width = [500]
		self.stimMsg.channel = [self.scannedChannels[self.channelIndex]]
		self.pubStim.publish(self.stimMsg)

		#print(self.counter)

		if self.counter < 200:
			if self.angle > self.maxAngle[self.channelIndex]:
				self.maxAngle[self.channelIndex] = self.angle
			self.counter += 1
		else:
			self.stimMsg.pulse_current = [0]
			self.stimMsg.pulse_width = [0]
			self.pubStim.publish(self.stimMsg)

			print('Pause')
			rospy.sleep(5)

			self.counter = 0
			self.channelIndex += 1

	def angle_callback(self, data):		
		self.get_angle(data)
		self.plotAngle.publish(self.angle)

	def channelScanner(self):
		''' 
			Stimulate through all channels for currents between min and max.
			When the maxAngle captured exeeds angleThreshold, save the channel and current value to the channelCurrent dict.
			Repeat the scan until all channelCurrents are set (not 0) or max_current is archieved.
		'''
		self.maxAngle = [0] * len(self.scannedChannels)

		#espera terminar calibragem
		rospy.sleep(5)
		rate = rospy.Rate(50)

		while not rospy.is_shutdown():
			# Scan through all channels
			if self.channelIndex < len(self.scannedChannels):
				# if channel currrent value for that chanel is not set (==0), estimulate
				if self.channelCurrent[self.scannedChannels[self.channelIndex]] == 0:
					self.stimulation_routine()
				else:
					# channel current already set
					self.channelIndex += 1
			else: # Evaluate scan results
				# if any channel moved above angleTreshold, add its current to channelCurrent
				for angIndex in range(len(self.maxAngle)):
					if self.maxAngle[angIndex] > self.angleThreshold:
						self.channelCurrent[self.scannedChannels[angIndex]] = self.current;
				# Print informations
				print(self.maxAngle)
				print(self.channelCurrent)

				# Reset maxAngle
				self.maxAngle = [0]*len(self.scannedChannels)
				# Reset channel index
				self.channelIndex = 0
				#update current
				self.current += self.current_step
				# If any of the channels were not added to dictionary
				if 0 in self.channelCurrent.values() and self.current <= self.max_current:
					continue # continue to estimulate
				else:
					break # all channels are set
				
			rate.sleep()

		# Zera canais de estimulação antes de desligar
		self.stimMsg.pulse_current = [0]
		self.stimMsg.pulse_width = [0]
		self.pubStim.publish(self.stimMsg)

if __name__ == '__main__':
	channel_scan = ChannelScanner()
	channel_scan.channelScanner()
