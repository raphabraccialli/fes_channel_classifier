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


####### TO DO #######
# - remover do imu.yaml streamming de dados que não esão sendo usados
# - remover o publish do loop da main(). Está publicando acima da taxa de atualização das imus e distorcendo o gráfico
# - desligar todos canais, em vez de um, tanto nos intervalos quanto no encerramento do programa

class ChannelScanner:
	'''
		Classe scanner de canais.
		Esta classe realiza um scanner dos canais mais interessantes para a eletroestimulacao.
		Para tanto, monitora as informacoes de orientacao no plano sagital por meio do topico 'imu/angle'.
		Os resultados do processo de scanner sao retornados pela funcao principal da classe: channelScanner em forma de dicionario.
	'''
	def __init__(self):
			# inicia nó
			rospy.init_node('channelScanner', anonymous = True)
			self.init_variables()
			# Inscreve nos canais de pulibação de ângulos de cada sensor
			self.subImu = rospy.Subscriber('imu/angle', Imu, callback = self.angle_callback, queue_size=1)
			# Publica ângulos  para mostrar em gráfico
			self.plotAngle = rospy.Publisher('angle', Float64, queue_size = 10)
			# Publica valores de eletroestimulação a serem lidos pelo estimulador
			self.pubStim = rospy.Publisher('stimulator/ccl_update', Stimulator, queue_size=10)
			# Publica lista final com canais de estimulação
			self.pubReturn = rospy.Publisher('selectedChannels', Int8MultiArray, queue_size=10)

	def init_variables(self):
		''' Iniciação das variaveis da classe '''
		self.scannedChannels = [1, 2, 4]
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
		self.angleThreshold = 30
		self.min_current = 6
		self.max_current = 14
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

		if self.counter < 50:
			if self.angle > self.maxAngle[self.channelIndex]:
				self.maxAngle[self.channelIndex] = self.angle
			self.counter += 1
		else:
			self.stimMsg.pulse_current = [0]
			self.stimMsg.pulse_width = [0]
			self.pubStim.publish(self.stimMsg)

			print('Pause')
			print('\n')
			rospy.sleep(2)

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

				# asks and removes unconfortable channels
				channelToRemove = int(input('Would you like to remove any channels? '))
				if channelToRemove in self.channelCurrent.keys():
					print('Removing channel ' + str(channelToRemove) + '...')
					self.channelCurrent[channelToRemove] = -1
					rospy.sleep(1)
					print('Done')
				
				rospy.sleep(2)

				# Print informations
				print(self.maxAngle)
				print(self.channelCurrent)
				print('\n\n')

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

		returnList = [0] * 8
		for c in self.channelCurrent.keys():
			if self.channelCurrent[c] > 0:
				returnList[c-1] = self.channelCurrent[c]

		returnMsg = Int8MultiArray()
		returnMsg.data = returnList
		self.pubReturn.publish(returnMsg)

		self.subImu.unregister()

if __name__ == '__main__':
	channel_scan = ChannelScanner()
	channel_scan.channelScanner()
