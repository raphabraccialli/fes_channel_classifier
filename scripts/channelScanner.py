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
		self.numOfChannels = 2
		self.stimMsg = Stimulator()
		self.stimMsg.channel = [1]
		self.stimMsg.mode = ['single']
		self.counter = 0
		self.channel = 1
		self.maxAngle = []
		self.breaker = False

	def get_angle(self, data):
		qx,qy,qz,qw = data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w
		euler = transformations.euler_from_quaternion([qx, qy, qz, qw], axes='syxz')
		self.angle = euler[0] * (180/pi)
		return self.angle

	def stimulation_routine(self):
		print('Estimulando canal ' + str(self.channel))
		self.stimMsg.pulse_current = [8]
		self.stimMsg.pulse_width = [500]
		self.stimMsg.channel = [self.channel]
		self.pubStim.publish(self.stimMsg)

		print(self.counter)

		if self.counter < 200:
			if self.angle > self.maxAngle[self.channel-1]:
						self.maxAngle[self.channel-1] = self.angle
			self.counter += 1
		else:
			self.stimMsg.pulse_current = [0]
			self.stimMsg.pulse_width = [0]
			self.pubStim.publish(self.stimMsg)

			rospy.sleep(5)

			self.counter = 0
			self.channel += 1

	def angle_callback(self, data):
		if self.channel <= self.numOfChannels:
			self.get_angle(data)
			self.plotAngle.publish(self.angle)

			self.stimulation_routine()
		else:
			self.breaker = True

	def channelScanner(self):
		self.maxAngle = [0] * self.numOfChannels

		#espera terminar calibragem
		rospy.sleep(5)

		while not self.breaker:
			pass

		self.stimMsg.pulse_current = [0]
		self.stimMsg.pulse_width = [0]
		self.pubStim.publish(self.stimMsg)

		print(self.maxAngle)



####### TO DO #######
# - remover do imu.yaml streamming de dados que não esão sendo usados
# - remover o publish do loop da main(). Está publicando acima da taxa de atualização das imus e distorcendo o gráfico

##################################################
##### Máquina de estados #########################
##################################################

##################################################
##### Funções de Callback ########################
##################################################

# def angle_callback(data):
# 	global plotAngle
# 	global pubStim
# 	global maxAngle
# 	global channel
# 	global breaker
# 	global counter
# 	global numOfChannels
#
# 	if channel <= numOfChannels:
#
# 		qx,qy,qz,qw = data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w
# 		euler = transformations.euler_from_quaternion([qx, qy, qz, qw], axes='syxz')
# 		angle = euler[0] * (180/pi)
# 		plotAngle.publish(euler[0] * (180/pi))
#
# 		print('Estimulando canal ' + str(channel))
# 		stimMsg.pulse_current = [8]
# 		stimMsg.pulse_width = [500]
# 		stimMsg.channel = [channel]
# 		pubStim.publish(stimMsg)
#
# 		print(counter)
#
# 		if counter < 200:
# 			if angle > maxAngle[channel-1]:
# 						maxAngle[channel-1] = angle
# 			counter += 1
# 		else:
# 			stimMsg.pulse_current = [0]
# 			stimMsg.pulse_width = [0]
# 			pubStim.publish(stimMsg)
#
# 			rospy.sleep(5)
#
# 			counter = 0
# 			channel += 1
# 	else:
# 		breaker = True
#
#
#
# ##################################################
# ##### Iniciação de variáveis globais #############
# ##################################################
#
# numOfChannels = 2
#
# stimMsg = Stimulator()
# stimMsg.channel = [1]
# stimMsg.mode = ['single']
#
# counter = 0
#
# channel = 1
#
# maxAngle = []
#
# breaker = False
#
# ##################################################
# ##### Loop do ROS ################################
# ##################################################
#
# def channelScanner():
# 	global plotAngle
# 	global pubStim
# 	global numOfChannels
#
# 	for i in range(0, numOfChannels):
# 		maxAngle.append(0)
#
# 	# inicia nó
# 	rospy.init_node('channelScanner', anonymous = True)
#
# 	#espera terminar calibragem
# 	rospy.sleep(5)
#
# 	# Inscreve nos canais de pulibação de ângulos de cada sensor
# 	rospy.Subscriber('imu/angle', Imu, callback = angle_callback, queue_size=1)
#
# 	# Publica ângulos  para mostrar em gráfico
# 	plotAngle = rospy.Publisher('angle', Float64, queue_size = 10)
#
# 	# Publica valores de eletroestimulação a serem lidos pelo estimulador
# 	pubStim = rospy.Publisher('stimulator/ccl_update', Stimulator, queue_size=10)
#
# 	while not breaker:
# 		pass
#
# 	stimMsg.pulse_current = [0]
# 	stimMsg.pulse_width = [0]
# 	pubStim.publish(stimMsg)
#
# 	print(maxAngle)
#
#

if __name__ == '__main__':
	channel_scan = ChannelScanner()
	channel_scan.channelScanner()
