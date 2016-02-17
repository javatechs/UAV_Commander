#!/usr/bin/env python
# TX_PIN: J17-7
# RX_PIN: J17-8
# GND PIN: J19-3

import mraa
from datetime import datetime
import time

S_TO_US = 1000000
S_TO_MS = 1000

class TimeCounter:
	def __init__(self):
		self.duration = 0
		self.OK=False

	def reset(self):
		self.duration = 0
		self.OK=False

	def click(self):
		if (self.duration == 0):
			# just been reset
			# record time in us
			self.duration = int(datetime.utcnow().strftime('%f'))
		else:
			tmp = int(datetime.utcnow().strftime('%f'))
			if tmp < self.duration:
				# from 0.9s to 1s
				self.duration = 1000000 - self.duration + tmp
			else:
				self.duration = tmp - self.duration
			self.OK = True

	def get_duration(self):
		if (not self.OK):
			return -1
		else:
			return self.duration



class Sonar:
	def __init__(self):
		self.timer = TimeCounter()

	def set_pin(self, tx_gpio=6, rx_gpio=7):
		self.tx_pin = mraa.Gpio(tx_gpio)
		self.rx_pin = mraa.Gpio(rx_gpio)
		self.tx_pin.dir(mraa.DIR_OUT)
		self.rx_pin.dir(mraa.DIR_IN)
		self.rx_pin.isr(mraa.EDGE_BOTH, self.isr_rx_callback, self.isr_rx_callback)

	def isr_rx_callback(self, args):
		self.timer.click()

	def read_distance(self):
		self.tx_pin.write(0)
		self.time.sleep(5/S_TO_US)
		self.tx_pin.write(1)
		time.sleep(10/S_TO_US)
		self.tx_pin.write(0)

		# waiting for reading
		time.sleep(50/S_TO_MS)
		duration = self.timer.get_duration()
		if (duration==-1):
			timer.reset()
			return -1		
		else:
			timer.reset()
			return duration*165.7/S_TO_US

mysonar = Sonar()
