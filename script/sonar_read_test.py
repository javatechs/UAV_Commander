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

timer = TimeCounter()

def isr_rx_callback(args):
	timer.click()


def main():
	tx_pin = mraa.Gpio(6)
	

	rx_pin = mraa.Gpio(7)

	tx_pin.dir(mraa.DIR_OUT)
	rx_pin.dir(mraa.DIR_IN)
	rx_pin.isr(mraa.EDGE_BOTH, isr_rx_callback, isr_rx_callback)

	while(True):
		tx_pin.write(0)
		time.sleep(5/S_TO_US)
		tx_pin.write(1)
		time.sleep(10/S_TO_US)
		tx_pin.write(0)

		# waiting for reading
		time.sleep(50/S_TO_MS)
		duration = timer.get_duration()
		if (duration==-1):
			continue
			print "Out of range!"
		else:
			print "The distance is: {0}".format(duration*165.7/S_TO_US)
			timer.reset()
		time.sleep(50/S_TO_MS)


if __name__ == '__main__':
	main()
