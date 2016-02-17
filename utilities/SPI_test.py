#!/usr/bin/env python
import sys
sys.path.append('/usr/local/lib/i386-linux-gnu/python2.7/site-packages/')

import mraa as m
import random as rand
import time

dev = m.Spi(0)




dev = m.Spi(0)
print "SPI mode is: {}".format(dev.mode(0))
dev.frequency(1000000)

while(True):
	txbuf = bytearray("HELLO\n".encode('ascii'))
	# txbuf[0] = 'A'
	# txbuf[1] = 'W'
	# txbuf[2] = 'C'
	# txbuf[3] = 'e'
	# txbuf[4] = '\n'

	dev.write(txbuf)
	print "Tx sent: {}".format(txbuf)
	time.sleep(0.1)
	


# while(True):
#   txbuf = bytearray(4)
#   for y in range(0,4):
#     txbuf[y] = rand.randrange(0, 256)
#   rxbuf = dev.write(txbuf)
#   #if rxbuf != txbuf:
#     #print("We have an error captain!")
#     ##break
#     #exit(1)
#   time.sleep(0.2)






