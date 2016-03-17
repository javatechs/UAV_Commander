import mraa
import time

x = mraa.Gpio(33)
x.dir(mraa.DIR_IN)

while(True):
    v = x.read()
    print "The reading is: {}".format(v)   
    time.sleep(1)


