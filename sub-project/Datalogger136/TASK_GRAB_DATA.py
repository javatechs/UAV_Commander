#!/usr/bin/env python
'''
This is the TASK_LOCAL_GOTO
This task will guide the drone to the destination point 
by keeping sending setpoint_local
This task is able to check is_reached
once the UAV reached the point, the task will exit
TODO:
implement an appropriate platfrom recognition
implement task publisher that talking to main TCS process
that this TASK has finished its job.
'''

# import utilities
import math
import sys
import signal
import os
import platform

import re

osname = platform.uname()[3]
sys.path.append('/opt/ros/jade/lib/python2.7/dist-packages')

# import ROS libraries
import rospy
import mavros
from mavros.utils import *
from mavros import setpoint as SP
import mavros.command
import mavros_msgs.msg
import mavros_msgs.srv
import time
from datetime import datetime
import mraa




# setup frame_id
frame_id='GRAB_DATA'
read_done=False

COLLECT_PIN = mraa.Gpio(33)
COLLECT_PIN.dir(mraa.DIR_IN)

class READDONE:
    status=False

read_done=READDONE()

def falldetected(args):
    read_done.status = True

   
COLLECT_PIN.isr(mraa.EDGE_FALLING, falldetected, falldetected)

def main():
    print "start to read DATALOGGER!"
    # In this while loop, do the job.
    loop_counter = 4
    while(loop_counter>0):
        print "Prepare to read the collector {}!".format(4-loop_counter)
        while (read_done.status is not True):
            continue
    
        print "Read done!"
        print "Waiting 10 Second for the next collection!"
        time.sleep(10)
        read_done.status = False 
        # TODO: publish the task status as conducting
        loop_counter-=1

    # TODO: publish the task status as FINISHING
    print "Program END"
    sys.exit()
    


if __name__ == '__main__':
    main()

