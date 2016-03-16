#!/usr/bin/env python
'''
This is the TASK_GLOBAL_GOTO
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
import subprocess
import os
import platform
import numpy

import re

osname = platform.uname()[3]
if (bool(re.search('[uU]buntu', osname))):
    sys.path.append('/opt/ros/jade/lib/python2.7/dist-packages')
elif(platform.uname()[1]=='edison'):
    sys.path.append('/opt/ros/jade/lib/python2.7/dist-packages')
elif(platform.uname()[1]=='xiaoguang'):
    # this is workstation in 079 Lab
    sys.path.append('/opt/ros/jade/lib/python2.7/dist-packages')

# import ROS libraries
import rospy
import mavros
from mavros.utils import *
from mavros import setpoint as SP
import mavros.command
import mavros_msgs.msg
import mavros_msgs.srvi

import time
from datetime import datetime

import TCS_util
import Coordinate

# Declare some global variables.
# home position: GPS
home_position = TCS_util.vector3()
# indicate task as initiated
task_started = False
# current position: Local
current_position = TCS_util.vector3()
# setpoint message. The type will be changed later in main()
setpoint_msg = 0
# setpoint position: GPS
setpoint_position_global = TCS_util.vector3()
# setpoint position: local
setpoint_position = TCS_util.vector3()
# precision setup. Need to tweak this parameter a bit to have a stable GPS flight
precision = 0.5
# setup frame_id
frame_id='GLOBAL_GOTO'

def convertSPtoLocal():
    """https://en.wikipedia.org/wiki/Geographic_coordinate_conversion
    """
    global home_position
    lato,lono,alto = home_position.x, home_position.y, home_position.z
    homeECEF = Coordinate.globalToECEF(lato,lono,alto)

    global setpoint_position_global
    latsp,lonsp,altsp = setpoint_position_global.x, setpoint_position_global.y, setpoint_position_global.z
    spECEF = Coordinate.globalToECEF(latsp,lonsp,altsp)

    #Compute setpoint_position
    global setpoint_position
    setpoint_position = Coordinate.ECEFtoENU(spECEF,homeECEF,lato,lono)
    
    

def set_target(msg, x, y, z):
    """A wrapper assigning the x,y,z values
    """
    msg.pose.position = x
    msg.pose.position = y
    msg.pose.position = z
    pose.header=mavros.setpoint.Header(
        frame_id="global_pose", 
        stamp=rospy.Time.now())

def local_cb(data):
    current_position.x = data.pose.position.x
    current_position.y = data.pose.position.y
    current_position.z = data.pose.position.z

def is_reached():
    """Check if the UAV reached the destination
    """
    if (abs(current_position.x-setpoint_position.x) < precision and
            abs(current_position.y-setpoint_position.y) < precision and
            abs(current_position.z-setpoint_position.z) < precision):
        print "Point reached!"
        return True
    else:
        return False


def print_help():
    s = 'Usage: python TASK_GLOBAL_GOTO.py SP_LATITUDE SP_LONGITUDE SP_ALTITUDE HOME_LATITUDE HOME_LONGITUDE HOME_ALTITUDE'
    print s
   

def main():
    if(len(sys.argv) != 7):
        print_help()
        sys.exit(1)

    # print "TASK: "+str(sys.argv)
    # setup rospy env
    rospy.init_node('TCS_task', anonymous=True)
    rate = rospy.Rate(20)
    mavros.set_namespace('/mavros')
   
    # setup publisher 
    setpoint_pub = rospy.Publisher(mavros.get_topic('setpoint_raw','local'), queue_size=10)

    # setup setpoint_msg
    setpoint_msg = mavros.setpoint.PoseStamped(
            header=std_msgs.msg.Header(
                frame_id="local_pose",
                stamp=rospy.Time.now()),
            )

    # setup subscriber
    position_local_sub = rospy.Subscriber(mavros.get_topic('local_position', 'pose'),
        SP.PoseStamped, local_cb)

    # setup task pub
    task_watchdog = TCS_util.Task_watchdog('TASK_GLOBAL_GOTO')

    # interprete the input position
    global setpoint_position_global
    global home_position
    setpoint_position_global.x=float(sys.argv[2])
    setpoint_position_global.y=float(sys.argv[3])
    setpoint_position_global.z=float(sys.argv[4])
    home_position.x = float(sys.argv[5])
    home_position.y = float(sys.argv[6])
    home_position.z = float(sys.argv[7])

    print "Home Setpoint"
    print "Latitude: {}, Longitude: {}, Altitude: {}".format(home_position.x, home_position.y, home_position.z)

    print "Target Setpoint"
    print "Latitude: {}, Longitude: {}, Altitude: {}".format(setpoint_position_global.x, setpoint_position_global.y, setpoint_position_global.z)
    
    # convert global setpoint coordinate to local setpoint coordinate
    convertSPtoLocal()

    # setup setpoint poisiton and prepare to publish the position
    global setpoint_position
    set_target(setpoint_msg,
    	setpoint_position.x,
    	setpoint_position.y,
    	setpoint_position.z)
    
    # In this while loop, do the job.
    while(not is_reached()):
        # When the UAV reached the position, 
        # publish the task finished signal and exit
    	setpoint_local_pub.publish(setpoint_msg)
        # TODO: publish the task status as conducting
        task_watchdog.report_running()

        rate.sleep()

    # TODO: publish the task status as FINISHING
    task_watchdog.report_finish()

    return 0


if __name__ == '__main__':
    main()

