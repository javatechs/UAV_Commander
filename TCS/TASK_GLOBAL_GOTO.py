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

# Declare some global variables.
# current position
current_position = TCS_util.vector3()
# setpoint message. The type will be changed later in main()
setpoint_msg = 0
# setpoint position
setpoint_position = TCS_util.vector3()
# precision setup. Need to tweak this parameter a bit to have a stable GPS flight
precision = 0.005
# setup frame_id
frame_id='GLOBAL_GOTO'

def set_target(msg, x, y, z):
    """A wrapper assigning the x,y,z values
    """
    msg.latitude = x
    msg.longitude = y
    msg.altitude = z
    pose.header=mavros.setpoint.Header(
        frame_id="global_pose", #?
        stamp=rospy.Time.now())

def position_cb(data):
    """position subscriber callback function
    """
    print data.header.frame_id
    current_position.x = data.latitude
    current_position.y = data.longitude
    current_position.z = data.altitude

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

   

def main():
    # print "TASK: "+str(sys.argv)
    # setup rospy env
    rospy.init_node('TCS_task', anonymous=True)
    rate = rospy.Rate(20)
    mavros.set_namespace('/mavros')
   
    # setup publisher 
    setpoint_pub = rospy.Publisher(mavros.get_topic('setpoint_raw','global'), queue_size=10)

    # setup setpoint_msg
    setpoint_msg = mavros.msg.GlobalPositionTarget(
            header=std_msgs.msg.Header(
                frame_id="global_pose",
                stamp=rospy.Time.now()),
            )

    # setup subscriber
    position_local_sub = rospy.Subscriber(mavros.get_topic('global_position', 'global'),
    	sensor_msgs.msg.NavSatFix, position_cb)

    # setup task pub
    task_watchdog = TCS_util.Task_watchdog('TASK_GLOBAL_GOTO')

    # interprete the input position
    setpoint_arg = sys.argv[1].split(' ')
    setpoint_position.x=float(setpoint_arg[0])
    setpoint_position.y=float(setpoint_arg[1])
    setpoint_position.z=float(setpoint_arg[2])
    print "Latitude: {}, Longitude: {}, Altitude: {}".format(setpoint_position.x,
    	setpoint_position.y, setpoint_position.z)

    # setup setpoint poisiton and prepare to publish the position
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

