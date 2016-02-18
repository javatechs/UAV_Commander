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
import subprocess
import os
import platform
if (platform.uname()[1]=='ubuntu'):
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
import mavros_msgs.srv
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
# precision setup. normally set it to 0.5m
precision = 0.5
# setup frame_id
frame_id='LOCAL_GOTO'

def set_target(pose, x, y, z):
    """A wrapper assigning the x,y,z values
    to the pose. pose usually is type of 
    mavros.setpoint.PoseStamped
    """
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    pose.header=mavros.setpoint.Header(
        frame_id="local_pose",
        stamp=rospy.Time.now())

def local_position_cb(topic):
    """local position subscriber callback function
    """
    current_position.x = topic.pose.position.x
    current_position.y = topic.pose.position.y
    current_position.z = topic.pose.position.z

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
    # setup local pub
    setpoint_local_pub = mavros.setpoint.get_pub_position_local(queue_size=10)

    # setup setpoint_msg
    setpoint_msg = mavros.setpoint.PoseStamped(
            header=mavros.setpoint.Header(
                frame_id="local_pose",
                stamp=rospy.Time.now()),
            )

    # setup local sub
    position_local_sub = rospy.Subscriber(mavros.get_topic('local_position', 'pose'),
    	SP.PoseStamped, local_position_cb)

    # setup task pub
    task_watchdog = TCS_util.Task_watchdog('TASK_LOCAL_GOTO')

    # interprete the input position
    setpoint_arg = sys.argv[1].split(' ')
    setpoint_position.x=float(setpoint_arg[0])
    setpoint_position.y=float(setpoint_arg[1])
    setpoint_position.z=float(setpoint_arg[2])
    print "X: {}, Y: {}, Z: {}".format(setpoint_position.x,
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

