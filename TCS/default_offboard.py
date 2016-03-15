#!/usr/bin/env python

# import ROS libraries
import rospy
import mavros
from mavros.utils import *
from mavros import setpoint as SP
import mavros.setpoint
import mavros.command
import mavros_msgs.msg
import mavros_msgs.srv
import time
from datetime import datetime
from UAV_Task import *
import TCS_util
# import utilities
import thread
import math
import sys
import signal
import subprocess



def signal_handler(signal, frame):
        print('You pressed Ctrl+C!')
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)


current_pose = vector3()
UAV_state = mavros_msgs.msg.State()



def _state_callback(topic):
    UAV_state.armed = topic.armed
    UAV_state.connected = topic.connected
    UAV_state.mode = topic.mode
    UAV_state.guided = topic.guided



def _setpoint_position_callback(topic):
    pass

def _set_pose(pose, x, y, z):
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    pose.header=mavros.setpoint.Header(
                frame_id="att_pose",
                stamp=rospy.Time.now())



def is_reached(current, setpoint):
    if (abs(current.x-setpoint.pose.position.x) < 0.5 and
        abs(current.y-setpoint.pose.position.y) < 0.5 and
        abs(current.z-setpoint.pose.position.z) < 0.5):
        print "Point reached!"
        return True
    else:
        return False

def update_setpoint():
    pass


def main():
    rospy.init_node('default_offboard', anonymous=True)
    rate = rospy.Rate(20)
    mavros.set_namespace('/mavros')


    # setup subscriber
    # /mavros/state
    state_sub = rospy.Subscriber(mavros.get_topic('state'),
        mavros_msgs.msg.State, _state_callback)
    # /mavros/local_position/pose
    # local_position_sub = rospy.Subscriber(mavros.get_topic('local_position', 'pose'),
    #     SP.PoseStamped, _local_position_callback)
    # /mavros/setpoint_raw/target_local
    setpoint_local_sub = rospy.Subscriber(mavros.get_topic('setpoint_raw', 'target_local'),
        mavros_msgs.msg.PositionTarget, _setpoint_position_callback)

    # setup publisher
    # /mavros/setpoint/position/local
    setpoint_local_pub =  mavros.setpoint.get_pub_position_local(queue_size=10)

    # setup service
    # /mavros/cmd/arming
    set_arming = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool) 
    # /mavros/set_mode
    set_mode = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)  

    setpoint_msg = mavros.setpoint.PoseStamped(
            header=mavros.setpoint.Header(
                frame_id="att_pose",
                stamp=rospy.Time.now()),
            )

    #read task list
    Task_mgr = TCS_util.Task_manager('task_list.log')


    # wait for FCU connection
    while(not UAV_state.connected):
        rate.sleep()

    # initialize the setpoint
    setpoint_msg.pose.position.x = 0
    setpoint_msg.pose.position.y = 0
    setpoint_msg.pose.position.z = 3
    	
    mavros.command.arming(True)

    # send 100 setpoints before starting
    for i in range(0,50):
        setpoint_local_pub.publish(setpoint_msg)
        rate.sleep()

    set_mode(0,'OFFBOARD')

    # start setpoint_update instance
    setpoint_keeper = TCS_util.SetpointMonitor(rospy)

    last_request = rospy.Time.now()


    # enter the main loop
    while(True):
        # print "Entered whiled loop"
        if( UAV_state.mode != "OFFBOARD" and
            (rospy.Time.now() - last_request > rospy.Duration(5.0))):
            if( set_mode(0,'OFFBOARD').success):
                print "Offboard enabled"
            last_request = rospy.Time.now()
        else:
            if(not UAV_state.armed and
                (rospy.Time.now() - last_request > rospy.Duration(5.0))):
                if( mavros.command.arming(True)):
                    print "Vehicle armed"
                last_request = rospy.Time.now()

        # update setpoint to stay in offboard mode
        # does it work with global setpoints? Do we need global setpoints at all?
        setpoint_keeper.update(type='LOCAL')
        

        if(Task_mgr.task_finished()):
            # If the current task has been done
            rospy.loginfo("Current task is finished!")
            if (not Task_mgr.alldone()):
                # If there are tasks left
                Task_mgr.nexttask()

            else:
                # Current task has been done and no task left
                rospy.loginfo("All tasks have been done!")
                while (UAV_state.mode != "AUTO.LAND"):
                    set_mode(0,'AUTO.LAND')
                    rate.sleep()
                return 0

        rate.sleep()
    return 0




if __name__ == '__main__':
    main()
