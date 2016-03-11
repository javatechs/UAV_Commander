#!/usr/bin/env python

import rospy
import mavros
import mavros.mavlink as mavlink
import mavros_msgs.msg as msg

from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget

from mavros_msgs.msg import Mavlink

# See https://pixhawk.ethz.ch/mavlink/#MAV_SEVERITY
MAV_SEVERITY_INFO = 6

mav_pub = None

#def CallBackLocal(data):
#    print 'local_position callbacked.'
#    print data.pose.position

def CallBackSP(data):
    print 'setpoint_raw callbacked.'
    print data

def CallBackLocal(data):
    print 'gps_position callbacked.'
    pose = data.pose.position
    #publish to GCS
    s = "[GPS Coordinate] x: %f, y: %f, z: %f" % (pose.x, pose.y, pose.z)
    print s
    bstr = bytearray(s)
    global MAV_SEVERITY_INFO
    bstr.insert(0,MAV_SEVERITY_INFO)
    pl64 = mavlink.convert_to_payload64(bstr)
    m = msg.Mavlink()
    m.header.stamp = rospy.Time.now()
    m.len = len(pl64)
    m.sysid = 1
    m.compid = 250
    m.msgid = 253 #STATUSTEXT 
    m.payload64 = pl64
    
    global mav_pub
    mav_pub.publish(m)


#rospy.Subscriber(mavros.get_topic('setpoint_raw', 'target_local'), PositionTarget, CallBackSP)

#rospy.Subscriber(mavros.get_topic('global_position', 'local'), PoseWithCovarianceStamped, CallBackGPS)



def main():
    
    rospy.init_node('TestGPS', anonymous=True)
    mavros.set_namespace('/mavros')
    
    # Test script
    rospy.Subscriber(mavros.get_topic('local_position', 'pose'), PoseStamped,CallBackLocal)
    
    global mav_pub
    mav_pub = rospy.Publisher(mavros.get_topic('gcs_bridge','to'), msg.Mavlink)
    
    
    while(True):
        pass

if __name__ == '__main__':
    main()
