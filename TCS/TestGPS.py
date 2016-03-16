#!/usr/bin/env python

import rospy
import mavros
import mavros.mavlink as mavlink
import mavros_msgs.msg as msg

from sesnor_msgs.msg import NavSatFix 
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

def CallBackGlobal(data):
    print 'gps_position callbacked.'
    lat = data.latitude
    lon = data.longitude
    alt = data.altitude
    #publish to GCS
    s = "[GPS Coordinate] x: %f, y: %f, z: %f" % (lat, lon, alt)
    print s

"""
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
"""

#rospy.Subscriber(mavros.get_topic('setpoint_raw', 'target_local'), PositionTarget, CallBackSP)

#rospy.Subscriber(mavros.get_topic('global_position', 'local'), PoseWithCovarianceStamped, CallBackGPS)



def main():
    
    rospy.init_node('TestGPS', anonymous=True)
    mavros.set_namespace('/mavros')
    
    # Test script
    rospy.Subscriber(mavros.get_topic('global_position', 'global'), NavSatFix,CallBackGlobal)
    
    global mav_pub
    mav_pub = rospy.Publisher(mavros.get_topic('gcs_bridge','to'), msg.Mavlink)
    
    
    while(True):
        pass

if __name__ == '__main__':
    main()
