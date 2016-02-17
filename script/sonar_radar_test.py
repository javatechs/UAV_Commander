#!/usr/bin/env python
# this script is used to send estimate barrier distance to companion computer to avoide collision

# import ros and msgs
import rospy
import mavros
import std_msgs.msg 
import mavros.utils

# import utils
import time
from datetime import datetime
import math

def main():
    # register a new node
    rospy.init_node('dart_radar_distance', anonymous=True)

    distance_pub = rospy.Publisher('/dart_radar_distance', std_msgs.msg.Float32, queue_size=10)
    rate = rospy.Rate(5)

    while not rospy.is_shutdown():
        distance = 0.0
        raw = raw_input("Test program:\nThe drone will try to keep 10m away from 'ground'\n \
            which the distance is given by user. \n \
            Please input estimate distance: ")
        if len(raw)==0:
            distance = 0.0
        elif (raw == 'q'):
            exit()
        else:
            distance = float(raw)
        distance_pub.publish(distance)
        # rospy.spinO()
        rate.sleep()

if __name__=='__main__':
    main()