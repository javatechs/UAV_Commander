#!/usr/bin/env python
import roslib
import rospy
import time
from sensor_msgs.msg import Temperature
def callback(data):
   rospy.loginfo(rospy.get_name()+ "I heard T=%s", data.temperature)

def listener():
   rospy.init_node('temp_listener', anonymous=True)
   rospy.Subscriber("/mavros/imu/temperature", Temperature, callback)
   rospy.spin()

if __name__ == '__main__':
   listener()