import rospy
import time
import threading
import mavros.srv
import mavros.msg
import mavros.command
import mavros.setpoint
from math import *
from mavros.utils import *
from tf.transformations import quaternion_from_euler



def setposition(dummy):
    set_mode = rospy.ServiceProxy('/mavros/set_mode', mavros.srv.SetMode)  
    # print "set mode: STABILIZE", set_mode(208,'')
    print "set mode: GUIDED", set_mode(216,'')
    rate = rospy.Rate(10)
    count = 0
    att_pose = mavros.setpoint.get_pub_attitude_pose()
    pos_local = mavros.setpoint.get_pub_position_local(queue_size=10)
    while(True):
        msg = mavros.setpoint.PoseStamped(
            header=mavros.setpoint.Header(
                frame_id="att_pose",
                stamp=rospy.Time.now()),
            )

        msg.pose.position.x = 0
        msg.pose.position.y = 0
        msg.pose.position.z = 2

        yaw_degrees = 20  # North
        yaw = radians(yaw_degrees)
        quaternion = quaternion_from_euler(0, 0, yaw)
        msg.pose.orientation = mavros.setpoint.Quaternion(*quaternion)

        att_pose.publish(msg)
        count+=1
        if (count%1000 == 0):
            print "Publish an att_pose once"


def offboard_mode(dummy):
    time.sleep(10)
    mavros.command.arming(True)
    set_mode = rospy.ServiceProxy('/mavros/set_mode', mavros.srv.SetMode)  
    print "set mode: ", set_mode(0,'OFFBOARD')



def start():
    rate = rospy.Rate(10) # 10Hz
    mavros.command.setup_services();

    rospy.loginfo("Start program")
    set_mode = rospy.ServiceProxy('/mavros/set_mode', mavros.srv.SetMode)  
    print "set mode: ", set_mode(0,'')
    # manual mode
    # command_Long = rospy.ServiceProxy('/mavros/cmd/command', mavros.srv.CommandLong)  
    # print "set mode: ", command_Long(176, 0, 64, 0, 0, 0, 0, 0, 0)
    
    

    rate.sleep()





if __name__ == '__main__':
    try:
        print "init_node"
        rospy.init_node('OFFBOARD', anonymous=True)
        print "Start()"
        start()
        time.sleep(5)
        print "Start thread setposition"
        thread_setposition = threading.Thread(target=setposition, args=(0,))
        # threads.append(thread_setposition)
        thread_setposition.start()

        print "Start thread offboard_mode"
        thread_offboard = threading.Thread(target=offboard_mode, args=(0,))
        # threads.append(thread_offboard)
        # thread_offboard.start()

    except rospy.ROSInterruptException:
        pass




# def start():
#     rate = rospy.Rate(10) # 10hz
#     mavros.command.setup_services();

#     msg = mavros.msg.OverrideRCIn()
#     msg.channels[8] = -1
#     pub = rospy.Publisher('/mavros/rc/override', mavros.msg.OverrideRCIn, queue_size=10)
#     pub.publish(msg)
#     rate.sleep()

#     command_Long = rospy.ServiceProxy('/mavros/cmd/command', mavros.srv.CommandLong)  
#     print "set mode: ", command_Long(176, 0, 64, 0, 0, 0, 0, 0, 0);

#     # set_mode = rospy.ServiceProxy('/mavros/set_mode', mavros.srv.SetMode)  
#     # print "set mode: ", set_mode(80,'MAV_MODE_STABILIZE_DISARMED')
    

#     #rosservice call /mavros/cmd/arming True
#     # arm = rospy.ServiceProxy('/mavros/cmd/arming', mavros.srv.CommandBool)  
#     print "arm:", mavros.command.arming(True)

#     print "set mode: ", command_Long(176, 0, 216, 0, 0, 0, 0, 0, 0);
#     # set_mode(216,'MAV_MODE_GUIDED_ARMED')            

#     # msg = mavros.msg.OverrideRCIn()
#     # msg.channels[3] = 1700
#     # pub = rospy.Publisher('/mavros/rc/override', mavros.msg.OverrideRCIn, queue_size=10)
#     # print "rc 3 1700:", pub.publish(msg)
#     rate.sleep()

# def takeoff():
#     takeoff = rospy.ServiceProxy('/mavros/cmd/takeoff', mavros.srv.CommandTOL)  
#     print "takeoff: ", takeoff(0,0,0,0,40)
    

# def land():
#     command_Long = rospy.ServiceProxy('/mavros/cmd/command', mavros.srv.CommandLong)  
#     print "set mode: ", command_Long(176, 0, 216, 0, 0, 0, 0, 0, 0);
#     # set_mode = rospy.ServiceProxy('/mavros/set_mode', mavros.srv.SetMode)  
#     # print "set mode: ", set_mode(216,'MAV_MODE_GUIDED_ARMED')

#     land = rospy.ServiceProxy('/mavros/cmd/land', mavros.srv.CommandTOL)  
#     print "land: ", land(0,0,0,0,0)

 
# if __name__ == '__main__':
#     rospy.init_node('pose', anonymous=True)
#     start()
#     takeoff()
#     time.sleep(20)
#     land()