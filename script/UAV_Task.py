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
import serial
from datetime import datetime

# import mraa
import sys
import mraa
sys.path.append('/usr/local/lib/i386-linux-gnu/python2.7/site-packages/')


class vector3(object):
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0


class Task_GOTO_Local(object):
    def __init__(self, setpoint_publish):
        self.setpoint_pub = setpoint_publish
        self.current_sub = rospy.Subscriber(mavros.get_topic('local_position', 'pose'),
            SP.PoseStamped, self._local_position_callback)
        self.current = vector3()
        self.msg = mavros.setpoint.PoseStamped(
            header=mavros.setpoint.Header(
                frame_id="att_pose",
                stamp=rospy.Time.now()),
            )
        

        self.x = 0
        self.y = 0
        self.z = 0
        # default precision is 0.5
        self.precision = 0.5    

    def goto(self, x=0, y=0, z=0, xyz=None):
        if (xyz==None):
            self.x = x
            self.y = y
            self.z = z
        else:
            self.x = xyz.x
            self.y = xyz.y
            self.z = xyz.z
        self._set_pose(self.msg, x, y, z)
        self.setpoint_pub.publish(self.msg)

    def check_task(self):
        if self.is_reached(self.current, self.msg):
            return True
        else:
            return False

    def is_reached(self, current, setpoint):
        if (abs(current.x-setpoint.pose.position.x) < self.precision and
            abs(current.y-setpoint.pose.position.y) < self.precision and
            abs(current.z-setpoint.pose.position.z) < self.precision):
            print "Point reached!"
            return True
        else:
            return False

    def _set_pose(self, pose, x, y, z):
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.header=mavros.setpoint.Header(
                    frame_id="att_pose",
                    stamp=rospy.Time.now())

    def _local_position_callback(self,topic):
        self.current.x = topic.pose.position.x
        self.current.y = topic.pose.position.y
        self.current.z = topic.pose.position.z

    def set_precision(self, value):
        self.precision = value

    def get_setpoint_pub(self):
        return self.setpoint_pub

    def get_current(self):
        return self.current


class Task_Stay(Task_GOTO_Local):
    def __init__(self, setpoint_publish):
        super(Task_Stay, self).__init__(setpoint_publish)
        self.timer = rospy.Time.now()
        self.stay_pose = vector3();
        self.timer_ready = False
        self.duration = 0

    def stay_at_time(self, duration):
        # if timer is not being reset, reset the timer
        if (self.init_ready==False):
            print "First enter stay stay_at_time"
            self.timer = rospy.Time.now()
            self.init_ready = True
            self.stay_pose.x = self.current.x
            self.stay_pose.y = self.current.y
            self.stay_pose.z = self.current.z
            

        self.goto(self.stay_pose.x,
            self.stay_pose.y,
            self.stay_pose.z)
        self.duration = duration

    def stay(self):
        if(self.init_ready==False):
            self.init_ready = True
            # need hard copy here because simple self.stay_pose = self.current will give reference to 
            # self.stay_pose and then self.stay_pose will change following the self.current
            self.stay_pose.x = self.current.x
            self.stay_pose.y = self.current.y
            self.stay_pose.z = self.current.z

        self.goto(self.stay_pose.x,
            self.stay_pose.y,
            self.stay_pose.z)

    def reset_stay(self):
        # need to reset say flag after use
        self.init_ready = False

    def check_task(self):
        if (rospy.Time.now() - self.timer > rospy.Duration(self.duration)):
            return True
        else:
            return False

    def print_current(self):
        print "x = {} y = {} z = {}".format(self.current.x, self.current.y, self.current.z)


class Task_GOTO_GPS(object):
    '''
    This is task that guide the UAV to GPS position
    '''
    pass


class API_XBee(object):
    '''
    This is the interface to the XBee via simple transparency grammer
    '''
    def __init__(self, PAN, ID):
        self.pan = PAN
        self.ID = ID
    def read(self):
        pass
    def _setID(self):
        pass
    def _setTargetID(self):
        pass
    def _serialize(self):
        pass
    def _deserialize(self):
        pass
    pass

class API_AirTemp(API_XBee):
    '''
    This is the interface to air temperature in ground data logger
    '''
    def read(self):
        pass
    def _serialize(self):
        pass
    def _deserialize(self):
        pass
    pass

class API_SoilTemp(API_XBee):
    '''
    This is the interface to Soil moisture sensor from ground data logger
    '''
    def read(self):
        pass
    def _serialize(self):
        pass
    def _deserialize(self):
        pass
    pass

class API_Moisture(API_XBee):
    '''
    This is interface to soil moisture sensor from  ground data logger
    '''
    def read(self):
        pass
    def _serialize(self):
        pass
    def _deserialize(self):
        pass
    pass



class Task_FakeDate(object):
    '''
        If data logger and data collect program cannot be finished in time
        which is very possible
    '''
    def __init__(self):
        from sensor_msgs.msg import Temperature
        rospy.init_node('fake_read_data', anonymous=True)
        self.pub = rospy.Publisher('/read_data', std_msgs.msg.Float32, queue_size=10)
        rospy.Subscriber("/mavros/imu/temperature", Temperature, self._callback)

    def run(self):
        if not rospy.is_shutdown():
            self.pub.publish(float(self.temp))

    def _callback(self, topic):
        self.temp = topic.temperature

class Task_GetData(object):
    '''
    This task is used to remotely get data from UAV.
    Please please make sure that all the opearations
    are not blocking. Blocking codes will kick UAV out
    of OFFBOARD mode
    '''
    def __init__(self):
        self.target = 0x00
        self.state = 'PREOPERATION'
        self.data = 0
        self.rxbuf = ""

        # delay related
        self.delay_state = 'clear'
        self.delay_duration = 0
        self.timestamp = 0

        self.spi = mraa.Spi(0)
        self.spi.mode(0)
        self.spi.frequency(1000000)    # 1MHz

    def run(self, ID, item):
        '''
        1. set target ID
        2. listen to feedback
        3. send data collecting command
        4. collect date
        '''
        if (self.state is 'PREOPERATION'):
            self._setTarget(ID)
            self.rxbuf = ""
            self.state = 'SETTARGETWAIT'
        elif (self.state is 'SETTARGETWAIT'):
            if (self._delay(5) == True):
                self.state = 'SETTARGETFB'
        elif(self.state is 'SETTARGETFB'):
            if ('Y' in self._read(10)):
                self.state = 'SENDCMD'
        elif(self.state is 'SENDCMD'):
            self._serialize("COLLECT")
            self.state = 'CMDWAIT'
        elif(self.state is 'CMDWAIT'):
            if(self._delay(5) == True):
                self.state = 'DATACOLLECT'
        elif(self.state is 'DATACOLLECT'):
            temp = self.spi.write_word(0x00)
            if (temp == '\n'):
                self.state = 'END'
                self.data = float(self.rxbuf)
            else:
                self.rxbuf.append(temp)
                if (len(self.rxbuf)>50):
                    self.data = 0
                    self.state = 'END'
        elif (self.state is 'END'):
            self.state = 'PREOPERATION'
            return self.data
        else:
            self.data = 0
            self.state = 'END'
        return False

        

    def get_date(self):
        self.state = 'PREOPERATION'
        return self.data

    def _delay(self, duration):
        if (self.delay_state is 'clear'):
            self.delay_time = rospy.Time.now()
            self.delay_state = 'pending'
            return False
        elif (self.delay_state is 'pending'):
            if (rospy.Time.now() - self.delay_time >= rospy.Duration(self.delay_duration)):
                return True
            else:
                return False

    def _clear_delay(self):
        self.delay_state = 'clear'


    def _read(self, len):
        temp = ""
        for i in range(0,len):
            temp.append(self.spi.write_word(0x00))

        return temp 

    def _setTarget(self, ID):
        self.target = ID
        self._serialize("ATDL "+str(ID))

    def _serialize(self, msg):
        txbuf = 'H'+ str(msg) + "\n"
        txbuf = bytearray(msg.encode('ascii'))
        self.spi.write(txbuf)

    pass