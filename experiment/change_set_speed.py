#! /usr/bin/env python3

name = "change_set_speed"

import sys
import time
import numpy
import math
import os
import datetime
sys.path.append("/home/exito/ros/src/necst-core/scripts")
import core_controller
import rospy

import std_msgs.msg

rospy.init_node(name)

logger = core_controller.logger()

date = datetime.datetime.today().strftime('%Y%m%d_%H%M%S')
file_name = name + '/' + date + '.necstdb'
print(file_name)

pub = {}
base = '/pyinterface/pci7415/rsw0'

use_axis = 'x'


def read_speed(q):
    global rspeed
    rspeed = q.data
    return

def read_counter(q):
    rstep = q.data
    return


pub_outputdo =rospy.Publisher(base+'/output_do', std_msgs.msg.Int64MultiArray, queue_size=1)
for ax in use_axis:
    b = '{base}/{ax}/'.format(**locals())
    pub[ax] = {}
    pub[ax]['start'] = rospy.Publisher(b+'internal/start', std_msgs.msg.Float64MultiArray, queue_size=1)
    pub[ax]['stop'] = rospy.Publisher(b+'internal/stop', std_msgs.msg.Int64, queue_size=1)
    pub[ax]['change_speed'] = rospy.Publisher(b+'internal/change_speed', std_msgs.msg.Float64, queue_size=1)
    pub[ax]['change_step'] = rospy.Publisher(b+'internal/change_step', std_msgs.msg.Int64, queue_size=1)
    rospy.Subscriber(b+'speed', std_msgs.msg.Float64, read_speed, queue_size=1)
    rospy.Subscriber(b+'step', std_msgs.msg.Float64, read_counter, queue_size=1)

    time.sleep(0.1)

logger.start(file_name)
time.sleep(1)
conf = std_msgs.msg.Int64MultiArray()
conf.data = [1,1,1,1]
pub_outputdo.publish(conf)



speed1 = 100000
speed2 = 50000
step = 1
#pub[use_axis]['set_speed'].publish(speed)
d1 = std_msgs.msg.Float64MultiArray()
d1.data = [speed1, step]

d2 = std_msgs.msg.Float64MultiArray()
d2.data = [speed2, step]

for i in range(8):
    pub[use_axis]['start'].publish(d1)
    time.sleep(5)
    pub[use_axis]['stop'].publish(1)
    time.sleep(5)

    #pub[use_axis]['set_step'].publish(step2)
    #time.sleep(1e-3)
    pub[use_axis]['start'].publish(d2)
    time.sleep(5)
    pub[use_axis]['stop'].publish(1)
    time.sleep(5)

    continue

logger.stop()
