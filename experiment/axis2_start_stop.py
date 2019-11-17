#! /usr/bin/env python3

name = "axis2_start_stop"

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

use_axis = 'xy'


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



xspeed = 200000
yspeed = 5000
xstep = 1
ystep = 1
#pub[use_axis]['set_speed'].publish(speed)
xd = std_msgs.msg.Float64MultiArray()
xd.data = [xspeed, xstep]

yd = std_msgs.msg.Float64MultiArray()
yd.data = [yspeed, ystep]

pub['x']['start'].publish(xd)
pub['y']['start'].publish(yd)
time.sleep(5)
pub['x']['stop'].publish(1)
pub['y']['stop'].publish(1)
time.sleep(5)

logger.stop()
