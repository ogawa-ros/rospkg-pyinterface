#! /usr/bin/env python3

name = "start_stop_motion"

import sys
import time
import numpy
import math
import os
import datetime
sys.path.append("/root/ros/src/necst-core/scripts")
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
    pub[ax]['start'] = rospy.Publisher(b+'internal/start', std_msgs.msg.Int64, queue_size=1)
    pub[ax]['stop'] = rospy.Publisher(b+'internal/stop', std_msgs.msg.Int64, queue_size=1)
    pub[ax]['set_speed'] = rospy.Publisher(b+'internal/set_speed', std_msgs.msg.Float64, queue_size=1)
    pub[ax]['set_step'] = rospy.Publisher(b+'internal/set_step', std_msgs.msg.Int64, queue_size=1)
    pub[ax]['set_acc'] = rospy.Publisher(b+'internal/set_acc', std_msgs.msg.Int64, queue_size=1)
    pub[ax]['set_dec'] = rospy.Publisher(b+'internal/set_dec', std_msgs.msg.Int64, queue_size=1)
    pub[ax]['change_speed'] = rospy.Publisher(b+'internal/change_speed', std_msgs.msg.Float64, queue_size=1)
    pub[ax]['change_step'] = rospy.Publisher(b+'internal/change_step', std_msgs.msg.Int64, queue_size=1)
    rospy.Subscriber(b+'speed', std_msgs.msg.Float64, read_speed, queue_size=1)
    rospy.Subscriber(b+'step', std_msgs.msg.Float64, read_counter, queue_size=1)

    time.sleep(0.1)

logger.start(file_name)
conf = std_msgs.msg.Int64MultiArray()
conf.data = [1,1,1,1]
pub_outputdo.publish(conf)

speed = 200000
step = 1
acc = 100
dec = 100

pub[use_axis]['set_speed'].publish(speed)
pub[use_axis]['set_step'].publish(step)
pub[use_axis]['set_acc'].publish(acc)
pub[use_axis]['set_dec'].publish(dec)
for i in range(10):
    pub[use_axis]['start'].publish(1)
    time.sleep(2)
    pub[use_axis]['stop'].publish(1)
    time.sleep(2)
    continue

logger.stop()
