#! /usr/bin/env python3

name = "change_speed_motion"

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

#use_axis = input("which use axis ? = ")
use_axis = "x"

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

#speed = input("speed = ")
speed = 100000
#step = input("step = ")
step = 1
#acc = input("acc =")
acc = 50
#dec = input("dec =")
dec = 50
#change_speed = input("change_speed = ")
change_speed = 300000

pub[use_axis]['set_speed'].publish(float(speed))
pub[use_axis]['set_step'].publish(int(step))
pub[use_axis]['set_acc'].publish(int(acc))
pub[use_axis]['set_dec'].publish(int(dec))
pub[use_axis]['start'].publish(1)

time.sleep(5)

pub[use_axis]['change_speed'].publish(float(change_speed))

#global rspeed
#while rspeed != float(change_speed):
    #time.sleep(1e-3)
    #continue

time.sleep(5)

pub[use_axis]['stop'].publish(1)

logger.stop()
