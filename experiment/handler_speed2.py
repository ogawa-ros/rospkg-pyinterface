#! /usr/bin/env python3

name = "handler_speed"

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

for ax in use_axis:
    b = '{base}/{ax}/'.format(**locals())
    pub[ax] = {}
    pub[ax]['step'] = rospy.Publisher(b+'step_cmd', std_msgs.msg.Int64, queue_size=1)
    pub[ax]['speed'] = rospy.Publisher(b+'speed_cmd', std_msgs.msg.Float64, queue_size=1)
pub_outputdo =rospy.Publisher(base+'/output_do', std_msgs.msg.Int64MultiArray, queue_size=1)

    time.sleep(0.1)

logger.start(file_name)
conf = std_msgs.msg.Int64MultiArray()
conf.data = [1,1,1,1]
pub_outputdo.publish(conf)


cycle = 1
max_speed = 300000
sec = 10

test_speed = numpy.sin(numpy.linspace(0, 2*cycle*math.pi, sec/0.1))*max_speed
test_speed = numpy.cos(numpy.linspace(0, 2*cycle*math.pi, sec/0.1))*max_speed

for i in test_speed:
    pub[use_axis[0]]['speed'].publish(i)
    pub[use_axis[1]]['speed'].publish(i)
    time.sleep(0.1)

pub[use_axis[0]]['speed'].publish(0)
pub[use_axis[1]]['speed'].publish(0)

logger.stop()
