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

use_axis = 'x'

for ax in use_axis:
    b = '{base}/{ax}/'.format(**locals())
    pub[ax] = {}
    pub[ax]['step'] = rospy.Publisher(b+'step_cmd', std_msgs.msg.Int64, queue_size=1)
    pub[ax]['speed'] = rospy.Publisher(b+'speed_cmd', std_msgs.msg.Float64, queue_size=1)

    time.sleep(0.1)

logger.start(file_name)
conf = std_msgs.msg.Int64MultiArray()
conf.data = [1,1,1,1]
pub_outputdo.publish(conf)

#=====================================================
test_speed = [1000, 2000, 4000, 8000, 16000, 32000]

for i in test_speed:
    pub[use_axis]['speed'].publish(i)
    time.sleep(1)

pub[use_axis]['speed'].publish(0)

logger.stop()
