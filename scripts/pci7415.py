#! /usr/bin/env python3

import rospy

from .pci7415_handler import pci7415_handler
from .pci7415_driver import pci7415_driver

default_name = 'pci7415'
default_rsw_id = '0'
default_do_conf = "[0, 0, 0, 0]"
default_use_axis = 'xyzu'
default_pulse_conf = "{'PULSE': '0', 'OUT': '0', 'DIR': '0', 'WAIT': '0', 'DUTY': '0'}"
default_mode = 'ptp'
default_clock = 299
default_acc_mode = 'acc_normal'
default_low_speed = 200
default_speed = 10000
default_acc = 1000
default_dec = 1000
default_step = 0


if __name__ == '__main__':
    name = rospy.get_param('~node_name'.format(**locals()), default_name)
    rospy.init_node(name)

    rsw_id = rospy.get_param('~rsw_id', default_rsw_id)
    use_axis = rospy.get_param('~use_axis', default_use_axis) # ex. 'xyzu', 'xy', or 'yu'

    params = []
    for ax in use_axis:
        p = {}
        p['axis'] = ax
        p['mode'] = rospy.get_param('~{ax}_mode'.format(**locals()), default_mode)
        p['do_conf'] = eval(rospy.get_param('~do_conf', default_do_conf))
        p['pulse_conf'] = [eval(rospy.get_param('~{ax}_pulse_conf'.format(**locals()), default_pulse_conf))]

        mp = {}
        mp[ax] = {}
        mp[ax]['clock'] = rospy.get_param('~{ax}_clock'.format(**locals()), default_clock)
        mp[ax]['acc_mode'] = rospy.get_param('~{ax}_acc_mode'.format(**locals()), default_acc_mode)
        mp[ax]['low_speed'] = rospy.get_param('~{ax}_low_speed'.format(**locals()), default_low_speed)
        mp[ax]['speed'] = rospy.get_param('~{ax}_speed'.format(**locals()), default_speed)
        mp[ax]['acc'] = rospy.get_param('~{ax}_acc'.format(**locals()), default_acc)
        mp[ax]['dec'] = rospy.get_param('~{ax}_dec'.format(**locals()), default_dec)
        mp[ax]['step'] = rospy.get_param('~{ax}_step'.format(**locals()), default_step)
        p['motion'] = mp
        params.append(p)
        continue

    ctrl = pci7415_handler(rsw_id)
    driver = pci7415_driver()

    rospy.spin()