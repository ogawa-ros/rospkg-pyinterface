#! /usr/bin/env python3

import rospy
import pyinterface

class pci7415_driver(object):

    def __init__(self, rsw_id, params):

        self.rsw_id = rsw_id
        self.use_axis = ''.join([p['axis'] for p in params])

        self.params = {}
        for p in params:
            self.params[p['axis']] = p
            continue

        # initialize motion controller
        self.mot = pyinterface.open(7415, rsw_id)
        self.mot.initialize()
        self.mot.output_do(p['do_conf'])
        [self.mot.set_pulse_out(p['axis'], 'method', p['pulse_conf']) for p in params]
        [self.mot.set_motion(p['axis'], p['mode'], p['motion']) for p in params]
        self.last_direction_dict = {p['axis']: 0 for p in params}

        rospy.Subscriber()
        return

    def set_motion(self):
        pass

    def start_motion(self):
        pass

    def stop_motion(self):
        pass

    def change_speed(self):
        pass

    def change_step(self):
        pass
