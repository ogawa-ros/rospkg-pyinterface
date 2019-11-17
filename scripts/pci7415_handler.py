#! /usr/bin/env python3

import time

import pyinterface
import threading
import rospy

import std_msgs.msg


class pci7415_handler(object):

    def __init__(self, rsw_id, params):
        self.rsw_id = rsw_id
        #use axis
        self.use_axis = ''.join([ax for ax in params])
        self.current_speed = {ax: 0 for ax in self.use_axis}
        self.current_step = {ax: 0 for ax in self.use_axis}
        self.last_direction = {ax: 0 for ax in self.use_axis}

        self.move_mode = {ax: p['mode'] for ax, p in params.items()}
        self.default_speed = {ax: p['motion']['speed'] for ax, p in params.items()}
        self.low_speed = {ax: p['motion']['low_speed'] for ax, p in params.items()}

        base = '/pyinterface/pci7415/rsw{rsw_id}'.format(**locals())
        # create publishers
        self.pub = {}
        for ax in self.use_axis:
            b = '{base}/{ax}/'.format(**locals())
            self.pub[ax+'_stop'] = rospy.Publisher(b+'internal/stop', std_msgs.msg.Int64, queue_size=1)
            self.pub[ax+'_start'] = rospy.Publisher(b+'internal/start', std_msgs.msg.Float64MultiArray, queue_size=1)
            self.pub[ax+'_change_speed'] = rospy.Publisher(b+'internal/change_speed', std_msgs.msg.Float64, queue_size=1)
            self.pub[ax+'_change_step'] = rospy.Publisher(+b+'internal/change_step', std_msgs.msg.Int64, queue_size=1)
            continue
        self.pub['ouput_do'] = rospy.Publisher('{base}/output_do'.format(**locals()), std_msgs.msg.Int64MultiArray, queue_size=1)

        # create subscrivers
        for ax in self.use_axis:
            b = '{base}/{ax}/'.format(**locals())
            rospy.Subscriber(b+'step_cmd', std_msgs.msg.Int64, self.set_speed, callback_args=ax)
            rospy.Subscriber(b+'speed_cmd', std_msgs.msg.Int64, self.set_step, callback_args=ax)

            rospy.Subscriber(b+'speed', std_msgs.msg.Int64, self.get_speed, callback_args=ax)
            rospy.Subscriber(b+'step', std_msgs.msg.Int64, self.get_step, callback_args=ax)
            continue


    def set_speed(self, speed, ax):
        if abs(speed.data) < self.low_speed[ax]:
            #pub stop
            self.pub[ax+'_stop'].publish(1)
            while self.current_speed[ax] != 0:
                time.sleep(10e-5)
            return

        if self.move_mode[ax] == 'jog':
            if (self.last_speed[axis] * param > 0) & (self.current_speed[ax] != 0):
                #pub change_speed
                self.pub[ax+'_change_speed'].publish(abs(speed.data))
                pass
            else:
                #pub stop
                self.pub[ax+'_stop'].publish(1)
                while self.current_speed[ax] != 0:
                    time.sleep(10e-5)

                if param > 0: step = +1
                else: step = -1

                speed_step = [abs(speed.data), step]
                self.last_direction[axis] = step
                #pub start
                self.pub[ax+'_start'].publish(speed_step)
                pass
        else: pass
        return


    def set_step(self, step, ax):
        if self.move_mode[ax] == 'ptp':
            if self.current_speed[ax] != 0:
                #pub change_step
                self.pub[ax+'_change_step'].publish(step.data)
            else:
                speed_step = [abs(self.default_speed[ax], step)]
                self.pub[ax+'_start_motion'].publish(speed_step)
                pass
        else: pass
        return


    def set_do(self, do):
        _do = std_msgs.msg.Int64MultiArray()
        _do.data = do.data
        pub['output_do'].publish(_do)
        return


    def get_speed(self, speed, ax):
        self.current_speed[axis] = speed.data
        return


    def get_step(self, step, ax):
        self.current_step[axis] = step.data
        return
