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
        self.current_moving = {ax: 0 for ax in self.use_axis}
        self.last_direction = {ax: 0 for ax in self.use_axis}
        self.do_status = [0, 0, 0, 0]

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
            self.pub[ax+'_change_step'] = rospy.Publisher(b+'internal/change_step', std_msgs.msg.Int64, queue_size=1)
            continue
        self.pub['output_do'] = rospy.Publisher('{base}/output_do'.format(**locals()), std_msgs.msg.Int64MultiArray, queue_size=1)

        # create subscrivers
        for ax in self.use_axis:
            b = '{base}/{ax}/'.format(**locals())
            rospy.Subscriber(b+'step_cmd', std_msgs.msg.Int64, self.set_step, callback_args=ax)
            rospy.Subscriber(b+'speed_cmd', std_msgs.msg.Float64, self.set_speed, callback_args=ax)

            rospy.Subscriber(b+'speed', std_msgs.msg.Float64, self.get_speed, callback_args=ax)
            rospy.Subscriber(b+'step', std_msgs.msg.Int64, self.get_step, callback_args=ax)
            rospy.Subscriber(b+'moving', std_msgs.msg.Int64, self.get_moving, callback_args=ax)
            continue
        for do_num in range(1,5):
            rospy.Subscriber('{}/output_do{}_cmd'.format(base, do_num), std_msgs.msg.Int64, self.set_do, callback_args=do_num)


    def set_speed(self, speed, ax):
        if abs(speed.data) < self.low_speed[ax]:
            #pub stop
            self.pub[ax+'_stop'].publish(1)
            while self.current_moving[ax] != 0:
                time.sleep(10e-5)
            self.last_direction[ax] = 0
            return

        if self.move_mode[ax] == 'jog':
            if (self.last_direction[ax] * speed.data > 0) & (self.current_moving[ax] != 0):
                #pub change_speed
                self.pub[ax+'_change_speed'].publish(abs(speed.data))
                pass

            else:
                #pub stop
                self.pub[ax+'_stop'].publish(1)
                while self.current_moving[ax] != 0:
                    print(1)
                    time.sleep(10e-5)

                if speed.data > 0:
                    step = +1
                    pass

                else:
                    step = -1
                    pass

                speed_step = [abs(speed.data), step]
                self.last_direction[ax] = step

                #pub start
                speed_step_array = std_msgs.msg.Float64MultiArray()
                speed_step_array.data = speed_step
                self.pub[ax+'_start'].publish(speed_step_array)
                pass
            pass

        else:
            pass

        print(speed.data,ax,self.last_direction[ax])
        return


    def set_step(self, step, ax):
        if self.move_mode[ax] == 'ptp':
            if self.current_moving[ax] != 0:
                #pub change_step
                self.pub[ax+'_change_step'].publish(step.data)
            else:
                speed_step = [abs(self.default_speed[ax]), step.data]
                speed_step_array = std_msgs.msg.Float64MultiArray()
                speed_step_array.data = speed_step
                self.pub[ax+'_start'].publish(speed_step_array)
                pass
        else: pass
        return


    def set_do(self, do, do_num):
        self.do_status[do_num-1] = do.data
        _do = std_msgs.msg.Int64MultiArray()
        _do.data = self.do_status
        self.pub['output_do'].publish(_do)
        return


    def get_speed(self, speed, ax):
        self.current_speed[ax] = speed.data
        return


    def get_step(self, step, ax):
        self.current_step[ax] = step.data
        return


    def get_moving(self, moving, ax):
        self.current_moving[ax] = moving.data
        return
