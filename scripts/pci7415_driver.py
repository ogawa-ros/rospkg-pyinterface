#! /usr/bin/env python3

import time
import numpy
import queue
import threading

import rospy
import pyinterface

class pci7415_driver(object):

    def __init__(self, rsw_id, params):
        self.func_queue = queue.Queue()
        self.pub = {}
        self.use_axis = ''.join([ax for ax in params])

        self.params = params
        self.motion = {ax: params[ax]['motion'] for ax in self.use_axis}

        # initialize motion controller
        self.mot = pyinterface.open(7415, rsw_id)
        [self.mot.set_pulse_out(ax, 'method', params[ax]['pulse_conf']) for ax in self.use_axis]
        self.mot.set_motion(self.use_axis, params[ax]['mode'], self.motion)

        #Subscriber&Publisher
        base = '/pyinterface/pci7415/rsw{rsw_id}'.format(**locals())
        for ax in self.use_axis:
            b = '{base}/{ax}/'.format(**locals())
            rospy.Subscriber(b+'internal/start', std_msgs.msg.Int64, self.regist_start, callback_args=ax)
            rospy.Subscriber(b+'internal/stop', std_msgs.msg.Int64, self.regist_stop, callback_args=ax)
            rospy.Subscriber(b+'internal/set_speed', std_msgs.msg.Int64, self.regist_set_speed, callback_args=ax)
            rospy.Subscriber(b+'internal/set_step', std_msgs.msg.Int64, self.regist_set_step, callback_args=ax)
            rospy.Subscriber(b+'internal/set_acc', std_msgs.msg.Int64, self.regist_set_acc, callback_args=ax)
            rospy.Subscriber(b+'internal/set_dec', std_msgs.msg.Int64, self.regist_set_dec, callback_args=ax)
            rospy.Subscriber(b+'internal/change_speed', std_msgs.msg.Int64, self.regist_change_speed, callback_args=ax)
            rospy.Subscriber(b+'internal/change_step', std_msgs.msg.Int64, self.regist_change_step, callback_args=ax)
            self.pub[ax+'_speed'] = rospy.Publisher(b+'speed', std_msgs.msg.Float64, queue_size=1)
            self.pub[ax+'_step'] = rospy.Publisher(b+'step', std_msgs.msg.Float64, queue_size=1)
            continue

        time.sleep(0.5)

        # loop start
        self.th = threading.Thread(target=self.loop)
        self.th.setDaemon(True)
        self.th.start()
        return

    def loop(self):
        while not rospy.is_shutdown():
            t0 = time.time()
            speed = self.mot.read_speed(self.use_axis)
            step = self.mot.read_counter(self.use_axis, cnt_mode='counter')

            t1 = time.time()

            for i, ax in enumerate(self.use_axis):
                self.pub[ax+'_speed'].publish(speed[i])
                self.pub[ax+'_step'].publish(step[i])
                continue

            t2 = time.time()
            print('dt1: {.6f:0}, dt2: {.6f:1}'.format(t1-t0, t2-t1))

            if not self.func_queue.empty():
                f = self.func_queue.get()
                f['func'](f['data'], f['axis'])
            else:
                pass

            time.sleep(1e-5)
            continue

    #regist_function(callback)
    def regist_start(self, req, axis):
        self.func_queue.put({'func': self.start ,'data': req.data 'axis': axis})
        pass

    def regist_stop(self, req, axis):
        self.func_queue.put({'func': self.stop ,'data': req.data 'axis': axis})
        pass

    def regist_set_speed(self, req, axis):
        self.func_queue.put({'func': self.set_speed ,'data': req.data 'axis': axis})
        pass

    def regist_set_step(self, req, axis):
        self.func_queue.put({'func': self.set_step ,'data': req.data 'axis': axis})
        pass

    def regist_set_acc(self, req, axis):
        self.func_queue.put({'func': self.set_acc ,'data': req.data 'axis': axis})
        pass

    def regist_set_dec(self, req, axis):
        self.func_queue.put({'func': self.set_dec ,'data': req.data 'axis': axis})
        pass

    def regist_change_speed(self, req, axis):
        self.func_queue.put({'func': self.change_speed ,'data': req.data 'axis': axis})
        pass

    def regist_change_step(self, req, axis):
        self.func_queue.put({'func': self.change_step ,'data': req.data 'axis': axis})
        pass

    #function
    def start(self, req, axis):
        self.mot.start_motion(axis=axis, start_mode='acc', move_mode=self.params[axis]['mode'])
        pass

    def stop(self, req, axis):
        self.mot.stop_motion(axis=axis, stop_mode='dec_stop')
        pass

    def set_speed(self, req, axis):
        self.motion[axis]['speed'] = req.data
        self.mot.set_motion(axis=axis, mode=self.params[axis]['mode'], motion=self.motion)
        pass

    def set_step(self, req, axis):
        self.motion[axis]['step'] = req.data
        self.mot.set_motion(axis=axis, mode=self.params[axis]['mode'], motion=self.motion)
        pass

    def set_acc(self, req, axis):
        self.params[axis]['acc'] = req.data
        self.mot.set_motion(axis=axis, mode=self.params[axis]['mode'], motion=self.motion)
        pass

    def set_dec(self, req, axis):
        self.params[axis]['dec'] = req.data
        self.mot.set_motion(axis=axis, mode=self.params[axis]['mode'], motion=self.motion)
        pass

    def change_speed(self, req, axis):
        self.mot.change_speed(axis=axis, mode='accdec_change', speed=[req.data])
        #self.params[axis]['motion'][axis]['speed'] = abs(req.data)
        pass

    def change_step(self, req, axis):
        self.mot.change_step(axis=axis, step=[req.data])
        pass
