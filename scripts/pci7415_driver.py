#! /usr/bin/env python3

import sys
import time
import queue
import threading

import rospy

import pyinterface
print(pyinterface.__version__)
import std_msgs.msg

class pci7415_driver(object):

    def __init__(self, rsw_id, params):
        self.func_queue = queue.Queue()
        self.pub = {}
        self.use_axis = ''.join([ax for ax in params])

        self.params = params
        self.mode = [params[ax]['mode'] for ax in self.use_axis]
        self.motion = {ax: params[ax]['motion'] for ax in self.use_axis}
        self.is_moving = [0 for i in range(len(self.use_axis))]


        # initialize motion controller
        self.mot = pyinterface.open(7415, rsw_id)
        [self.mot.set_pulse_out(ax, 'method', params[ax]['pulse_conf']) for ax in self.use_axis]
        self.mot.set_motion(self.use_axis, self.mode, self.motion)

        #Subscriber&Publisher
        base = '/pyinterface/pci7415/rsw{rsw_id}'.format(**locals())
        rospy.Subscriber(base+'/output_do', std_msgs.msg.Int64MultiArray, self.regist_output_do)
        #self.pub_qsize = rospy.Publisher(base+'/fun_qsize', std_msgs.msg.Float64, queue_size=1)
        for ax in self.use_axis:
            b = '{base}/{ax}/'.format(**locals())
            rospy.Subscriber(b+'internal/start', std_msgs.msg.Float64MultiArray, self.regist_start, callback_args=ax)
            rospy.Subscriber(b+'internal/stop', std_msgs.msg.Int64, self.regist_stop, callback_args=ax)
            rospy.Subscriber(b+'internal/change_speed', std_msgs.msg.Float64, self.regist_change_speed, callback_args=ax)
            rospy.Subscriber(b+'internal/change_step', std_msgs.msg.Int64, self.regist_change_step, callback_args=ax)
            self.pub[ax+'_speed'] = rospy.Publisher(b+'speed', std_msgs.msg.Float64, queue_size=1)
            self.pub[ax+'_step'] = rospy.Publisher(b+'step', std_msgs.msg.Int64, queue_size=1)
            self.pub[ax+'_moving'] = rospy.Publisher(b+'moving', std_msgs.msg.Int64, queue_size=1)
            #self.pub_dt1 = rospy.Publisher(b+'internal/dt1', std_msgs.msg.Float64, queue_size=1)
            #self.pub_dt2 = rospy.Publisher(b+'internal/dt2', std_msgs.msg.Float64, queue_size=1)
            continue

        time.sleep(0.5)

        # loop start
        self.th = threading.Thread(target=self.moving_flag)
        self.th.setDaemon(True)
        self.th.start()
        self.th2 = threading.Thread(target=self.loop)
        self.th2.setDaemon(True)
        self.th2.start()

        return

    def loop(self):
        while not rospy.is_shutdown():
            #t0 = time.time()
            speed = self.mot.read_speed(self.use_axis)
            step = self.mot.read_counter(self.use_axis, cnt_mode='counter')

            #t1 = time.time()

            for i, ax in enumerate(self.use_axis):
                self.pub[ax+'_speed'].publish(speed[i])
                self.pub[ax+'_step'].publish(step[i])
                self.pub[ax+'_moving'].publish(self.is_moving[i])
                continue
            # 所要時間を測る
            #t2 = time.time()
            #self.pub_dt1.publish(t1-t0)
            #self.pub_dt2.publish(t2-t1)
            #self.pub_qsize.publish(self.func_queue.qsize())
            if not self.func_queue.empty():
                f = self.func_queue.get()
                f['func'](f['data'], f['axis'])
            else:
                pass

            time.sleep(1e-5)
            # 要検討
            continue

    #regist_function(callback)
    def regist_output_do(self, req):
        self.func_queue.put({'func': self.output_do, 'data': list(req.data), 'axis': 0})
        pass

    def regist_start(self, req, axis):
        self.func_queue.put({'func': self.start, 'data': req.data, 'axis': axis})
        pass

    def regist_stop(self, req, axis):
        self.func_queue.put({'func': self.stop, 'data': req.data, 'axis': axis})
        pass

    def regist_change_speed(self, req, axis):
        self.func_queue.put({'func': self.change_speed, 'data': req.data, 'axis': axis})
        pass

    def regist_change_step(self, req, axis):
        self.func_queue.put({'func': self.change_step, 'data': req.data, 'axis': axis})
        pass

    #function
    def output_do(self, data, axis):
        self.mot.output_do(data)
        pass

    def moving_flag(self):
        while not rospy.is_shutdown():
            _moving = self.mot.driver.get_main_status(self.use_axis)
            self.is_moving = [int(_moving[i][0]) for i in range(len(self.use_axis))]
            continue
        pass

    def start(self, data, axis):
        self.mot.stop_motion(axis=axis, stop_mode='immediate_stop')
        self.motion[axis]['speed'] = data[0]
        self.motion[axis]['step'] = int(data[1])
        axis_mode = [self.mode[self.use_axis.find(axis)]]
        while self.is_moving[self.use_axis.find(axis)] != 0:
            time.sleep(10e-5)
            continue
        self.mot.set_motion(axis=axis, mode=axis_mode, motion=self.motion)
        self.mot.start_motion(axis=axis, start_mode='const', move_mode=self.params[axis]['mode'])
        pass

    def stop(self, data, axis):
        self.mot.stop_motion(axis=axis, stop_mode='immediate_stop')
        pass

    def change_speed(self, data, axis):
        self.mot.change_speed(axis=axis, mode='accdec_change', speed=[data])
        #self.params[axis]['motion'][axis]['speed'] = abs(req.data)
        pass

    def change_step(self, data, axis):
        self.mot.change_step(axis=axis, step=[data])
        pass
