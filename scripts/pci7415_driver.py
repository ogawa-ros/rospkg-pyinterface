#! /usr/bin/env python3

import time
import threading

import rospy
import pyinterface

class pci7415_driver(object):

    def __init__(self, rsw_id, params):
        self.func_dict_li = []
        self.pub = {}
        self.use_axis = ''.join([p['axis'] for p in params])

        self.params = {}
        for p in params:
            self.params[p['axis']] = p
            continue

        # initialize motion controller
        self.mot = pyinterface.open(7415, rsw_id)
        self.mot.initialize()
        self.mot.output_do(params[0]['do_conf'])
        [self.mot.set_pulse_out(p['axis'], p['method'], p['pulse_conf']) for p in params]
        [self.mot.set_motion(p['axis'], p['mode'], p['motion']) for p in params]
        self.last_direction_dict = {p['axis']: 0 for p in params}

        #Subscriber&Publisher
        base = '/pci7415/rsw{rsw_id}'.format(**locals())
        for ax in self.use_axis:
            b = '{base}/{ax}/'.format(**locals())
            rospy.Subscriber('/dev'+b+'internal/set_speed', std.msgs.msg.Int64, self.regist_set_speed, callback_args=ax)
            rospy.Subscriber('/dev'+b+'internal/set_step', std.msgs.msg.Int64, self.regist_set_step, callback_args=ax)
            rospy.Subscriber('/dev'+b+'internal/start_motion', std.msgs.msg.Int64, self.regist_start, callback_args=ax)
            rospy.Subscriber('/dev'+b+'internal/stop_motion', std.msgs.msg.Int64, self.regist_stop, callback_args=ax)
            rospy.Subscriber('/dev'+b+'internal/change_speed', std.msgs.msg.Int64, self.regist_change_speed, callback_args=ax)
            self.pub[ax+'_speed'] = rospy.Publisher('/dev'+b+'internal/speed', std.msgs.msg.Float64, queue_size=1)
            self.pub[ax+'_step'] = rospy.Publisher('/dev'+b+'internal/step', std.msgs.msg.Float64, queue_size=1)

        time.sleep(0.5)

        # loop start
        self.th = threading.Thread(target= self.loop)
        self.th.setDaemon(True)
        self.th.start()
        return

    def loop(self):
        while not rospy.is_shutdown():
            speed =self.mot.read_speed(self.use_axis)
            step = self.mot.read_counter(self.use_axis, cnt_mode = 'counter')
            for i, ax in enumerate(self.use_axis):
                self.pub[ax+'_speed'].publish(speed[i])
                self.pub[ax+'_step'].publish(step[i])
            if self.func_dict_li != {}:
                func = self.func_dict_li.pop(0)
                func['func'](func['data'], func['axis'])
            else:
                pass
            time.sleep(1e-5)
            continue

    #regist_function(callback)
    def regist_set_speed(self, req, axis):
        self.func_dict_li.append({'func': self.set_speed ,'data': req.data 'axis': axis})
        pass

    def regist_set_step(self, req, axis):
        self.func_dict_li.append({'func': self.set_step ,'data': req.data 'axis': axis})
        pass

    def regist_start(self, req, axis):
        self.func_dict_li.append({'func': self.start_motion ,'data': req.data 'axis': axis})
        pass

    def regist_stop(self, req, axis):
        self.func_dict_li.append({'func': self.stop_motion ,'data': req.data 'axis': axis})
        pass

    def regist_change_speed(self, req, axis):
        self.func_dict_li.append({'func': self.change_speed ,'data': req.data 'axis': axis})
        pass

    def regist_change_step(self, req, axis):
        self.func_dict_li.append({'func': self.change_step ,'data': req.data 'axis': axis})
        pass

    #func_dict = {'func': ,'data': 'axis': }

    #function
    def set_speed(self, req, axis):
        self.params[axis]['motion'][axis]['speed'] = abs(req.data)
        pass

    def set_step(self, req, axis):
        self.params[axis]['motion'][axis]['step'] = req.data
        pass

    def start_motion(self, req, axis):
        self.mot.set_motion(axis=axis, mode=self.params[axis]['mode'], motion=self.params[axis]['motion'])
        self.mot.start_motion(axis=axis, start_mode='acc', move_mode=self.params[axis]['mode'])
        pass

    def stop_motion(self, req, axis):
        self.mot.stop_motion(axis=axis, stop_mode='dec_stop')
        pass

    def change_speed(self, req, axis):
        self.mot.change_speed(axis=axis, mode='accdec_change', speed=[abs(req.data)])
        pass

    def change_step(self, req, axis):
        self.mot.change_step(axis=axis, step=[req.data])
        pass
