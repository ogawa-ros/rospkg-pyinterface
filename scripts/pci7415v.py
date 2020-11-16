#! /usr/bin/env python3

node_name = 'pci7415_driver'

import sys
import time
import queue
import rospy
import threading
import pyinterface
print(pyinterface.__version__)
import std_msgs.msg

class pci7415(object):
    def __init__(self,rsw_id, params):
        self.params = params
        self.func_queue = queue.Queue()

        self.use_axis = ''.join([ax for ax in self.params])
        self.mode = [self.params[ax]['mode'] for ax in self.use_axis]
        self.motion = {ax: self.params[ax]['motion'] for ax in self.use_axis}

        self.current_speed = {ax: 0 for ax in self.use_axis}
        self.current_step = {ax: 0 for ax in self.use_axis}
        self.current_moving = {ax: 0 for ax in self.use_axis}
        self.last_direction = {ax: 0 for ax in self.use_axis}
        self.do_status = [0, 0, 0, 0]
        self.move_mode = {ax: p['mode'] for ax, p in self.params.items()}
        self.default_speed = {ax: p['motion']['speed'] for ax, p in self.params.items()}
        self.low_speed = {ax: p['motion']['low_speed'] for ax, p in self.params.items()}

        self.mot = pyinterface.open(7415,rsw_id)
        #self.mot.initialize()
        for ax in self.use_axis:
            self.mot.set_pulse_out(ax, 'method', self.params[ax]['pulse_conf'])
        self.mot.set_motion(self.use_axis, self.mode, self.motion)


        self.pub = {}
        base = '/pyinterface/pci7415/rsw{self.rsw_id}'.format(**locals())
        for ax in self.use_axis:
            b = '{base}/{ax}/'.format(**locals())
            #create publish
            self.pub[ax+'_speed'] = rospy.Publisher(b+'speed', std_msgs.msg.Float64, queue_size=1)
            self.pub[ax+'_step'] = rospy.Publisher(b+'step', std_msgs.msg.Int64, queue_size=1)

            rospy.Subscriber(b+'step_cmd', std_msgs.msg.Int64, self.set_step, callback_args=ax)
            rospy.Subscriber(b+'speed_cmd', std_msgs.msg.Float64, self.set_speed, callback_args=ax)

        for do_num in range(1,5):
            rospy.Subscriber('{}/output_do{}_cmd'.format(base, do_num), std_msgs.msg.Int64, self.set_do, callback_args=do_num)

        self.th = threading.Thread(target=self.loop)
        self.th.setDaemon(True)
        self.th.start()
        return

    def loop(self):
        while not rospy.is_shutdown():
            #t0 = time.time()
            speed = self.mot.read_speed(self.use_axis)
            step = self.mot.read_counter(self.use_axis, cnt_mode='counter')
            moving = [int(_[0]) for _ in self.mot.driver.get_main_status(self.use_axis)]

            for i, ax in enumerate(self.use_axis):
                self.pub[ax+'_speed'].publish(speed[i])
                self.pub[ax+'_step'].publish(step[i])
                continue

                #t1 = time.time()
             # speed, step はそのまま辞書に入れる　　for文はいらない

            self.current_speed = {ax: _ for ax, _ in zip(self.use_axis, speed)}
            self.current_step = {ax: _ for ax, _ in zip(self.use_axis, step)}
            self.current_moving = {ax: _ for ax, _ in zip(self.use_axis, moving)}

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
        #time.sleep(1e-5)
            # 要検討

    def set_speed(self, speed, ax):
        if abs(speed.data) < self.low_speed[ax]:
            #pub stop
            self.func_queue.put({'func': self.stop, 'data': 1, 'axis': ax}) # ->req data は１のこと？？？？？???????

            self.last_direction[ax] = 0
            return

        if self.move_mode[ax] == 'jog':
            if (self.last_direction[ax] * speed.data > 0) & (self.current_moving[ax] != 0):
                #pub change_speed
                self.func_queue.put({'func': self.change_speed, 'data': abs(speed.data), 'axis': ax}) #req dataはabs(speed.data)???????
                pass

            else:

                if speed.data > 0:
                    step = +1
                    pass

                else:
                    step = -1
                    pass

                speed_step = [abs(speed.data), step]
                self.last_direction[ax] = step

                #pub start

                self.func_queue.put({'func': self.start, 'data': speed_step, 'axis': ax})
                time.sleep(0.01)
                pass
            pass

        else:
            pass


        return

    def set_step(self, step, ax):
        if self.move_mode[ax] == 'ptp':
            if self.current_moving[ax] != 0:
                #pub change_step
                self.func_queue.put({'func': self.change_step, 'data': step.data, 'axis': ax})

            else:
                speed_step = [self.params[ax]['motion']['speed'],step.data]
                self.func_queue.put({'func': self.start, 'data': speed_step, 'axis': ax})
                pass
        else: pass
        return

    def set_do(self, do, do_num):
        self.do_status[do_num-1] = do.data
        self.func_queue.put({'func': self.output_do, 'data': self.do_status, 'axis': 0})
        return

    #function
    def output_do(self, data, axis):
        self.mot.output_do(data)
        pass

    def start(self, data, axis):
        self.mot.stop_motion(axis=axis, stop_mode='immediate_stop')
        self.motion[axis]['speed'] = data[0]
        self.motion[axis]['step'] = int(data[1])

        axis_mode = [self.mode[self.use_axis.find(axis)]]
        while int(self.mot.driver.get_main_status(axis)[0][0]) != 0:
            time.sleep(1e-5)
            continue
        self.mot.set_motion(axis=axis, mode=axis_mode, motion=self.motion)
        self.mot.start_motion(axis=axis, start_mode='const', move_mode=self.params[axis]['mode'])
        pass

    def stop(self, data, axis):
        self.mot.stop_motion(axis=axis, stop_mode='immediate_stop')
        while int(self.mot.driver.get_main_status(axis)[0][0]) != 0:
            time.sleep(1e-5)
        pass

    def change_speed(self, data, axis):
        self.mot.change_speed(axis=axis, mode='accdec_change', speed=[data])
        #self.params[axis]['motion'][axis]['speed'] = abs(req.data)
        pass

    def change_step(self, data, axis):
        self.mot.change_step(axis=axis, step=[data])
        pass

# default_name = 'pci7415'
# default_rsw_id = '0'
# default_do_conf = "[0, 0, 0, 0]"
# default_use_axis = 'xyzu'
# default_pulse_conf = "{'PULSE': '0', 'OUT': '0', 'DIR': '0', 'WAIT': '0', 'DUTY': '0'}"
# default_mode = 'ptp'
# default_clock = 299
# default_acc_mode = 'acc_normal'
# default_low_speed = 200
# default_speed = 10000
# default_acc = 1000
# default_dec = 1000
# default_step = 0

if __name__ == '__main__':
    name = rospy.get_param('~node_name'.format(**locals()), default_name)
    rospy.init_node(name)

    rsw_id = rospy.get_param('~rsw_id', default_rsw_id)
    use_axis = rospy.get_param('~use_axis', default_use_axis) # ex. 'xyzu', 'xy', or 'yu'

    conf_list={'x':{'PULSE': 1, 'OUT': 0, 'DIR': 0, 'WAIT': 0, 'DUTY': 0},
               'y':{'PULSE': 1, 'OUT': 0, 'DIR': 0, 'WAIT': 0, 'DUTY': 0},
               'z':{'PULSE': 1, 'OUT': 0, 'DIR': 0, 'WAIT': 0, 'DUTY': 0},
               'u':{'PULSE': 1, 'OUT': 1, 'DIR': 1, 'WAIT': 0, 'DUTY': 0}
               }

    params = {}
    for ax in use_axis:
        params[ax] = {}
        params[ax]['mode'] = rospy.get_param('~{ax}_mode'.format(**locals()), default_mode)
        params[ax]['pulse_conf'] = [eval(rospy.get_param('~{ax}_pulse_conf'.format(**locals()), default_pulse_conf))]

        mp = {}
        mp['clock'] = rospy.get_param('~{ax}_clock'.format(**locals()), default_clock)
        mp['acc_mode'] = rospy.get_param('~{ax}_acc_mode'.format(**locals()), default_acc_mode)
        mp['low_speed'] = rospy.get_param('~{ax}_low_speed'.format(**locals()), default_low_speed)
        mp['speed'] = rospy.get_param('~{ax}_speed'.format(**locals()), default_speed)
        mp['acc'] = rospy.get_param('~{ax}_acc'.format(**locals()), default_acc)
        mp['dec'] = rospy.get_param('~{ax}_dec'.format(**locals()), default_dec)
        mp['step'] = rospy.get_param('~{ax}_step'.format(**locals()), default_step)

        params[ax]['motion'] = mp
        continue

    handler = pci7415(rsw_id, params)

    rospu.spin()
