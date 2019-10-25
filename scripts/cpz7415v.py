#! /usr/bin/env python3

import pyinterface
import threading

import std_msgs.msg

name = 'cpz7415'

default_rsw_id = '0'
default_use_axes = 'xyzu'
default_mode = 'ptp'
default_clock = 299
default_acc_mode = 'acc_normal'
default_low_speed = 200
default_speed = 10000
default_acc = 1000
default_dec = 1000
default_step = 0


class cpz7415v_controller(object):
    self.rsw_id = ''
    self.use_axes = ''
    self.params = {}
    self.execute_params = []
    self.execute_params_lock = False
    self.last_mode = {}
    self.status = {}

    def __init__(self, rsw_id, params):
        self.rsw_id = rsw_id
        self.use_axes = ''.join([p['axis'] for p in params])

        self.params = {}
        for p in params:
            self.params[p['axis']] = p
            continue

        # initialize motion controller
        self.mot = pyinterface.open(7415, rsw_id)
        [self.mot.set_motion(p['axis'], p['mode'], p['motion']) for p in params]

        # create publishers
        base = '/cpz7415_rsw{rsw_id}'.format(**locals())
        self.pub = {}
        for ax in self.use_axes:
            b = '{base}_{ax}_'.format(**locals())
            self.pub[ax+'_step'] = rospy.Publisher(b+'_step', std_msgs.msg.Int64, queue_size=1)
            self.pub[ax+'_speed'] = rospy.Publisher(b+'_speed', std_msgs.msg.Int64, queue_size=1)
            continue

        # create subscrivers
        for ax in self.use_axes:
            b = '{base}_{ax}_'.format(**locals())
            rospy.Subscriber(b+'_mode_cmd', std_msgs.msg.Int64, self.regist, callback_args=ax+'_mode')
            rospy.Subscriber(b+'_step_cmd', std_msgs.msg.Int64, self.regist, callback_args=ax+'_step')
            rospy.Subscriber(b+'_speed_cmd', std_msgs.msg.Int64, self.regist, callback_args=ax+'_speed')
            continue

        # create DIO pub/sub
        self.pub['di'] = rospy.Publisher(base+'_di', std_msgs.msg.Int64, queue_size=1)
        rospy.Subscriber(base+'_do_cmd', std_msgs.msg.Int64, self.regist, callback_args='do')

        # start threads
        time.sleep(0.5)
        self._start_threads()
        pass

    def execute(self):
        '''
        self.execute_params の実態
            - 辞書のリスト
                - 'category' : x_mode, x_speed, x_step, y_mode, ..., do
                - 'params' : 受信したパラメータ

        この関数の機能
        - self.execute_params を取り出し、self.execute_params を空にする
        - 取り出したものを順番に処理 (for文)
            - type が軸設定の場合
                - _execute_axis を呼ぶ
                - mode が切り替わったら、何かしらの初期化する
                    - JOG -> PTP の時
                        - 一度止めて、mode切り替え
                    - PTP -> JOG の時
                        - counter値がカオス?
            - type が DO の場合
                - _execute_do を呼ぶ
        '''
        if len(self.execute_params) == 0:
            return

        self.execute_params_lock = True
        execute_params = self.execute_params.copy()
        self.execute_params = []
        self.execute_params_lock = False

        for _p in execute_params:
            if _p['category'] == 'do':
                self._execute_do(_p['param'])
            else:
                self._execute_axis(**_p)
                pass
            continue
        return

    def _execute_axis(self, category, param):
        '''
        - mode に応じて、受信した speed, step から、実際に使う motion params を生成し、実行
        - JOG の場合
            - 0 speed は適切に処理 (Stop, Start)
            - speed の正負に応じて、step +1 / -1 を指定(できない)
            - speed の絶対値をつかう
        - PTP の場合
            - 0 speed はSTOP
            - speed の絶対値を使う
        '''
        def stop_move(axis):
            self.mot.stop_motion(axis=axis, stop_mode='dec_stop')
            while is_moving(axis):
                time.sleep(1e-5)
                continue
            return

        def set_start(axis):
            self.mot.set_motion(axis=axis, mode=self.params[axis]['mode'], motion=self.params[axis]['motion'])
            self.mot.start_motion(axis=axis, start_mode='acc', move_mode=self.params[axis]['motion'])
            return

        def is_moving(axis):
            return (self.mot.read_speed(axis)[0] != 0)

        axis, type_ = category.split('_')

        if type_ == 'mode':
            '''
            の場合
            '''
            stop_move(axis)
            self.params[axis]['mode'] = param
            pass

        elif type_ == 'speed':
            if abs(param) < self.mot.motion_conf['jog'][axis]['low_speed']:
                param = 0
                pass

            if param == 0:
                stop_move(axis)
                pass

            if self.params[axis]['mode'] == 'ptp':
                # absolute value of 'speed' ros-topic will be used for PTP mode
                if is_moving(axis):
                    self.mot.change_speed(axis=axis, mode='accdec_change', speed=abs(param))
                else:
                    self.params[axis]['motion']['speed'] = abs(param)
                    set_start(axis)
                    pass

            elif self.params[axis]['mode'] == 'jog':
                last_direction = self.mot.motion_conf['jog'][axis]['step']
                if (last_direction * param > 0) & (is_moving(axis)):
                    self.mot.change_speed(axis=axis, mode='accdec_change', speed=abs(param))
                else:
                    stop_move(axis)
                    if param > 0:
                        self.paprams[axis]['motion']['step'] = +1
                    else:
                        self.paprams[axis]['motion']['step'] = -1
                        pass
                    self.params[axis]['motion']['speed'] = abs(param)
                    set_start(axis)
                    pass
                pass

        elif type_ == 'step':
            if self.params[axis]['mode'] == 'ptp':
                if is_moving(axis):
                    self.mot.change_step(axis=axis, step=param)
                else:
                    self.params[axis]['motion']['step'] = param
                    set_start(axis)
                    pass

            elif self.params[axis]['mode'] == 'jog':
                # IGNORE 'step' ros-topic if moving as JOG mode
                pass

        else:
            pass

    def _execute_do(self, do):
        '''
        - do にoutput する
        '''
        self.mot.output_do(do)
        pass

    def get_status(self):
        for ax in self.use_axes:
            step, speed = self._get_status_axis(ax)
            self.status[ax+'_step'] = step
            self.status[ax+'_speed'] = speed
            continue

        di = self.mot.input_di()
        self.status['di'] = di
        return

    def _get_status_axis(self, axis):
        '''
        - step, speed, di を取得
        - pub は別スレ (loop_publish_status)
        - なんかの変数使って渡す
        '''
        step = self.mot.read_counter(axis=axis, cnt_mode='counter')
        speed = self.mot.read_speed(axis)
        return step, speed

    def regist(self, req, type_):
        self.execute_params.append({
            'category': type_,
            'param': req.data,
        })
        return

    def loop_main_operation(self):
        '''
        loop の最大周波数は？？？
        '''
        while not rospy.is_shutdown():
            self.execute()
            self.get_status()
            time.sleep(0.001)
            continue
        return

    def loop_publish_status(self):
        while not rospy.is_shutdown():
            for name, value in self.status.items():
                if self.last_status[name] == value:
                    continue

                self.pub[name].publish(value)
                self.last_status[name] = value
                continue

            time.sleep(0.001) # あとでよう調整
            continue
        return

    def _start_threads(self):
        th = []
        th.append(threading.Thread(target=self.loop_main_operation))
        th.append(threading.Thread(target=self.loop_publish_status))
        [t.setDaemon(True) for t in th]
        [t.start() for t in th]
        return


if __name__ == '__main__':
    rospy.init_node(name)

    rsw_id = rospy.get_param('~rsw_id', default_rsw_id)
    use_axes = rospy.get_param('~use_axes', default_use_axes) # ex. 'xyzu', 'xy', or 'yu'

    params = []
    for ax in use_axes:
        p = {}
        p['axis'] = ax
        p['mode'] = rospy.get_param('~{ax}_mode'.format(**locals()), default_mode)

        mp = {}
        mp['clock'] = rospy.get_param('~{ax}_clock'.format(**locals()), default_clock)
        mp['acc_mode'] = rospy.get_param('~{ax}_acc_mode'.format(**locals()), default_acc_mode)
        mp['low_speed'] = rospy.get_param('~{ax}_low_speed'.format(**locals()), default_low_speed)
        mp['speed'] = rospy.get_param('~{ax}_speed'.format(**locals()), default_speed)
        mp['acc'] = rospy.get_param('~{ax}_acc'.format(**locals()), default_acc)
        mp['dec'] = rospy.get_param('~{ax}_dec'.format(**locals()), default_dec)
        mp['step'] = rospy.get_param('~{ax}_step'.format(**locals()), default_step)
        p['motion'] = mp
        params.append(p)
        continue

    ctrl = cpz7415v_controller(rsw_id, params)
    rospy.spin()



# ------------------------------------------------
# ------------------------------------------------
# ------------------------------------------------
# ------------------------------------------------
# ------------------------------------------------
# ------------------------------------------------




import sys
import time
import numpy
import pyinterface
import threading

import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Int64
from std_msgs.msg import Bool

class cpz7415v_controller(object):

    motion = {
        'x': {
            'clock':  0, 'acc_mode': '', 'low_speed': 0,
            'speed': 0, 'acc': 0, 'dec': 0, 'step': 0
        },
        'y': {
            'clock':  0, 'acc_mode': '', 'low_speed': 0,
            'speed': 0, 'acc': 0, 'dec': 0, 'step': 0
        },
        'z': {
            'clock':  0, 'acc_mode': '', 'low_speed': 0,
            'speed': 0, 'acc': 0, 'dec': 0, 'step': 0
        },
        'u': {
            'clock':  0, 'acc_mode': '', 'low_speed': 0,
            'speed': 0, 'acc': 0, 'dec': 0, 'step': 0
        }
    }

    move_mode = {'x': '', 'y': '', 'z': '', 'u': ''}

    last_position = {'x': 0, 'y': 0, 'z': 0, 'u': 0}

    last_speed = {'x': 0, 'y': 0, 'z': 0, 'u': 0}

    do_status = 0

    step_flag = {'x': False, 'y': False, 'z': False, 'u': False}
    speed_flag = {'x': False, 'y': False, 'z': False, 'u': False}
    do_flag = False


    def __init__(self):
        ###=== Define member-variables ===###
        self.rsw_id = rospy.get_param('~rsw_id')
        self.node_name = rospy.get_param('~node_name')
        self.move_mode['x'] = rospy.get_param('~mode_x')
        self.move_mode['y'] = rospy.get_param('~mode_y')
        self.move_mode['z'] = rospy.get_param('~mode_z')
        self.move_mode['u'] = rospy.get_param('~mode_u')

        ###=== Create instance ===###
        try: self.mot = pyinterface.open(7415, self.rsw_id)
        except OSError as e:
            rospy.logerr("{e.strerror}. node={node_name}, rsw={rsw_id}".
                         format(self.node_name, self.rsw_id))
            sys.exit()

        ###=== Setting the board ===###
        self.mot.initialize()
        self.motion['x']['clock'] = rospy.get_param('~clock_x')
        self.motion['x']['acc_mode'] = rospy.get_param('~acc_mode_x')
        self.motion['x']['low_speed'] = rospy.get_param('~low_speed_x')
        self.motion['x']['speed'] = rospy.get_param('~speed_x')
        self.motion['x']['acc'] = rospy.get_param('~acc_x')
        self.motion['x']['dec'] = rospy.get_param('~dec_x')
        self.motion['x']['step'] = rospy.get_param('~step_x')
        self.motion['y']['clock'] = rospy.get_param('~clock_y')
        self.motion['y']['acc_mode'] = rospy.get_param('~acc_mode_y')
        self.motion['y']['low_speed'] = rospy.get_param('~low_speed_y')
        self.motion['y']['speed'] = rospy.get_param('~speed_y')
        self.motion['y']['acc'] = rospy.get_param('~acc_y')
        self.motion['y']['dec'] = rospy.get_param('~dec_y')
        self.motion['y']['step'] = rospy.get_param('~step_y')
        self.motion['z']['clock'] = rospy.get_param('~clock_z')
        self.motion['z']['acc_mode'] = rospy.get_param('~acc_mode_z')
        self.motion['z']['low_speed'] = rospy.get_param('~low_speed_z')
        self.motion['z']['speed'] = rospy.get_param('~speed_z')
        self.motion['z']['acc'] = rospy.get_param('~acc_z')
        self.motion['z']['dec'] = rospy.get_param('~dec_z')
        self.motion['z']['step'] = rospy.get_param('~step_z')
        self.motion['u']['clock'] = rospy.get_param('~clock_u')
        self.motion['u']['acc_mode'] = rospy.get_param('~acc_mode_u')
        self.motion['u']['low_speed'] = rospy.get_param('~low_speed_u')
        self.motion['u']['speed'] = rospy.get_param('~speed_u')
        self.motion['u']['acc'] = rospy.get_param('~acc_u')
        self.motion['u']['dec'] = rospy.get_param('~dec_u')
        self.motion['u']['step'] = rospy.get_param('~step_u')
        self.mot.set_motion(axis='x', mode=self.move_mode['x'], motion=self.motion)
        self.mot.set_motion(axis='y', mode=self.move_mode['y'], motion=self.motion)
        self.mot.set_motion(axis='z', mode=self.move_mode['z'], motion=self.motion)
        self.mot.set_motion(axis='u', mode=self.move_mode['u'], motion=self.motion)

        ###=== Define topic ===###
        topic_step_x_cmd = '/{0}_rsw{1}_x_step_cmd'.format(self.node_name, self.rsw_id)
        topic_step_x = '/{0}_rsw{1}_x_step'.format(self.node_name, self.rsw_id)
        topic_speed_x_cmd = '/{0}_rsw{1}_x_speed_cmd'.format(self.node_name, self.rsw_id)
        topic_speed_x = '/{0}_rsw{1}_x_speed'.format(self.node_name, self.rsw_id)
        topic_step_y_cmd = '/{0}_rsw{1}_y_step_cmd'.format(self.node_name, self.rsw_id)
        topic_step_y = '/{0}_rsw{1}_y_step'.format(self.node_name, self.rsw_id)
        topic_speed_y_cmd = '/{0}_rsw{1}_y_speed_cmd'.format(self.node_name, self.rsw_id)
        topic_speed_y = '/{0}_rsw{1}_y_speed'.format(self.node_name, self.rsw_id)
        topic_step_z_cmd = '/{0}_rsw{1}_z_step_cmd'.format(self.node_name, self.rsw_id)
        topic_step_z = '/{0}_rsw{1}_z_step'.format(self.node_name, self.rsw_id)
        topic_speed_z_cmd = '/{0}_rsw{1}_z_speed_cmd'.format(self.node_name, self.rsw_id)
        topic_speed_z = '/{0}_rsw{1}_z_speed'.format(self.node_name, self.rsw_id)
        topic_step_u_cmd = '/{0}_rsw{1}_u_step_cmd'.format(self.node_name, self.rsw_id)
        topic_step_u = '/{0}_rsw{1}_u_step'.format(self.node_name, self.rsw_id)
        topic_speed_u_cmd = '/{0}_rsw{1}_u_speed_cmd'.format(self.node_name, self.rsw_id)
        topic_speed_u = '/{0}_rsw{1}_u_speed'.format(self.node_name, self.rsw_id)
        topic_output_do_cmd = '/{0}_rsw{1}_do_cmd'.format(self.node_name, self.rsw_id)

        ###=== Define Publisher ===###
        self.pub_step_x = rospy.Publisher(topic_step_x, Int64, queue_size=1)
        self.pub_speed_x = rospy.Publisher(topic_speed_x, Int64, queue_size=1)
        self.pub_step_y = rospy.Publisher(topic_step_y, Int64, queue_size=1)
        self.pub_speed_y = rospy.Publisher(topic_speed_y, Int64, queue_size=1)
        self.pub_step_z = rospy.Publisher(topic_step_z, Int64, queue_size=1)
        self.pub_speed_z = rospy.Publisher(topic_speed_z, Int64, queue_size=1)
        self.pub_step_u = rospy.Publisher(topic_step_u, Int64, queue_size=1)
        self.pub_speed_u = rospy.Publisher(topic_speed_u, Int64, queue_size=1)

        ###=== Define Subscriber ===###
        self.sub_step_x_cmd = rospy.Subscriber(topic_step_x_cmd, Int64, self.set_step, callback_args='x')
        self.sub_speed_x_cmd = rospy.Subscriber(topic_speed_x_cmd, Int64, self.set_speed, callback_args='x')
        self.sub_step_y_cmd = rospy.Subscriber(topic_step_y_cmd, Int64, self.set_step, callback_args='y')
        self.sub_speed_y_cmd = rospy.Subscriber(topic_speed_y_cmd, Int64, self.set_speed, callback_args='y')
        self.sub_step_z_cmd = rospy.Subscriber(topic_step_z_cmd, Int64, self.set_step, callback_args='z')
        self.sub_speed_z_cmd = rospy.Subscriber(topic_speed_z_cmd, Int64, self.set_speed, callback_args='z')
        self.sub_step_u_cmd = rospy.Subscriber(topic_step_u_cmd, Int64, self.set_step, callback_args='u')
        self.sub_speed_u_cmd = rospy.Subscriber(topic_speed_u_cmd, Int64, self.set_speed, callback_args='u')
        self.sub_output_do_cmd = rospy.Subscriber(topic_output_do_cmd, Int64, self.output_do)

        pass


    def set_step(self, q, axis):
        self.motion[axis]['step'] = q.data
        self.step_flag[axis] = True
        return

    def _set_step(self):
        axis = ''
        step = []
        for i in self.move_mode:
            if self.step_flag[i]:
                axis += i
                step.append(self.motion[i]['step'])
            else: pass

        if axis != '':
            self.mot.set_motion(axis=axis, mode='ptp', motion=self.motion)
            onoff = self.mot.driver.check_move_onoff(axis)

            for i, ax in enumerate(axis):
                if onoff[i]:
                    self.mot.change_step(ax, [self.motion[ax]['step']])
                else:
                    self.mot.start_motion(axis=ax, start_mode='acc', move_mode='ptp')
                self.step_flag[ax] = False
        else: pass
        time.sleep(0.0001)
        return

    def _get_step(self):
        step = self.mot.read_counter(axis='xyzu', cnt_mode='counter')

        if self.last_position['x'] != step[0]:
            self.pub_step_x.publish(step[0])
            self.last_position['x'] = step[0]
        if self.last_position['y'] != step[1]:
            self.pub_step_y.publish(step[1])
            self.last_position['y'] = step[1]
        if self.last_position['z'] != step[2]:
            self.pub_step_z.publish(step[2])
            self.last_position['z'] = step[2]
        if self.last_position['u'] != step[3]:
            self.pub_step_u.publish(step[3])
            self.last_position['u'] = step[3]

        time.sleep(0.0001)
        return


    def set_speed(self, q, axis):
        self.motion[axis]['speed'] = q.data
        self.speed_flag[axis] = True
        return


    def _set_speed(self):
        axis = ''
        speed = []
        for i in self.move_mode:
            if self.speed_flag[i]:
                axis += i
                speed.append(self.motion[i]['speed'])
            else: pass

        if axis != '':
            self.mot.driver.set_motion(axis=axis, mode='jog', motion=self.motion)
            onoff = self.mot.driver.check_move_onoff(axis)

            for i, ax in enumerate(axis):
                if onoff[i]:
                    self.mot.change_speed(axis=ax, mode='accdec_change', speed=[speed[i]])
                else:
                    self.mot.start_motion(axis=ax, start_mode='acc', move_mode='jog')
                self.speed_flag[ax] = False
        else: pass
        time.sleep(0.0001)
        return


    def _get_speed(self):
        speed = self.mot.read_speed(axis='xyzu')

        if self.last_speed['x'] != speed[0]:
            self.pub_speed_x.publish(speed[0])
            self.last_speed['x'] = speed[0]
        if self.last_speed['y'] != speed[1]:
            self.pub_speed_y.publish(speed[1])
            self.last_speed['y'] = speed[1]
        if self.last_speed['z'] != speed[2]:
            self.pub_speed_z.publish(speed[2])
            self.last_speed['z'] = speed[2]
        if self.last_speed['u'] != speed[3]:
            self.pub_speed_u.publish(speed[3])
            self.last_speed['u'] = speed[3]

        time.sleep(0.0001)
        return


    def output_do(self, q):
        self.do_status = q.data
        self.do_flag = True
        return


    def _output_do(self):
        self.mot.output_do(self.do_status)
        self.do_flag = False
        time.sleep(0.001)
        return


    def _main_thread(self):
        while not rospy.is_shutdown():
            self._set_step()
            self._set_speed()
            self._get_step()
            self._get_speed()
            if self.do_flag:
                self._output_do()
            continue
        return


    def start_thread_ROS(self):
        th = threading.Thread(target=self._main_thread)
        th.setDaemon(True)
        th.start()
        return


if __name__ == '__main__':
    rospy.init_node('cpz7415v')
    ctrl = cpz7415v_controller()
    ctrl.start_thread_ROS()
    rospy.spin()
