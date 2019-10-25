#! /usr/bin/env python3

import pyinterface
import threading

import std_msgs.msg

name = 'cpz7415'

default_rsw_id = '1'
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
            self.params[p['axis']]['is_moving'] = False
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
                        - ...
                    - PTP -> JOG の時
                        - ...
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
            - speed の正負に応じて、step +1 / -1 を指定
            - speed の絶対値をつかう
        - PTP の場合
            - 0 speed はSTOP
            - speed の絶対値を使う
        '''
        def stop_move():
            self.mot.stop_motion(axis=axis, stop_mode='dec_stop')
            while self.mot.read_speed(axis)[0] != 0:
                continue
            self.params[axis]['is_moving'] = False
            self.last_speed = 0

        def set_start():
            self.mot.set_motion(axis=axis, mode=self.params[axis]['mode'], motion=self.params[axis]['motion'])
            self.mot.start_motion(axis=axis, start_mode='acc', move_mode=self.params[axis]['motion'])
            self.params[axis]['is_moving'] = True

        axis, type_ = category.split('_')
        if type_ == 'mode':
            stop_move()
            self.params[axis]['mode'] = param
        '''
        PTPの場合
        '''
        if self.param[axis]['mode'] = 'ptp':
            if self.mot.read_speed(axis)[0] == 0:
                self.params[axis]['motion']['is_moving'] = False
            else:
                self.params[axis]['motion']['is_moving'] = True

            if type_ == 'speed':
                if param == 0:
                    stop_move()

                else:
                    if self.params[axis]['is_moving']:
                        self.mot.change_speed(axis=axis, mode='accdec_change', speed=abs(param))
                    else:
                        self.params[axis]['motion']['speed'] = abs(param)
                        set_start()

            if type_ == 'step':
                if self.params[axis]['is_moving']:
                    stop_move()
                    #self.mot.change_step(axis=axis, step=param)
                #else:
                self.params[axis]['motion']['step'] = param
                set_start()
        '''
        JOGの場合
        '''
        if self.param[axis]['mode'] = 'jog':
            if self.mot.read_speed(axis)[0] == 0:
                self.params[axis]['motion']['is_moving'] = False
            else:
                self.params[axis]['motion']['is_moving'] = True

            if type_ == 'speed':
                if prama == 0:
                    stop_move()

                else:
                    if not self.params[axis]['is_moving']:
                        if param > 0:
                            self.param[axis]['mode'] = '+jog'
                        else:
                            self.param[axis]['mode'] = '-jog'
                        self.params[axis]['motion']['speed'] = abs(param)
                        ste_start()
                        self.last_speed = param

                    else:
                        if param*self.last_speed > 0:
                            self.mot.change_speed(axis=axis, mode='accdec_change', speed=abs(param))
                            self.last_speed = param
                        elif param*self.last_speed < 0:
                            stop_move()
                            if param > 0:
                                self.param[axis]['mode'] = '+jog'
                            else:
                                self.param[axis]['mode'] = '-jog'
                            self.params[axis]['motion']['speed'] = abs(param)
                            set_start()
                            self.last_speed = param

            if type_ == 'step':
                pass

    def _execute_do(self, do):
        '''
        - do にoutput する
        '''
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
