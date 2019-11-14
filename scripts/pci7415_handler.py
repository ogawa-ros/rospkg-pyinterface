#! /usr/bin/env python3

import time

import pyinterface
import threading
import rospy

import std_msgs.msg


class pci7415_handler(object):
    rsw_id = ''
    use_axis = ''
    params = {}
    execute_params = []
    execute_params_lock = False
    last_mode = {}
    status = {}

    def __init__(self, rsw_id, params):

        # create publishers
        base = '/cpz7415/rsw{rsw_id}'.format(**locals())
        self.pub = {}
        for ax in self.use_axis:
            b = '{base}/{ax}/'.format(**locals())
            self.pub[ax+'_step'] = rospy.Publisher('/dev'+b+'step', std_msgs.msg.Int64, queue_size=1)
            self.pub[ax+'_speed'] = rospy.Publisher('/dev'+b+'speed', std_msgs.msg.Int64, queue_size=1)
            continue

        # create subscrivers
        for ax in self.use_axis:
            b = '{base}/{ax}/'.format(**locals())
            rospy.Subscriber('/dev'+b+'mode_cmd', std_msgs.msg.String, self.regist, callback_args=ax+'_mode')
            rospy.Subscriber('/dev'+b+'step_cmd', std_msgs.msg.Int64, self.regist, callback_args=ax+'_step')
            rospy.Subscriber('/dev'+b+'speed_cmd', std_msgs.msg.Int64, self.regist, callback_args=ax+'_speed')
            continue

        # create DIO pub/sub
        #self.pub['di'] = rospy.Publisher(base+'_di', std_msgs.msg.Int64, queue_size=1)
        rospy.Subscriber('/dev'+base+'/do_cmd', std_msgs.msg.Int64, self.regist, callback_args='do')

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
            #print(self.params[axis]['motion'], self.params[axis]['mode'])
            self.mot.set_motion(axis=axis, mode=self.params[axis]['mode'], motion=self.params[axis]['motion'])
            self.mot.start_motion(axis=axis, start_mode='acc', move_mode=self.params[axis]['mode'])
            return

        def is_moving(axis):
            return (self.mot.read_speed(axis)[0] != 0)

        axis, type_ = category.split('_')

        if type_ == 'mode':
            stop_move(axis)
            self.params[axis]['mode'] = param
            pass

        elif type_ == 'speed':
            if abs(param) < self.params[axis]['motion'][axis]['low_speed']:
                param = 0
                pass

            if param == 0:
                stop_move(axis)
                return

            if self.params[axis]['mode'] == 'ptp':
                # absolute value of 'speed' ros-topic will be used for PTP mode
                if is_moving(axis):
                    self.mot.change_speed(axis=axis, mode='accdec_change', speed=[abs(param)])
                else:
                    self.params[axis]['motion'][axis]['speed'] = abs(param)
                    set_start(axis)
                    pass

            elif self.params[axis]['mode'] == 'jog':
                print('last_direction: {}'.format(self.last_direction_dict[axis]) + ', ' + 'param: {}'.format(param) + ', ' + 'is_moving: {}'.format(is_moving(axis)) + ', ' + 'axis: {}'.format(axis))

                if (self.last_direction_dict[axis] * param > 0) & (is_moving(axis)):
                    self.mot.change_speed(axis=axis, mode='accdec_change', speed=[abs(param)])
                else:
                    stop_move(axis)
                    if param > 0:
                        self.params[axis]['motion'][axis]['step'] = +1
                    else:
                        self.params[axis]['motion'][axis]['step'] = -1
                        pass
                    self.params[axis]['motion'][axis]['speed'] = abs(param)
                    set_start(axis)
                    pass
                self.last_direction_dict[axis] = self.params[axis]['motion'][axis]['step']
                pass

        elif type_ == 'step':
            if self.params[axis]['mode'] == 'ptp':
                if is_moving(axis):
                    self.mot.change_step(axis=axis, step=[param])
                else:
                    self.params[axis]['motion'][axis]['step'] = param
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
        for ax in self.use_axis:
            step, speed = self._get_status_axis(ax)
            self.status[ax+'_step'] = step
            self.status[ax+'_speed'] = speed
            continue

        #di = self.mot.input_di()
        #self.status['di'] = di
        return

    def _get_status_axis(self, axis):
        '''
        - step, speed, di を取得
        - pub は別スレ (loop_publish_status)
        - なんかの変数使って渡す
        '''
        step = self.mot.read_counter(axis=axis, cnt_mode='counter')[0]
        speed = self.mot.read_speed(axis)[0]
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
        self.last_status = {}
        while not rospy.is_shutdown():
            for name, value in self.status.items():
                if not name in self.last_status.keys():
                    self.pub[name].publish(value)
                    self.last_status[name] = value

                if self.last_status[name] == value:
                    continue

                else:
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
