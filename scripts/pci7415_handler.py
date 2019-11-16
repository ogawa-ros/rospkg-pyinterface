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

        self.move_mode = {ax: p['mode'] for ax, p in params.items()}
        self.low_speed = {ax: p['motion']['low_speed'] for ax, p in params.items()}

        base = '/pci7415/rsw{rsw_id}'.format(**locals())
        # create publishers
        self.pub = {}
        for ax in self.use_axis:
            b = '{base}/{ax}/'.format(**locals())
            self.pub[ax+'_stop_motion'] = rospy.Publisher('/dev'+b+'internal/stop_motion', std_msgs.msg.Int64, queue_size=1)
            self.pub[ax+'_start_motion'] = rospy.Publisher('/dev'+b+'internal/start_motion', std_msgs.msg.Int64, queue_size=1)
            self.pub[ax+'_change_speed'] = rospy.Publisher('/dev'+b+'internal/change_speed', std_msgs.msg.Int64, queue_size=1)
            self.pub[ax+'_change_step'] = rospy.Publisher('/dev'+b+'internal/change_step', std_msgs.msg.Int64, queue_size=1)
            self.pub[ax+'_set_speed'] = rospy.Publisher('/dev'+b+'internal/set_speed', std_msgs.msg.Int64, queue_size=1)
            self.pub[ax+'_set_step'] = rospy.Publisher('/dev'+b+'internal/set_step', std_msgs.msg.Int64, queue_size=1)
            continue
        self.pub['ouput_do'] = rospy.Publisher('/dev/pci7415/internal/output_do', std_msgs.msg.Int64MultiArray, queue_size=1)

        # create subscrivers
        for ax in self.use_axis:
            b = '{base}/{ax}/'.format(**locals())
            #rospy.Subscriber('/dev'+b+'mode_cmd', std_msgs.msg.String, self.set_mode, callback_args=ax+'_mode')
            rospy.Subscriber('/dev'+b+'step_cmd', std_msgs.msg.Int64, self.set_speed, callback_args=ax)
            rospy.Subscriber('/dev'+b+'speed_cmd', std_msgs.msg.Int64, self.set_step, callback_args=ax)

            rospy.Subscriber('/dev'+b+'internal/speed', std_msgs.msg.Int64, self.get_speed, callback_args=ax)
            rospy.Subscriber('/dev'+b+'internal/step', std_msgs.msg.Int64, self.get_step, callback_args=ax)
            continue

        # create DIO pub/sub
            ### future developing ###

    def set_speed(self, speed, ax):
        if abs(speed) < self.low_speed[ax]:
            #pub stop
            self.pub[ax+'_stop_motion'].publish(1)
            while self.current_speed[ax] != 0:
                time.sleep(10e-5)
            return

        if self.move_mode[ax] == 'ptp':
            if self.current_speed[ax] != 0:
                #pub change_speed
                self.pub[ax+'_change_speed'].publish(abs(speed))
                pass

            else:
                self.pub[ax+'_change_speed'].publish(abs(speed))
                #pub start
                self.pub[ax+'_start_motion'].publish(1)
                pass

        elif self.move_mode[ax] == 'jog':
            if (self.last_direction_dict[axis] * param > 0) & (self.current_speed[ax] != 0):
                #pub change_speed
                self.pub[ax+'_change_speed'].publish(abs(speed))
                pass
            else:
                #pub stop
                self.pub[ax+'_stop_motion'].publish(1)
                while self.current_speed[ax] != 0:
                    time.sleep(10e-5)

                if param > 0:
                    self.pub[ax+'_set_step'].publish(+1)
                    self.last_direction_dict[axis] = +1
                else:
                    self.pub[ax+'_set_step'].publish(-1)
                    self.last_direction_dict[axis] = -1
                    pass
                self.pub[ax+'_set_speed'].publish(abs(speed))
                #pub start
                self.pub[ax+'_start_motion'].publish(1)
                pass
            pass
        return


    def set_step(self, step, ax):
        if self.move_mode[ax] == 'ptp':
            if self.current_speed[ax] != 0:
                #pub change_step
                self.pub[ax+'_change_step'].publish(step)
            else:
                self.pub[ax+'_set_step'].publish(step)
                self.pub[ax+'_start_motion'].publish(1)
                pass

        elif self.move_mode[ax] == 'jog':
            # IGNORE 'step' ros-topic if moving as JOG mode
            pass
        return


    def set_do(self, do):
        _do = std_msgs.msg.Int64MultiArray()
        _do.data = do

        pass


    def get_speed(self, speed, ax):
        self.current_speed[axis] = speed
        pass


    def get_step(self, step, ax):
        self.current_step[axis] = speed
        pass
