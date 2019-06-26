#! /usr/bin/env python3


import sys
import time
import threading
import pyinterface

import rospy
from std_msgs.msg import Int32
from std_msgs.msg import Float64


class pci3346(object):

    def __init__(self):
        self.da = pyinterface.open(3346,rsw_id)
        self.da.initialize()
        topic_li = []

        for ch in ch_num_li:
            rospy.Subscriber(name = "/dev/pci3346A/rsw%d/ch%d"%(rsw_id,ch),
                            data_class = Float64,
                            callback = self.callback,
                            callback_args = ch,
                            queue_size = 1
                            )

        self.ch_list = [eval(rospy.get_param(i)) for i in _ch_name_li]
        self.data_li = []
        self.data_dic = {}
        pass

    def callback(self,q,arg):
        self.data_dic[arg] = q.data
        pass

    def output_vol(self):
        while not rospy.is_shutdown():
            data_dic_list = list(self.data_dic)
            self.data_li = []
            for tp in data_dic_list:
                data = self.data_dic[tp]
                self.data_li.append(data)
            self.da.output_da(self.ch_list, self.data_li)
            continue

    def start_thread(self):
        th = threading.Thread(target=self.output_vol)
        th.setDaemon(True)
        th.start()

if __name__ == '__main__':
    rospy.init_node('pci3346')
    rsw_id = rospy.get_param('~rsw_id')
    all_ch_num = rospy.get_param("~all_ch_num")
    ch_num_li = eval(rospy.get_param("~ch_num_li"))
    _ch_name_li = [ "~ch%s"%(i) for i in ch_num_li]

    ctrl = pci3346()
    ctrl.start_thread()
    rospy.spin()
