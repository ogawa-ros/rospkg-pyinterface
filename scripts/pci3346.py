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
        da = pyinterface.open(3346,rsw_id)
        da.initialize()
        topic_li = []
        for tp in topic_li:
            rospy.Subscriber(tp,Float64,self.callback) callback_args=

        self.data_li = []
        self.data_dic = {}
        pass

    def callback(self,q,args):
        self.data_dic[args] = q.data
        pass

    da.output_da([{'ch_no': 1, 'range': '5V'},
                  	{'ch_no': 2, 'range': '5V'},
                  	{'ch_no': 3, 'range': '5V'},
                  	{'ch_no': 4, 'range': '5V'}],
                 	[1, 1, 1,1])



    def pub_vol(self):
        while not rospy
            for data in data_dic:
                self.data_li.append(data)
            self.da.output_da(self.ch_list, self.data_li)
            continue

    def start_thread(self):
        th = threading.Thread(target=self.output_voltage)
        th.setDaemon(True)
        th.start()

if __name__ == '__main__':
    rospy.init_node('pci3346')
    rospy.get_param("all_ch_num")
    rospy.get_param("ch_num_li")

    ctrl = pci3346()

    _ch_name_li = [ "~ch%s"%(i) for i in ch_num_li]
    ch_list = [eval(rospy.get_param(i)) for i in _ch_name_li]

    ctrl.start_thread()
    rospy.spin()
