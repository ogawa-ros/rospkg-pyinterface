#! /usr/bin/env python3


import sys
import time
import numpy
import pyinterface

import rospy
from std_msgs.msg import Float64


class pci3177(object):
    def __init__(self):

        self.ad = pyinterface.open(3177, rsw_id)
        self.ad.initialize()
        self.ad.set_sampling_config(smpl_ch_req=smpl_ch_req,
                               smpl_num=1000,
                               smpl_freq=100,
                               single_diff=single_diff,
                               trig_mode='ETERNITY'
                               )

        self.pub_list = [rospy.Publisher("/dev/pci3177/rsw/ch", Float64, queue_size=1)
                               for ch in ch_num]

        pass

    def get_data(self):
        self.ad.start_sampling('ASYNC')
        data = ad.read_sampling_buffer(ave_num, ad.get_status()['smpl_count']-ave_num)
        data_li = [data[:][i] for i in all_ch_num]
        ave_data_li = []
        for data in data_li:
            d = sum(data)/ave_num
            ave_data_li.append(d)
        return ave_data_li

    def pub_data(self):
        while not rospy.is_shutdown():
            data = self.get_data()
            for i in all_ch_num:
                self.pub_list[i].publish(data[i])
            continue


if __name__ == '__main__':
    rospy.init_node('pci3177')

    rate = rospy.get_param('~rate')
    rsw_id = rospy.get_param('~rsw_id')
    all_ch_num = rospy.get_param('~all_ch_num')
    ch_num_li = rospy.get_param('~ch_num_li')
    single_diff = rospy.get_param('~single_diff')

    ave_num = rospy.get_param('~ave_num')
    ch_list = [rospy.get_param("~ch%s"%(i)) for i in ch_num_li]

    smpl_ch_req = []
    for i in ch_list:
        smpl_ch_req.append(eval(i))

    ad = pci3177()
