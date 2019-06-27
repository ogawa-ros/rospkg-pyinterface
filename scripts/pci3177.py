#! /usr/bin/env python3

import sys
import time
import pyinterface
import threading
import rospy
from std_msgs.msg import Float64


class pci3177(object):
    def __init__(self):

        self.pub_rate = rospy.get_param("~pub_rate")
        self.ave_num = rospy.get_param('~ave_num')
        self.smpl_freq = rospy.get_param("~smpl_freq")

        self.ad = pyinterface.open(3177, rsw_id)
        self.ad.stop_sampling()
        self.ad.initialize()
        self.ad.set_sampling_config(smpl_ch_req=smpl_ch_req,
                               smpl_num=1000,
                               smpl_freq=self.smpl_freq,
                               single_diff=single_diff,
                               trig_mode='ETERNITY'
                               )
        self.ad.start_sampling('ASYNC')
        self.pub_list = [rospy.Publisher("/dev/pci3177/rsw%d/ch%d"%(rsw_id,ch), Float64, queue_size=1)
                               for ch in ch_num_li]

        rospy.Subscriber("/dev/pci3177/rsw%d/pub_rate"%(rsw_id),Float64, self.pub_rate_set)
        pass

    def get_data(self):
        offset = self.ad.get_status()['smpl_count']-self.ave_num
        data = self.ad.read_sampling_buffer(self.ave_num, offset)
        data_li_2 = []
        for i in range(all_ch_num)
            data_li = []
            for k in range(ave_num)
                data_li.append(data[k][i])
            data_li_2.append(data_li)

        ave_data_li = []
        for data in data_li_2:
            d = sum(data)/self.ave_num
            ave_data_li.append(d)
        return ave_data_li

    """
    def get_data(self):
        offset = self.ad.get_status()['smpl_count']-self.ave_num
        data = self.ad.read_sampling_buffer(self.ave_num, offset)
        ave_data_dic = {}
        data_li = [data[:][i] for i in range(all_ch_num)]
        for i in range(all_ch_num):
            d = sum(data_li[i])/self.ave_num
            ave_data_dic["ch%d"%(i)] = d
        return ave_data_dic

    def pub_data(self):
        while not rospy.is_shutdown():
            time.sleep(self.pub_rate)
            data = self.get_data()
            for i in range(all_ch_num):
                self.pub_list[i].publish(data["ch%d"%(i)])
            continue
    """
    def pub_data(self):
        while not rospy.is_shutdown():
            time.sleep(self.pub_rate)
            data = self.get_data()
            for i in range(all_ch_num):
                self.pub_list[i].publish(data[i])
            continue


    def pub_rate_set(self,q):
        self.pub_rate = q.data
        self.ave_num = int(self.smpl_freq * self.pub_rate)

    def start_thread(self):
        th = threading.Thread(target=self.pub_data)
        th.setDaemon(True)
        th.start()

if __name__ == '__main__':
    rospy.init_node('pci3177')

    rate = rospy.get_param('~rate')
    rsw_id = rospy.get_param('~rsw_id')
    all_ch_num = rospy.get_param('~all_ch_num')
    ch_num_li = eval(rospy.get_param('~ch_num_li'))
    single_diff = rospy.get_param('~single_diff')


    _ch_name_li = [ "~ch%s"%(i) for i in ch_num_li]
    ch_list = [rospy.get_param(i) for i in _ch_name_li]

    smpl_ch_req = []
    for i in ch_list:
        smpl_ch_req.append(eval(i))

    ctrl = pci3177()
    ctrl.start_thread()
    rospy.spin()
