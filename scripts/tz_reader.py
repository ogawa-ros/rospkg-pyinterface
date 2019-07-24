#! /usr/bin/env python3
import rospy
import os, sys, time ,datetime

import std_msgs
from std_msgs.msg import Float64
from std_msgs.msg import String
from std_msgs.msg import Int32


class reader(object):
    def __init__(self):
        rospy.Subscriber("sis_vol_ch1", Float64, self.ch1_vol_switch)
        rospy.Subscriber("sis_cur_ch1", Float64, self.ch1_cur_switch)
        rospy.Subscriber("sis_vol_ch2", Float64, self.ch2_vol_switch)
        rospy.Subscriber("sis_cur_ch2", Float64, self.ch2_cur_switch)
        rospy.Subscriber("sis_vol_ch3", Float64, self.ch3_vol_switch)
        rospy.Subscriber("sis_cur_ch3", Float64, self.ch3_cur_switch)
        rospy.Subscriber("sis_vol_ch4", Float64, self.ch4_vol_switch)
        rospy.Subscriber("sis_cur_ch4", Float64, self.ch4_cur_switch)

        rospy.Subscriber("pm_power_ch1",Float64, self.ch1_power_switch)
        rospy.Subscriber("pm_power_ch2",Float64, self.ch2_power_switch)


###data###

    def ch1_vol_switch(self,q):
        self.ch1_vol = q.data

    def ch1_cur_switch(self,q):
        self.ch1_cur = q.data

    def ch2_vol_switch(self,q):
        self.ch2_vol = q.data

    def ch2_cur_switch(self,q):
        self.ch2_cur = q.data

    def ch3_vol_switch(self,q):
        self.ch3_vol = q.data

    def ch3_cur_switch(self,q):
        self.ch3_cur = q.data

    def ch4_vol_switch(self,q):
        self.ch4_vol = q.data

    def ch4_cur_switch(self,q):
        self.ch4_cur = q.data

    def ch1_power_switch(self,q):
        self.ch1_power = q.data

    def ch2_power_switch(self,q):
        self.ch2_power = q.data

###reader###

    def iv_reader(self):
        ad = []
        ad.append(self.ch1_vol)
        ad.append(self.ch1_cur)
        ad.append(self.ch2_vol)
        ad.append(self.ch2_cur)
        ad.append(self.ch3_vol)
        ad.append(self.ch3_cur)
        ad.append(self.ch4_vol)
        ad.append(self.ch4_cur)
        return ad

    def piv_reader(self):
        ad = []
        ad.append(self.ch1_vol)
        ad.append(self.ch1_cur)
        ad.append(self.ch1_power)
        ad.append(self.ch4_vol)
        ad.append(self.ch4_cur)
        ad.append(self.ch2_power)
        return ad

if __name__ == "__main__" :
    rospy.init_node("reader")
    rospy.spin()
