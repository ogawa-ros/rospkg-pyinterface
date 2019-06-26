#! /usr/bin/env python3
import rospy
import os, sys, time ,datetime

import std_msgs
from std_msgs.msg import Float64
from std_msgs.msg import String
from std_msgs.msg import Int32

class sis_iv(object):
    def __init__(self):

        self.pub_vol_ch1 = rospy.Publisher("/dev/pci3346A/rsw0/ch1", Float64, queue_size=1)
        self.pub_vol_ch2 = rospy.Publisher("/dev/pci3346A/rsw0/ch1", Float64, queue_size=1)
        self.pub_vol_ch3 = rospy.Publisher("/dev/pci3346A/rsw0/ch1", Float64, queue_size=1)
        self.pub_vol_ch4 = rospy.Publisher("/dev/pci3346A/rsw0/ch1", Float64, queue_size=1)
        self.pub_vol_ch_all = rospy.Publisher("/necst/rx_sis2sb/vgap_cmd", Float64, queue_size=1)

    def measure(self, initv, interval, repeat):
        da_all = []
        self.pub_vol_ch1.publish(initv)
        self.pub_vol_ch2.publish(initv)
        self.pub_vol_ch3.publish(initv)
        self.pub_vol_ch4.publish(initv)
        time.sleep(0.3)
        for i in range(repeat+1):
            da = []
            vol = initv+interval*i
            msg = Float64()
            msg.data = vol
            self.pub_vol_ch1.publish(msg)
            self.pub_vol_ch2.publish(msg)
            self.pub_vol_ch3.publish(msg)
            self.pub_vol_ch4.publish(msg)

    def measure2(self, initv, interval, repeat):
        da_all = []
        self.pub_vol_ch_all.publish(initv)
        time.sleep(0.3)
        for i in range(repeat+1):
            time.sleep(0.1s)
            da = []
            vol = initv+interval*i
            msg = Float64()
            msg.data = vol
            self.pub_vol_ch_all.publish(msg)



if __name__ == "__main__" :
    rospy.init_node("iv_measure")
    iv = sis_iv()
    """
    initv = int(input("start_voltage = ? [mV]"))
    lastv = int(input("finish_voltage = ? [mV]"))
    interval = float(input("interval_voltage = ? [mV]"))
    """
    init_vgap = int(input("start_vgap = ? [mV]"))
    last_vgap = int(input("finish_vgap = ? [mV]"))
    interval_vgap = float(input("interval_voltage = ? [mV]"))

    #repeat = int((lastv-initv)/interval)
    repeat_vgap = int((last_vgap-init_vgap)/interval_vgap)
    sys.exit(iv.measure2(init_vgap,interval_vgap,repeat_vgap))

#20181204
#change for tz
#written by H.Kondo
