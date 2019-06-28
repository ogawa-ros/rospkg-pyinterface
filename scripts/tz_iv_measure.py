#! /usr/bin/env python3
import rospy
import os, sys, time ,datetime

import std_msgs
from std_msgs.msg import Float64
from std_msgs.msg import String
from std_msgs.msg import Int32

class sis_iv(object):
    def __init__(self):

        #self.pub_vol_ch_all = rospy.Publisher("/necst/rx_sis2sb/vgap_cmd", Float64, queue_size=1)
        self.pub_vol_ch_all = rospy.Publisher("/tz2019/sis_v1/vgap_cmd", Float64, queue_size=1)

        self.pub_path = rospy.Publisher("/logger_path", String, queue_size=1)
        self.pub_rate = rospy.Publisher("/dev/pci3177/rsw0/pub_rate", Float64, queue_size=1)

    def set(self):
        self.pub_rate.publish(0.1)
        self.pub_path.publish("/home/exito/data/logger/test/20190628/%s"%(save_name))
        pass

    def measure(self, initv, interval, repeat):
        da_all = []
        self.pub_vol_ch_all.publish(initv)
        time.sleep(0.3)
        for i in range(repeat+1):
            time.sleep(1)
            vol = initv+interval*i
            msg = Float64()
            msg.data = vol
            self.pub_vol_ch_all.publish(msg)
        self.pub_path.publish("")



if __name__ == "__main__" :
    rospy.init_node("iv_measure")

    """
    initv = int(input("start_voltage = ? [mV]"))
    lastv = int(input("finish_voltage = ? [mV]"))
    interval = float(input("interval_voltage = ? [mV]"))
    """
    init_vgap = float(input("start_Vgap = ? [Vgap]"))
    last_vgap = float(input("finish_Vgap = ? [Vgap]"))
    interval_vgap = float(input("interval_Vgap = ? [Vgap]"))
    save_name = str(input("savename = ? "))
    iv = sis_iv()
    #iv.set()
    #repeat = int((lastv-initv)/interval)
    repeat_vgap = int(abs((last_vgap-init_vgap)/interval_vgap))
    sys.exit(iv.measure(init_vgap,interval_vgap,repeat_vgap))

#20181204
#change for tz
#written by H.Kondo
