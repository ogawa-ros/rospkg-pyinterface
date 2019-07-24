#! /usr/bin/env python3
import rospy
import os, sys, time ,datetime
import numpy as np
import matplotlib.pyplot as plt

import std_msgs
from std_msgs.msg import Float64
from std_msgs.msg import String
from std_msgs.msg import Int32

sys.path.append("/home/root/ros/src/rospkg-pyinterface/scripts")
import tz_reader

os.chdir("/home/forest/Desktop/data")

class sis_iv(object):
    def __init__(self):

        self.pub_vol_ch1 = rospy.Publisher("sis_vol_cmd_ch1", Float64, queue_size=1)
        self.pub_vol_ch2 = rospy.Publisher("sis_vol_cmd_ch2", Float64, queue_size=1)
        self.pub_vol_ch3 = rospy.Publisher("sis_vol_cmd_ch3", Float64, queue_size=1)
        self.pub_vol_ch4 = rospy.Publisher("sis_vol_cmd_ch4", Float64, queue_size=1)

        self.t = datetime.datetime.now()
        self.ut = self.t.strftime("%Y%m%d-%H%M%S")

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
            time.sleep(0.1)
            ret = reader.iv_reader()
            time.sleep(0.01)
            da.append(ret[0])
            da.append(ret[1])
            da.append(ret[2])
            da.append(ret[3])
            da.append(ret[4])
            da.append(ret[5])
            da.append(ret[6])
            da.append(ret[7])
            print(da)
            da_all.append(da)
            time.sleep(0.01)
        np.savetxt("sis_iv_{0}.txt".format(self.ut,save_name), np.array(da_all), delimiter=" ")
        iv.iv_plot()


    def iv_plot(self):
        iv = np.loadtxt("sis_iv_{0}.txt".format(self.ut,save_name))

        plt.figure()
        plt.subplot(221)
        plt.scatter(iv[:,0], iv[:,1], marker=".", color="red")
        plt.xlabel("voltage[mV]")
        plt.ylabel("current[uA]")
        plt.title("SIS-IV_ch1")
        plt.grid(True)

        plt.subplot(222)
        plt.scatter(iv[:,2], iv[:,3], marker=".", color="red")
        plt.xlabel("voltage[mV]")
        plt.ylabel("current[uA]")
        plt.title("SIS-IV_ch2")
        plt.grid(True)

        plt.subplot(223)
        plt.scatter(iv[:,4], iv[:,5], marker=".", color="red")
        plt.xlabel("voltage[mV]")
        plt.ylabel("current[uA]")
        plt.title("SIS-IV_ch3")
        plt.grid(True)

        plt.subplot(224)
        plt.scatter(iv[:,6], iv[:,7], marker=".", color="red")
        plt.xlabel("voltage[mV]")
        plt.ylabel("current[uA]")
        plt.title("SIS-IV_ch4")
        plt.grid(True)

        plt.subplots_adjust(wspace=0.4, hspace=0.6)
        plt.savefig("sis_iv_{0}.png".format(self.ut,save_name))
        plt.show()

if __name__ == "__main__" :
    rospy.init_node("sis_iv_measure")
    reader = tz_reader.reader()
    iv = sis_iv()
    initv = int(input("start_voltage = ? [mV]"))
    lastv = int(input("finish_voltage = ? [mV]"))
    interval = float(input("interval_voltage = ? [mV]"))
    save_name = str(input("save_name="))

    repeat = int((lastv-initv)/interval)
    sys.exit(iv.measure(initv,interval,repeat))

#20181204
#change for tz
#written by H.Kondo
