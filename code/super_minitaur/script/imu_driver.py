#!/usr/bin/env python
# coding:utf-8
import threading
import time
import rospy
import Queue
from super_minitaur.srv import StateFlag
from collections import deque
import numpy as np
from threading import Thread
from super_minitaur.msg import ImuMsg
from lpmslib import LpmsME
from settings_new import *
class ImuDriver:

    def __init__(self):
        self.imu_publisher = rospy.Publisher('super_minitaur/imu_data', ImuMsg, queue_size=10)
        self.imu_clinet = rospy.ServiceProxy("/state_flag_imu",StateFlag)
        self.lpmsSensor = LpmsME.LpmsME(IMU_PORT, IMU_BAUDRATE)
        self.disconnect()
        self.lpmsSensor.connect()
        self.freq = IMU_FREQENCY
        self.euler = ['', '', '']  # row pitch yaw
        self.pitch_deque = deque(maxlen = 200)
        self.old_flag = 0

    def spin(self):
        while not rospy.is_shutdown():
            self.get_data()
            self.pub()
            self.pitch_deque.append(self.euler[1])  # pitch角机体上坡为正
            if np.mean(np.array(self.pitch_deque)[:len(self.pitch_deque)/2]) < -10:  # 前2s 向上倾斜
                if abs(np.mean(np.array(self.pitch_deque)[len(self.pitch_deque)/2:])) < 1:  # 后2s imu基本水平
                    flag = 1
                    if flag - self.old_flag:
                        # self.imu_clinet.call([1])
                        print("flag changed: {}".format(flag))
                        break
                    self.old_flag = flag
            time.sleep(1/self.freq)
        print("Finish!")

    def disconnect(self):
        self.lpmsSensor.disconnect()

    def is_connected(self):                           #连接状态检测
        self.lpmsSensor.is_connected()

    def get_data(self):
        sensor_data = self.lpmsSensor.get_stream_data()
        # self.acc = sensor_data[4]                     #加速度
        # self.imu_id=sensor_data[0]
        # self.timestamp=sensor_data[1]
        # self.framecounter=sensor_data[2]
        # self.temperature=sensor_data[3]
        # self.gyr=sensor_data[5]
        # self.mag=sensor_data[6]
        # self.quat=sensor_data[7]
        self.euler = sensor_data[8]                     #欧拉角
        # self.linacc=sensor_data[9]

    def pub(self):
        #ImuMsg
        msg = ImuMsg()
        msg.roll = self.euler[0]
        msg.pitch = self.euler[1]
        msg.yaw = self.euler[2]
        # msg.xrev = self.gyr[0]
        # msg.yrev = self.gyr[1]
        # msg.zrev = self.gyr[2]
        self.imu_publisher.publish(msg)

def task(queue):
    imu = ImuDriver()
    queue.put("1")
    imu.spin()


if __name__=="__main__":
    queue =Queue.Queue()
    flag = 0          #标志位 0没连接上  1连接上
    i=1               #计次
    rospy.init_node("imu_node",anonymous=True)
    rospy.set_param('~/Imu_ready_flag',0)
    while(1):
        while flag == 0 :
            p = threading.Thread(target=task,args=(queue,))
            p.start()
            time.sleep(2)
            if queue.empty()!=True :
                print "第%d次连接结果："%i+queue.get()
                flag = 1
                rospy.set_param('~/Imu_ready_flag',1)
            else :
                print "第%d次连接结果："%i+"失败"
            i=i+1


