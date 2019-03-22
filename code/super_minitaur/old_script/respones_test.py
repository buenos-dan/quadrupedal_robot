#!/usr/bin/env python3
# coding:utf-8
import rospy
import odrive
from super_minitaur.msg import *
from settings import *
import numpy as np
import time

class Respones:
    """
    odrive读取数据速度测试
    """
    def __init__(self):
        rospy.init_node("test_node",anonymous=True)
        self.pub_pos = rospy.Publisher('pos_topic',FourLegPosMsg,queue_size=10)
        self.pub_cur = rospy.Publisher('cur_topic',FourLegCurrentMsg,queue_size=10)
        print('waiting Odrives')
        # self.odrive = odrive.find_any()
        # print('OK')
        self.spin()

    def spin(self):
        i = 0
        while 1:
            start = time.time()
            #self.pos_set = self.odrive.axis0.controller.pos_setpoint
            # self.odrive.axis0.controller.config.control_mode = 2
            # self.odrive.axis0.controller.vel_setpoint = 5000
            for i in range(1000):
                # self.pos_estimate1 = self.odrive.axis0.encoder.pos_estimate
                # self.pos_estimate2 = self.odrive.axis1.encoder.pos_estimate
                # self.current1 = self.odrive.axis0.motor.current_control.Iq_measured
                # self.current2 = self.odrive.axis0.motor.current_control.Iq_measured
                self.pos_estimate1 = i
                self.pos_estimate2 = i
                self.current1 = 10
                self.current2 = 30
                self.pub()
            end = time.time()
            print("执行时间%f", end-start)

    def pub(self):
        pos = FourLegPosMsg()
        cur = FourLegCurrentMsg()
        pos.RF_pos = [self.pos_estimate1]
        pos.RB_pos = [self.pos_estimate2]
        cur.RF_current = [self.current1]
        cur.RB_current = [self.current2]
        self.pub_pos.publish(pos)
        self.pub_cur.publish(cur)


if __name__ == '__main__':
    my_contorller = Respones()
