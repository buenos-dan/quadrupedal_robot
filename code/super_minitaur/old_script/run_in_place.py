#!/usr/bin/env python3
# coding:utf-8
import rospy
from super_minitaur.msg import *
from settings import *
import numpy as np
import time
import math
from squat import *

class Run_in_place:
    """
    蹲起测试
    """
    def __init__(self):
        # ros相关初始化
        rospy.init_node('Run_in_place', anonymous=True)
        self.rate = rospy.Rate(10)#spin循环频率10HZ

        self.imu_subber = rospy.Subscriber('imu_drive', ImuMsg, self.imu_callback)
        self.pos_subber = rospy.Subscriber('Odrive/actual_pos', FourLegPosMsg, self.pos_callback)
        self.current_subber = rospy.Subscriber('Odrive/actual_current', FourLegCurrentMsg, self.current_callback)

        self.leg_pubs = [rospy.Publisher(name, LegControlMsg, queue_size=10) for name in PUBLISHERS_NAME['four_leg']]
        self.control_para_publisher = rospy.Publisher('Odrive/parameter', OdriveParamMsg, queue_size=10)

        self.x = np.zeros(4)
        self.y = np.zeros(4)
        self.spin()

    def spin(self):
        while not rospy.is_shutdown():
            pass

    def up_and_down(self, t, bottom=-0.2, up=-0.25):
        if t<=0.5:

        pass

    # def end_point(self, t, step_length=0.2, step_high=0.2):
    #     slope = 2*step_length
    #     if slope == 0:
    #         pass
    #     if t <= 0.25:
    #         self.x = slope*t-step_length/2
    #         self.y = 2*(slope*t)*step_high/step_length
    #     elif t <= 0.5:
    #         t = t-0.25
    #         self.x = slope*t
    #         y = (x_scale_-x)/tan(phi/2) + init_end_point[1]
    #     else:
    #         t_ = 1 - t
    #         t_2 = 2*t_
    #         x_normalized = t_2
    #         x = x_scale_ * x_normalized + init_end_point[0]
    #         y = init_end_point[1]
    #         x = x - x_scale_/2


if __name__ == '__main__':
    my_contorller = Run_in_place()
