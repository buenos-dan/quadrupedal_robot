#!/usr/bin/env python
# coding:utf-8

import rospy
import time
from super_minitaur.msg import ImuMsg
from super_minitaur.msg import LegControlMsg
from super_minitaur.msg import FourLegCurrentMsg
from super_minitaur.msg import FourLegPosMsg
import numpy as np
from settings import *
from math import *


class InitStand:
    '''
    用于初始化时的站立
    完成的任务是机身水平和四条腿均接触地面
    '''
    def __init__(self):
        rospy.init_node("init_stand", anonymous=True)
        #读取imu的数据
        self.leg_pubs = [rospy.Publisher(name, LegControlMsg, queue_size = 10) for name in PUBLISHERS_NAME['four_leg']]
        self.imu_subscriber = rospy.Subscriber("imu_driver", ImuMsg, self.imu_callback)
        self.odr_current_subscriber = rospy.Subscriber("Odrive/actual_current", FourLegCurrentMsg, self.odr_current_callback)
        self.odr_pos_subscriber = rospy.Subscriber("Odrive/actual_pos", FourLegPosMsg, self.odr_pos_callback)
        #向odrive发出指令
        self.bodylength = BODY_LENGTH[1]
        self.bodywidth = BODY_LENGTH[0]
        self.control_mode_pos = 3   #位置模式
        self.x_pos = np.zeros(4)    # 输出的位置坐标rf,rb,lf,lb
        self.y_pos = np.zeros(4)
        self.x_pos_old = [0.01, -0.03, 0.01, -0.03]
        self.y_pos_old = [-0.25, -0.25 , -0.25, -0.25]
        self.RF_pos = np.zeros(2)   # 读取的足端位置坐标
        self.RB_pos = np.zeros(2)
        self.LF_pos = np.zeros(2)
        self.LB_pos = np.zeros(2)
        self.RF_current = np.zeros(2)   # 读取的电机电流值
        self.RB_current = np.zeros(2)
        self.LF_current = np.zeros(2)
        self.LB_current = np.zeros(2)
        self.euler_new = {'pitch':0, 'roll':0, 'yaw':0}
        self.euler_old = {'pitch':0, 'roll':0, 'yaw':0}
        self.euler = {'pitch':0, 'roll':0, 'yaw':0}
        self.dx = 0      #重心偏移量
        self.leg = np.zeros(4)     #当前腿长，即髋关节到脚踝的距离
        self.LEG_LENGTH_MAX = 0.35      #腿长极限值
        self.LEG_LENGTH_MIN = 0.22
        self.state = " "

        self.spin()


    def spin(self):
        '''
        控制器主循环
        '''
        self.stand1()
        time.sleep(1)
        while not rospy.is_shutdown():
            self.state = input("change state: ")
            if self.state == "cog":
                self.dx = input("input dx:")
                self.cog_adjust(self.dx)

            # self.stand2()
            # time.sleep(1)
            # #self.stand()
            # #self.stand_balance()

    def stand1(self):
        '''
        初步站立,通过简单地给定四条腿的末端位置进行控制
        '''
        for i in range(4):
            if i == 0 or i == 2:
                self.x_pos[i] = 0.01
                self.y_pos[i] = -0.25  #实验后的
            else:
                self.x_pos[i] = -0.03
                self.y_pos[i] = -0.25  #实验后的
            self.pub_data()

    def stand2(self):
        '''
        初步站立,通过简单地给定四条腿的末端位置进行控制
        '''
        for i in range(4):
            if i == 0 or i == 2:
                self.x_pos[i] = 0.01
                self.y_pos[i] = -0.20  #实验后的
            else:
                self.x_pos[i] = -0.03
                self.y_pos[i] = -0.20  #实验后的
            self.pub_data()

    def stand_balance(self):
        '''
        保持机身水平;imu读取欧拉角
        欧拉角方向：面向轴看，逆时针方向角度增大
        '''
        #俯仰角pitch影响下
        self.euler['pitch'] = self.euler_new['pitch'] - self.euler_old['pitch']

        self.x_pos[1] = self.x_pos[1] + 0.5 * self.bodylength * (1 - cos(self.euler['pitch']*pi/180))
        self.x_pos[3] = self.x_pos[3] + 0.5 * self.bodylength * (1 - cos(self.euler['pitch']*pi/180))
        self.y_pos[1] = self.y_pos[1] + 0.5 * self.bodylength * sin(self.euler['pitch']*pi/180)
        self.y_pos[3] = self.y_pos[3] + 0.5 * self.bodylength * sin(self.euler['pitch']*pi/180)

        self.x_pos[0] = self.x_pos[0] - 0.5 * self.bodylength * (1 - cos(self.euler['pitch']*pi/180))
        self.x_pos[2] = self.x_pos[2] - 0.5 * self.bodylength * (1 - cos(self.euler['pitch']*pi/180))
        self.y_pos[0] = self.y_pos[0] - 0.5 * self.bodylength * sin(self.euler['pitch']*pi/180)
        self.y_pos[2] = self.y_pos[2] - 0.5 * self.bodylength * sin(self.euler['pitch']*pi/180)
        #滚转角roll影响下
        self.euler['roll'] = self.euler_new['roll'] - self.euler_old['roll']

        self.x_pos[0] = self.x_pos[0]
        self.x_pos[1] = self.x_pos[1]
        self.y_pos[0] = self.y_pos[0] - 0.5 * self.bodywidth * sin(self.euler['roll']*pi/180)
        self.y_pos[1] = self.y_pos[1] - 0.5 * self.bodywidth * sin(self.euler['roll']*pi/180)

        self.x_pos[2] = self.x_pos[2]
        self.x_pos[3] = self.x_pos[3]
        self.y_pos[2] = self.y_pos[2] + 0.5 * self.bodywidth * sin(self.euler['roll']*pi/180)
        self.y_pos[3] = self.y_pos[3] + 0.5 * self.bodywidth * sin(self.euler['roll']*pi/180)
        
        #发送坐标
        self.pub_data()
        #更新数值
        self.euler_old['pitch'] = self.euler_new['pitch']
        self.euler_old['roll'] = self.euler_new['roll']
    
    def read_pos_and_keep(self):
        '''
        读取当前足端的坐标并保持
        '''
        #读取当前足端坐标
        self.x_pos[0] = self.RF_pos[0]
        self.y_pos[0] = self.RF_pos[1]
        self.x_pos[1] = self.RB_pos[0]
        self.y_pos[1] = self.RB_pos[1]
        self.x_pos[2] = self.LF_pos[0]
        self.y_pos[2] = self.LF_pos[1]
        self.x_pos[3] = self.LB_pos[0]
        self.y_pos[3] = self.LB_pos[1]
        self.pub_data()

    def cog_adjust(self, dx):
        '''
        调节重心，给出重心在x方向的移动dx
        机器人向后移动重心，dx > 0
        机器人向前移动重心，dx < 0
        '''
        #self.read_pos_and_keep()
        time.sleep(0.05)
        #计算腿长
        for i in range(4):
            self.leg[i] = sqrt(pow(self.x_pos[i], 2) + pow(self.y_pos[i], 2))
        #重心偏移量计算
        for i in range(4):
            self.x_pos[i] = self.x_pos[i] + dx
            self.y_pos[i] = -self.leg[i] * cos(asin((self.x_pos[i] + dx) / self.leg[i]))
        self.pub_data()
        
    
    def cog_self_adjust(self):
        '''
        读取电流值，自动调节机器人重心进行平衡
        '''
        pass


    def pub_data(self):
        '''
        进行安全判断并发送数据
        '''
        #安全检测 检测给定的坐标是否超出腿长的范围
        #计算腿长
        for i in range(4):
            self.leg[i] = sqrt(pow(self.x_pos[i], 2) + pow(self.y_pos[i], 2))
        #
        for i in range(4):
            if self.leg[i] > self.LEG_LENGTH_MAX or self.leg[i] < self.LEG_LENGTH_MIN:
                self.x_pos[i] = self.x_pos_old[i]
                self.y_pos[i] = self.y_pos_old[i]
        #发送数据
        legControlMsg = LegControlMsg()
        legControlMsg.control_mode = self.control_mode_pos
        legControlMsg.x = self.x_pos[0]
        legControlMsg.y = self.y_pos[0]
        self.leg_pubs[0].publish(legControlMsg)
        legControlMsg.x = self.x_pos[1]
        legControlMsg.y = self.y_pos[1]
        self.leg_pubs[1].publish(legControlMsg)
        legControlMsg.x = self.x_pos[2]
        legControlMsg.y = self.y_pos[2]
        self.leg_pubs[2].publish(legControlMsg)
        legControlMsg.x = self.x_pos[3]
        legControlMsg.y = self.y_pos[3]
        self.leg_pubs[3].publish(legControlMsg)
        #更新数据
        for i in range(4):
            self.x_pos_old[i] = self.x_pos[i]
            self.y_pos_old[i] = self.y_pos[i]

    def imu_callback(self,msg = ImuMsg):
        self.euler_new['pitch'] = msg.pitch
        self.euler_new['roll'] = msg.roll
        self.euler_new['yaw'] = msg.yaw
        #print msg

    def odr_current_callback(self, msg = FourLegCurrentMsg):
        self.RF_current = msg.RF_current
        self.RB_current = msg.RB_current
        self.LF_current = msg.LF_current
        self.LB_current = msg.LB_current

    def odr_pos_callback(self, msg = FourLegPosMsg):
        self.RF_pos = msg.RF_pos
        self.RB_pos = msg.RB_pos
        self.LF_pos = msg.LF_pos
        self.LB_pos = msg.LB_pos

if __name__ == "__main__":
    myrobotstand = InitStand()
