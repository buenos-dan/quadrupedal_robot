#!/usr/bin/env python3
# coding:utf-8
import rospy
from super_minitaur.msg import *
from settings import *
import numpy as np
import time

class Squat:
    """
    蹲起测试
    """
    def __init__(self):
        # ros相关初始化
        rospy.init_node('state_squat', anonymous=True)
        self.rate = rospy.Rate(10)#spin循环频率10HZ

        self.imu_subber = rospy.Subscriber('imu_drive', ImuMsg, self.imu_callback)
        #self.pos_subber = rospy.Subscriber('Odrive/actual_pos', FourLegPosMsg, self.pos_callback)
        self.current_subber = rospy.Subscriber('Odrive/actual_current', FourLegCurrentMsg, self.current_callback)

        self.leg_pubs = [rospy.Publisher(name, LegControlMsg, queue_size=10) for name in PUBLISHERS_NAME['four_leg']]
        self.control_para_publisher = rospy.Publisher('Odrive/parameter', OdriveParamMsg, queue_size=10)

        self.imu_msg = ImuMsg
        self.pos_msg = FourLegPosMsg
        self.x = np.zeros(4)
        self.y = np.zeros(4)
        self.tem =np.zeros(10)
        self.motor_x = np.zeros(4)
        self.motor_y = np.zeros(4)
        self.motor_current = np.zeros(8)
        self.state = " "
        self.spin()

    def spin(self):
        #蹲起主循环
        while not rospy.is_shutdown():
            self.state = input('change_state:')
            if self.state == "stand":
                self.stand_up()
                print("stand")
                self.state = " "
            if self.state == "squat":
                self.full_squat()
                print("squat")
                self.state = " "
            if self.state == "full_loop":
                while 1:
                    self.stand_up()
                    time.sleep(0.5)
                    self.full_squat()
                    time.sleep(0.5)

            if self.state == "trot":
                while 1:
                    self.trot1()
                    self.trot2()
            if self.state == "super_trot":
                while 1:
                    self.super_trot1()
                    self.super_trot2()
            if self.state == "jump":
                while 1:
                    self.jump()
                    time.sleep(0.5)
		    
    def jump(self):

        for i in range(4):
            self.x[i] = 0
            self.y[i] = -0.31 
        self.pub()
        time.sleep(0.15)

        for i in range(4):
            self.x[i] = 0
            self.y[i] = -0.25 
            self.pub()
        time.sleep(0.5)

        self.full_squat()
        time.sleep(0.5)
        
        

    def super_trot1(self):
        y_down = np.linspace(-0.2,-0.25,STEP_FREQUENCY)
        y_up = np.linspace(-0.25,-0.2,STEP_FREQUENCY)
        x_back= np.linspace(STEP_LENGTH/2,-STEP_LENGTH/2,STEP_FREQUENCY*2)
        x_forward= np.linspace(-STEP_LENGTH/2,STEP_LENGTH/2,STEP_FREQUENCY*2)
        for j in range(STEP_FREQUENCY*2):
            for i in range(4):
                if i == 0 or i == 3:
                    self.x[i] = x_back[j]
                    self.y[i] = -0.25  #随便给的
                else:
                    self.x[i] = x_forward[j]
                    if j//STEP_FREQUENCY==0:
                        self.y[i] = y_up[j%STEP_FREQUENCY]  #随便给的
                    if j//STEP_FREQUENCY==1:
                        self.y[i] = y_down[j%STEP_FREQUENCY]  #随便给的
            time.sleep(0.1/STEP_FREQUENCY)
            self.pub()

    def super_trot2(self):
        y_down = np.linspace(-0.2,-0.25,STEP_FREQUENCY)
        y_up = np.linspace(-0.25,-0.2,STEP_FREQUENCY)
        x_back= np.linspace(STEP_LENGTH/2,-STEP_LENGTH/2,STEP_FREQUENCY*2)
        x_forward= np.linspace(-STEP_LENGTH/2,STEP_LENGTH/2,STEP_FREQUENCY*2)
        for j in range(STEP_FREQUENCY*2):
            for i in range(4):
                if i == 1 or i == 2:
                    self.x[i] = x_back[j]
                    self.y[i] = -0.25  #随便给的
                else:
                    self.x[i] = x_forward[j]
                    if j//STEP_FREQUENCY==0:
                        self.y[i] = y_up[j%STEP_FREQUENCY]  #随便给的
                    if j//STEP_FREQUENCY==1:
                        self.y[i] = y_down[j%STEP_FREQUENCY]  #随便给的
            time.sleep(0.1/STEP_FREQUENCY)
            self.pub()

    def trot1(self):
        y_down = np.linspace(-0.2,-0.26,STEP_FREQUENCY)
        y_up = np.linspace(-0.26,-0.2,STEP_FREQUENCY)
        for j in range(STEP_FREQUENCY):
            for i in range(4):
                if i == 0 or i == 3:
                    self.x[i] = 0
                    self.y[i] = y_down[j]  #随便给的
                else:
                    self.x[i] = 0
                    self.y[i] = y_up[j]  #随便给
            time.sleep(0.2/STEP_FREQUENCY)
            self.pub()

    def trot2(self):
        y_up = np.linspace(-0.2,-0.26,STEP_FREQUENCY)
        y_down = np.linspace(-0.26,-0.2,STEP_FREQUENCY)
        for j in range(STEP_FREQUENCY):
            for i in range(4):
                if i == 0 or i == 3:
                    self.x[i] = 0
                    self.y[i] = y_down[j]  #随便给的
                else:
                    self.x[i] =0 
                    self.y[i] = y_up[j]  #随便给的
            time.sleep(0.2/STEP_FREQUENCY)
            self.pub()

    def stand_up(self):
        #从蹲下到站立，插10个值
        self.tem = np.linspace(-0.22,-0.26,10)
        for j in range(10):
            for i in range(4):
                if i == 0 or i == 2:
                    self.x[i] = 0
                    self.y[i] = self.tem[j]  #随便给的
                else:
                    self.x[i] =0 
                    self.y[i] = self.tem[j]  #随便给的
            self.rate.sleep()
            self.pub()

    def full_squat(self):
        #从蹲下到站立，插10个值
        self.tem = np.linspace(-0.25,-0.22,10)
        for j in range(10):
            for i in range(4):
                if i == 0 or i == 2:
                    self.x[i] = 0
                    self.y[i] = self.tem[j]  #随便给的
                else:
                    self.x[i] = 0
                    self.y[i] = self.tem[j]  #随便给的
            self.rate.sleep()
            self.pub()


    def pub(self):
        for i in range(4):
            leg_msg = LegControlMsg()
            leg_msg.control_mode = 3
            leg_msg.x = self.x[i]
            leg_msg.y = self.y[i]
            self.leg_pubs[i].publish(leg_msg)

    def imu_callback(self, msg=ImuMsg):
        self.imu_msg = msg

    def pos_callback(self, msg=FourLegPosMsg):
        self.pos_msg = msg
        self.motor_x[0] = self.pos_msg.RF_pos[0]
        self.motor_x[1] = self.pos_msg.RB_pos[0]
        self.motor_x[2] = self.pos_msg.LF_pos[0]
        self.motor_x[3] = self.pos_msg.LB_pos[0]

        self.motor_y[0] = self.pos_msg.RF_pos[1]
        self.motor_y[1] = self.pos_msg.RB_pos[1]
        self.motor_y[2] = self.pos_msg.LF_pos[1]
        self.motor_y[3] = self.pos_msg.LB_pos[1]

    def current_callback(self,msg=FourLegCurrentMsg):
        pass


if __name__ == '__main__':
    my_contorller = Squat()
