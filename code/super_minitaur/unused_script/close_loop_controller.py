#!/usr/bin/env python
# coding:utf-8
import rospy
import numpy as np
import time
import utils
from super_controller import LegControl
from math import *
from settings import *
from super_minitaur.msg import *
from std_msgs.msg import *

class CloseLoopController:
    def __init__(self):
        rospy.init_node('close_loop_controller',anonymous=True)
        self.LegControllers = [LegControl("RF"),
                               LegControl("RB"),
                               LegControl("LF"),
                               LegControl("LB")]
        self.imu_suber = rospy.Subscriber("imu_driver",ImuMsg,self.imu_callback)
        self.Euler = [0,0,0] #yaw,pitch,roll
        self.Rev = [0,0,0] #三轴角速度
        self.T = 1.0  # 每个相位的周期
        self.t = 0  # 相位中当前时间
        self.spin_rate = 0.01  # 控制频率
        self.phase = 0  # 相位,[0+phase,3-pahse]支撑相，[1-pahse,2+phase]摆动相
        # 支撑相参数
        self.Vd = 0.0  # 期望前进速度
        self.Wd = 0.0  # 期望z轴转速
        self.Hd = 0.3  # 期望高度
        # 摆动相参数
        self.x0 = [0.0,0.0]
        self.y0 = [-0.35,-0.35]
        self.xT = 0.15
        self.yH = 0.1
        #虚拟力计算所需上次参数
        self.Euler_old =[0,0,0]
        self.psi_y_old = 0.0
        self.hs_old = -(self.LegControllers[0+self.phase]+self.LegControllers[3-self.phase])/2
        self.run()

    def imu_callback(self,msg):
        self.Euler = [msg.yaw,msg.pitch,msg.roll]
        self.Rev = [msg.xrev,msg.yrev,msg.zrev]

    def run(self):
        while not rospy.is_shutdown():
            self.stance_control(self.phase)
            self.flight_control(self.phase)
            self.t += self.spin_rate
            self.t = self.T if self.t >= self.T else self.t
            time.sleep(self.spin_rate)


    def stance_control(self,phase = 0):
        virtual_force_mat = self.virtual_force_cal(phase)
        torque_mat = self.torque_cal(phase,virtual_force_mat)
        curret_mat = utils.TorqueToCur(torque_mat)
        self.LegControllers[0+phase].WriteCurMsg([curret_mat[0][0],curret_mat[2][0]])
        self.LegControllers[3-phase].WriteCurMsg([curret_mat[3][0],curret_mat[5][0]])

    def virtual_force_cal(self,phase = 0):

        K_dict = {'K_thi':0.1, 'K_thi_':0.1,
                  'K_psi':0.1, 'K_psi_':0.1,
                  'K_h': 0.1, 'K_h_':0.1,
                  'K_vx':0.1,
                  'K_wz':0.1}

        leg1 = self.LegControllers[0+phase]
        leg2 = self.LegControllers[3-phase]
        xf = 0.23+leg1.actual_pos[0]
        zf = leg1.actual_pos[1]
        xb = -0.23+leg2.actual_pos[0]
        zb = leg2.actual_pos[1]
        Mg = 10*9.8

        Euler_new = self.Euler * pi / 180
        Euler_change = Euler_new-self.Euler_old
        Tx = K_dict['K_thi'] * Euler_new[2] + K_dict['K_thi_'] * Euler_change[2]

        psi_y_new = atan((zf-zb)/(xf-xb))
        psi_y_change = psi_y_new - self.psi_y_old
        Ty = K_dict['K_psi'] * psi_y_new + K_dict['K_psi_'] * psi_y_change

        hs_new = -(zf+zb)/2
        hs_change = hs_new - self.hs_old
        Fz = K_dict['K_h'] * (hs_new - self.Hd) + K_dict['K_h_'] * hs_change

        #电机角速度矩阵
        leg1_rev_mat = [
                        [leg1.actual__motor_rev[0]],
                        [0],
                        [leg1.actual__motor_rev[1]]
        ]
        leg2_rev_mat = [
                        [leg2.actual__motor_rev[0]],
                        [0],
                        [leg2.actual__motor_rev[1]]
        ]
        V_mat = leg1.calculate_jacob()*leg1_rev_mat+leg2.calculate_jacob()*leg2_rev_mat
        Fx = K_dict['K_vx'] * (V_mat[0][0] - self.Vd)

        Tz = K_dict['K_wz'] * (Euler_change[0] - self.Wd)
        
        self.Euler_old = Euler_new
        self.psi_y_old = psi_y_new
        self.hs_old = hs_new
        
        virtual_force = [
            [Fx-Mg*sin(self.Euler[1])],
            [Fz-Mg*cos(self.Euler[1])],
            [Tx],
            [Ty],
            [Tz],
            [0]
        ]
        return np.mat(virtual_force)

    def torque_cal(self,phase = 0,virtual_force_mat):
        jacob_F = list(self.LegControllers[0+phase].calculate_jacob())
        jacob_B = list(self.LegControllers[3-phase].calculate_jacob())
        top = np.column_stack((jacob_F, np.zeros((3, 3))))
        bottom = np.cloumn_stack((np.zeros((3, 3)), jacob_B))
        big_jacob = np.row_stack((top, bottom))
        big_jacob_mat = np.mat(big_jacob)       #计算6*6雅可比矩阵
        QI_mat = utils.mat_Q(self.LegControllers[0+phase].actual_pos[0], self.LegControllers[0+phase].actual_pos[1], self.LegControllers[3-phase].actual_pos[0], self.LegControllers[3-phase].actual_pos[1])
        torque_mat = big_jacob_mat * QI_mat * virtual_force_mat
        return torque_mat

    def flight_control(self,phase = 0):
        # 摆动相使用位置控制，矩形轨迹
        if self.t < self.T/4:
            x = self.x0
            y = [self.y0[0]+self.yH*self.t/(self.T/4),self.y0[1]+self.yH*self.t/(self.T/4)]
        elif self.t <= T*3/4:
            y = [self.y0[0] + self.yH,self.y0[1] + self.yH]
            x = [self.x0[0] + self.xT*(self.t-self.T/4)/(self.T/2),self.x0[1] + self.xT*(self.t-self.T/4)/(self.T/2)]
        else:
            x = [self.x0[0] + self.xT,self.x0[1] + self.xT]
            y = [self.y0[0]+self.yH-self.yH*(self.t-self.T*3/4)/(self.T/4),self.y0[1]+self.yH-self.yH*(self.t-self.T*3/4)/(self.T/4)]
        if self.t>=self.T*3/4 and self.LegControllers[1-phase].is_touch(13) and self.LegControllers[2+phase].is_touch(13):
            # 如果两条摆动腿同时触地，将x0y0更新为当前支撑腿（即下一个相位的摆动腿）的实际坐标，并更新t，切换相位
            self.x0 = [self.LegControllers[0+phase].actual_pos[0],self.LegControllers[3-phase].actual_pos[0]]
            self.y0 = [self.LegControllers[0+phase].actual_pos[1],self.LegControllers[3-phase].actual_pos[1]]
            self.t = 0
            self.phase = 1 - self.phase
        elif self.t>=self.T*3/4 and self.LegControllers[1-phase].is_touch(13):
            self.LegControllers[2+phase].WritePosMsg[x[1],y[1]]
        elif self.t>=self.T*3/4 and self.LegControllers[2+phase].is_touch(13):
            self.LegControllers[1-phase].WritePosMsg[x[0],y[0]]
        else:
            self.LegControllers[1-phase].WritePosMsg[x[0],y[0]]
            self.LegControllers[2+phase].WritePosMsg[x[1],y[1]]




