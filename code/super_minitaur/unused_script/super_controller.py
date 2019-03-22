#!/usr/bin/env python
# coding:utf-8
import rospy
import numpy as np
import time
import utils
from math import *
from collections import deque
from settings import *
from super_minitaur.msg import *
from std_msgs.msg import *
import scipy.interpolate
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt


class LegControl:
    def __init__(self,name):
        self.name = name + "_leg"
        self.control_puber = rospy.Publisher("Odrive/" + name + "/control",LegControlMsg,queue_size=10)
        self.para_puber = rospy.Publisher("Odrive/parameter",OdriveParamMsg,queue_size=10)
        self.cur_suber = rospy.Subscriber("Odrive/" + name + "/actual_current",FourLegCurrentMsg,self.cur_callback)
        self.pos_suber = rospy.Subscriber("Odrive/" + name + "/actual_pos",FourLegPosMsg,self.pos_callback)
        self.motor_angle_suber = rospy.Subscriber("Odrive/" + name + "/actual_angle",FourLegPosMsg,self.motor_angle_callback)
        self.leg_name = name
        self.actual_pos = [0,0]  # [x,y] M
        self.actual_cur = [0,0]  # [I_M0,I_M1] A
        self.actual_motor_angle = [0,0]  # [front, rear] rad
        self.cur_deque = deque(maxlen=3)
        
    def is_touch(self,thres=25):
        '''
        test whether touch ground
        '''
        self.cur_deque.append(self.actual_cur)
        I_M0 = [item[0] for item in self.cur_deque]
        I_M1 = [item[1] for item in self.cur_deque]
        if np.var(I_M0) < 100 and np.mean(I_M0) > thres:
            return 1
        if np.var(I_M1) < 100 and np.mean(I_M1) > thres:
            return 1
        return 0

    def WritePosMsg(self,pos):
        self.control_puber.publish(LegControlMsg(3,pos[0],pos[1]))

    def WriteCurMsg(self,cur):
        self.control_puber.publish(LegControlMsg(1,cur[0],cur[1]))

    def WriteForce(self,force):
        '''
        设置足端力force[x分量,y分量]
        '''
        cur = utils.TorqueCal(force[0],force[1],self.actual_motor_angle[0],self.actual_motor_angle[1])
        rospy.loginfo("{}_leg current:front:{} rear:{}\n\t"
                      "angle:front:{} rear:{}\n".format(self.leg_name,cur[0],cur[1],self.actual_motor_angle[0]/pi*180,self.actual_motor_angle[1]/pi*180))
        self.WriteCurMsg(cur)

    def Set_para(self,pos_gain = ODRV_POS_GAIN,
                 vel_gain = ODRV_VEL_GAIN,
                 vel_integrator_gain = ODRV_VEL_INTEGRATOR_GAIN,
                 vel_limit = ODRV_VEL_LIMIT,
                 current_limit = ODRV_CURRENT_LIM):
        msg = OdriveParamMsg()
        msg.odrive_index = leg_cipher[self.leg_name][0]
        msg.pos_gain = pos_gain
        msg.vel_gain = vel_gain
        msg.vel_integrator_gain = vel_integrator_gain
        msg.vel_limit = vel_limit
        msg.current_limit = current_limit
        self.para_puber.publish(msg)

    def cur_callback(self,msg):
        self.actual_cur = [abs(msg.front_current),abs(msg.rear_current)]

    def pos_callback(self,msg):
        self.actual_pos = [msg.x,msg.y]

    def motor_angle_callback(self,msg):
        self.actual_motor_angle = [msg.x/180*pi,msg.y/180*pi]

    def calculate_jacob(self):
        jacobian = utils.Jacobian3(self.actual_motor_angle[0],self.actual_motor_angle[1])
        return jacobian

class minitaur:
    def __init__(self):
        rospy.init_node('super_control',anonymous=True)
        self.LegControllers = [LegControl("RF"),
                               LegControl("RB"),
                               LegControl("LF"),
                               LegControl("LB")]
        self.imu_suber = rospy.Subscriber("imu_driver",ImuMsg,self.imu_callback)
        self.task_suber = rospy.Subscriber("task_topic",UInt8,self.task_callback)
        self.Euler = [0,0,0]
        self.run()

    def MoveLegsP2P(self,target_pos=DEFAULT_POS,run_time = T_STAND,frequency = STEP_FREQUENCY):
        '''
        target_pos = [ [[x1,y1],[x2,y2],[x3,y3],[x4,y4]],
                        [[x1,y1],[x2,y2],[x3,y3],[x4,y4]],
                         ....	]
        T_STAND = 0.2           #float is necessary
        STEP_FREQUENCY = 25     #float is necessary
        '''
        run_time = float(run_time)
        target_pos_temp = list(target_pos)
        if len(target_pos_temp) == 0:
            rospy.logerr("target_pos is empty!")
            return -1
        if len(target_pos_temp) == 1:
            mode = 'linear'
        else:
            mode = 'cubic'

        target_pos_temp.insert(0,[leg.actual_pos for leg in self.LegControllers])
        #target_pos_temp = [ [[x1,y1],[x2,y2],[x3,y3],[x4,y4]],  actual pos
        #				[[x1,y1],[x2,y2],[x3,y3],[x4,y4]],
        #				[[x1,y1],[x2,y2],[x3,y3],[x4,y4]],
        #			     ....	]


        x = [[item[i][0] for item in target_pos_temp] for i in range(4)]
        y = [[item[i][1] for item in target_pos_temp] for i in range(4)]
        #x = [[x1,x1,x1,x1...],[x2,x2,x2,x2...],[x3,x3,x3,x3...],[x4,x4,x4,x4...]]

        # functs = [interp1d(x[i],y[i],kind = mode) for i in range(4)]
        functs = [0, 0, 0, 0]
        for i in range(4):
            if (x[i].count(x[i][1]) >= len(x[i]) - 1 or y[i].count(y[i][1]) >= len(y[i]) - 1):
                functs[i] = interp1d(x[i], y[i], kind='linear')
            else:
                functs[i] = interp1d(x[i], y[i], kind=mode)
        #functs = [f1,f2,f3,f4]

        xxs = [np.linspace(target_pos_temp[0][i][0],
                           target_pos_temp[-1][i][0], frequency) for i in range(4)]
        #xxs = [[x1,x1,x1,x1...],[x2,x2,x2,x2...],[x3,x3,x3,x3...],[x4,x4,x4,x4...]]

        yys = [functs[i](xxs[i]) for i in range(4)]
        #yys = [[y1,y1,y1,y1...],[y2,y2,y2,x2...],[x3,x3,x3,x3...],[x4,x4,x4,x4...]]

        for i in range(int(frequency)):
            for index,leg in enumerate(self.LegControllers):
                pos =  [xxs[index][i],yys[index][i]]
                leg.WritePosMsg(pos)
            #print self.LegControllers[2].is_touch(thres = 15)
            time.sleep(run_time/frequency)

    def run(self):
        while 1:
            command = raw_input("please input command:")
            if command == 'stand':
                self.StandInit()
            if command == 'jstand':
                self.jstand()
            if command == 'trot':
                self.Trot()
            if command == 'tiger_bound':
                self.tiger_bound()
            if command == 'jump':
                self.Jump()
            if command == 'jump_turn':
                self.JumpTurn()
            time.sleep(0.1)

    def StandInit(self):
        self.MoveLegsP2P(target_pos=[[[STAND_OFFSET[i], -0.23] for i in range(4)]])
        # raw_input('press any key to start!')
        # for leg in self.LegControllers:
        #     leg.WriteForce((0,-25))

    def Trot(self):
        #对角步态目前最佳参数T = 0.5 T1 = T/6, 步长25cm，频率50HZ
        T = 0.5
        t1 = T/6.0
        t2 = T/6.0*5
        step_length = 0.25
        min_height = -0.28
        step_height_up = 0.05
        step_height_down = 0.02
        spin_rate = 0.02
        t = 0.0
        for index,leg in enumerate(self.LegControllers):
            leg.Set_para(60,0.001)
        while not rospy.is_shutdown():
            t = t%T
            if t <t1:
                xs = t/t1*step_length-step_length/2
                ys = step_height_up*sin(t/t1*pi)+min_height
                self.LegControllers[0].WritePosMsg([xs+TROT_OFFSET[0],ys])
                self.LegControllers[3].WritePosMsg([xs+TROT_OFFSET[1],ys])
            elif (t-t1) < t2:
                xs = -(t-t1)/t2*step_length+step_length/2
                ys = step_height_down*-sin((t-t1)/t2*pi)+min_height
                self.LegControllers[0].WritePosMsg([xs+TROT_OFFSET[0],ys])
                self.LegControllers[3].WritePosMsg([xs+TROT_OFFSET[1],ys])
            t_other = t+T/2
            t_other=t_other%T
            if t_other < t1:
                xs = t_other/t1*step_length-step_length/2
                ys = step_height_up*sin(t_other/t1*pi)+min_height
                self.LegControllers[1].WritePosMsg([xs+TROT_OFFSET[1],ys])
                self.LegControllers[2].WritePosMsg([xs+TROT_OFFSET[0],ys])
            elif (t_other-t1) < t2:
                xs = -(t_other-t1)/t2*step_length+step_length/2
                ys = step_height_down*-sin((t_other-t1)/t2*pi)+min_height
                self.LegControllers[1].WritePosMsg([xs+TROT_OFFSET[1],ys])
                self.LegControllers[2].WritePosMsg([xs+TROT_OFFSET[0],ys])
            t += spin_rate
            time.sleep(spin_rate)
    def tiger_bound(self):
        #猛虎跳跃最佳参数：
        T = 0.5
        t1 = T/6.0*5
        t2 = T/6.0
        step_length = 0.2
        min_height = -0.25
        step_height_up = 0.02
        step_height_down = 0.05
        spin_rate = 0.02
        t = 0.0
        for index,leg in enumerate(self.LegControllers):
            leg.Set_para(60,0.001)
        while not rospy.is_shutdown():
            t = t%T
            if t <t1:
                xs = t/t1*step_length-step_length/2
                ys = step_height_up*sin(t/t1*pi)+min_height
                self.LegControllers[0].WritePosMsg([xs+TIGER_OFFSET[0],ys])
                self.LegControllers[2].WritePosMsg([xs+TIGER_OFFSET[0],ys])
            elif (t-t1) < t2:
                xs = -(t-t1)/t2*step_length+step_length/2
                ys = step_height_down*-sin((t-t1)/t2*pi)+min_height
                self.LegControllers[0].WritePosMsg([xs+TIGER_OFFSET[0],ys])
                self.LegControllers[2].WritePosMsg([xs+TIGER_OFFSET[0],ys])

            t_other = t+T/2
            t_other=t_other%T
            if t_other < t1:
                xs = t_other/t1*step_length-step_length/2
                ys = step_height_up*sin(t_other/t1*pi)+min_height
                self.LegControllers[1].WritePosMsg([xs+TIGER_OFFSET[1],ys])
                self.LegControllers[3].WritePosMsg([xs+TIGER_OFFSET[1],ys])
            elif (t_other-t1) < t2:
                xs = -(t_other-t1)/t2*step_length+step_length/2
                ys = step_height_down*-sin((t_other-t1)/t2*pi)+min_height
                self.LegControllers[1].WritePosMsg([xs+TIGER_OFFSET[1],ys])
                self.LegControllers[3].WritePosMsg([xs+TIGER_OFFSET[1],ys])
            t += spin_rate
            time.sleep(spin_rate)

    def jstand(self):
        self.MoveLegsP2P(target_pos=[[[JUMP_OFFSET[i],SQUAT_HEIGHT] for i in range(4)]], run_time=0.3, frequency=24)

    def Jump(self): # 19.1.24:极限电流设为25时，力量不足以起跳,极限7870电流为30时，左后腿odrive会掉电
        # 站立
        self.MoveLegsP2P(target_pos=[[[JUMP_OFFSET[i],SQUAT_HEIGHT] for i in range(4)]], run_time=0.3, frequency=24)
        time.sleep(1)
        # 腾空
        for index,leg in enumerate(self.LegControllers):
            leg.WriteCurMsg([30,30])
        time.sleep(0.15)
        # 收腿
        self.MoveLegsP2P(target_pos=[[[0.03,SQUAT_HEIGHT-0.05] for i in range(4)]], run_time=0.2, frequency=20)
        # 判断触地
        while 1:
            break_flag = 0
            for leg in self.LegControllers:
                if leg.is_touch(13):
                    rospy.loginfo('landing')
                    break_flag = 1
                    break
            if break_flag == 1:
                break
            rospy.loginfo('flying')
            time.sleep(0.01)
        # 落地站立
        self.MoveLegsP2P(target_pos=[[[STAND_OFFSET[i],SQUAT_HEIGHT] for i in range(4)]], run_time=0.2, frequency=20)

    def JumpTurn(self,direction='LEFT'):
        direct_dict = {'LEFT':1,"RIGHT":-1}
        k = direct_dict[direction]
        turn_offset = 0.03
        # 站立
        self.MoveLegsP2P(target_pos=[[[-k*turn_offset,SQUAT_HEIGHT],[-k*turn_offset,SQUAT_HEIGHT],[k*turn_offset,SQUAT_HEIGHT],[k*turn_offset,SQUAT_HEIGHT]]], run_time=0.3, frequency=24)
        time.sleep(1)
        while not rospy.is_shutdown():
            # 腾空
            for index,leg in enumerate(self.LegControllers):
                leg.WriteCurMsg([25,25])
            time.sleep(0.1)
            # 收腿
            self.MoveLegsP2P(target_pos=[[[STAND_OFFSET[i],SQUAT_HEIGHT-0.05] for i in range(4)]], run_time=0.2, frequency=16)
            while 1 :
                break_flag = 0
                for leg in self.LegControllers:
                    if leg.is_touch(13):
                        rospy.loginfo('landing')
                        break_flag = 1
                        break
                if break_flag == 1:
                    break
                rospy.loginfo('flying')
                time.sleep(0.001)
            # 落地站立
            self.MoveLegsP2P(target_pos=[[[STAND_OFFSET[i],SQUAT_HEIGHT] for i in range(4)]], run_time=0.2, frequency=32)

    def imu_callback(self,msg):
        self.Euler = [msg.yaw,
                      msg.pitch,
                      msg.roll]

    def task_callback(self,msg):
        self.tasks.append(msg.data)


if __name__ == '__main__':
    my_contorller = minitaur()



#
# def InitMinitaur(self):
#     while rospy.get_param('~/imu_ready_flag') == 0:
#         time.sleep(0.1)
#     rospy.loginfo("[+]imu is ready")
#
#     while rospy.get_param('~/Odrive_ready_flag') == 0:
#         time.sleep(0.1)
#     rospy.loginfo("[+]odrive is ready")
#
#     for leg in self.LegControllers:
#         leg.InitLeg()
#         rospy.loginfo("[+]{}_leg is ready".format(leg.name))
#
# def cog_adjust(self,dx):
#     leg_length = [sqrt(pow(leg.actual_pos[0],2)+pow(leg.actual_pos[1],2)) for leg in self.LegControllers]
#     minitaur_pos = [[[0., 0.] for i in range(4)]]
#     for index,leg in enumerate(self.LegControllers):       #赋初值
#         minitaur_pos[0][index] = leg.actual_pos
#     for i in range(4):
#         minitaur_pos[0][i][0] = minitaur_pos[0][i][0] + dx
#         minitaur_pos[0][i][1] = minitaur_pos[0][i][1] * cos(asin(minitaur_pos[0][i][0]/leg_length[i]))
#     self.MoveLegsP2P(target_pos = minitaur_pos, run_time = 0.5, frequency = 20)
#
# def run(self):
#     while 1:
#         command = raw_input("please input command:")
#         if command == 'test':
#             self.MoveLegsP2P(DEFAULT_POS,0.2)
#             DEFAULT_POS1 = [[[-0.03,-0.27] for i in range(4)],
#                             [[0.03,-0.27] for i in range(4)],
#                             [[0.06,-0.3] for i in range(4)]]
#             self.MoveLegsP2P(DEFAULT_POS1,0.3)
#         if command == 'jumpip':
#             for leg in self.LegControllers:
#                 leg.Set_para()
#             self.JumpInPlace()
#         if command == 'stand':
#             self.StandInit()
#         if command == 'trotip':
#             self.TrotInPlace()
#         if command == 'trot':
#             self.Trot()
#         if command == 'four_phase_trot':
#             self.four_phase_trot()
#         if command == 'fsf_trot':
#             self.fsf_trot()
#         if command == 'walk':
#             self.Walk()
#         if command == 'upanddown':
#             self.UpAndDown()
#         if command == 'balance':
#             self.stand_balance()
#         if command == 'jump':
#             self.Jump()
#         if command == 'jump_turn':
#             self.JumpTurn()
#         if command == 'turn':
#             self.Turn()
#         if command == 'jac':
#             print(self.Jacobian(1,-1,pi/4,pi*3/4))
#         if command == 'jstand':
#             self.jstand()
#         time.sleep(0.1)
#
# def UpAndDown(self):
#     self.MoveLegsP2P(target_pos=[[[0, -0.32] for i in range(4)]])
#     time.sleep(0.5)
#     self.MoveLegsP2P(target_pos=[[[0, -0.25] for i in range(4)]])
#
# def TrotInPlace(self):
#     xf=0.005
#     xb=-0.01
#     y1=-0.30
#     y2=-0.42
#     while 1:
#         # pitch=self.Euler(1)
#         target_RF = [xf, y1]
#         target_RB = [xb, y2]
#         target_LF = [xf, y2]
#         target_LB = [xb, y1]
#         # xf = xf+0.0005*pitch
#         self.MoveLegsP2P(target_pos=[[target_RF, target_RB, target_LF, target_LB]], run_time = 0.1, frequency = 2)
#         self.MoveLegsP2P(target_pos=[[target_LF, target_LB, target_RF, target_RB]], run_time = 0.1, frequency = 2)
#
# def Trot(self):
#     xf = DEFAULT_OFFSET[0]
#     xb = DEFAULT_OFFSET[1]
#     y1 = -0.23
#     y2 = -0.30
#     y3 = -0.33
#     xstep = 0.06
#     ystep = (y1-y2)/2
#     ystep_down = (y2-y3)/2
#     T= 0.25
#     target_RF = [xf+xstep/2, y2]
#     target_RB = [xb-xstep/2, y2]
#     target_LF = [xf-xstep/2, y2]
#     target_LB = [xb+xstep/2, y2]
#     self.MoveLegsP2P(target_pos=[[target_RF, target_RB, target_LF, target_LB]], run_time=0.5, frequency=40)
#     time.sleep(1)
#     while not rospy.is_shutdown():
#         # target_RF = [xf, y1]
#         # target_RB = [xb, y2]
#         # target_LF = [xf, y2]
#         # target_LB = [xb, y1]
#         pos1=[
#             [[xf-xstep/4,y2+ystep*3/2],    [xb+xstep/2,y2],                [xf+xstep/2,y2],                    [xb-xstep/4,y2+ystep*3/2]],
#             [[xf,y1],                      [xb+xstep/3,y2-ystep_down/3],   [xf+xstep/3,y2-ystep_down/3],       [xb,y1]],
#             [[xf+xstep/4,y2+ystep*3/2],    [xb+xstep/6,y2-ystep_down*2/3], [xf+xstep/6,y2-ystep_down*2/3],     [xb+xstep/4,y2+ystep*3/2]],
#             [[xf+xstep/2,y2],              [xb,y3],                        [xf,y3],                            [xb+xstep/2,y2]]
#         ]
#         pos2=[
#             [[xf+xstep/2,y2],    [xb+xstep/2,y2],                [xf+xstep/2,y2],                    [xb-xstep/4,y2+ystep*3/2]],
#             [[xb+xstep/3,y2-ystep_down/3],                      [xb+xstep/3,y2-ystep_down/3],   [xf+xstep/3,y2-ystep_down/3],       [xb,y1]],
#             [[xb+xstep/6,y2-ystep_down*2/3],    [xb+xstep/6,y2-ystep_down*2/3], [xf+xstep/6,y2-ystep_down*2/3],     [xb+xstep/4,y2+ystep*3/2]],
#             [[xb,y3],              [xb,y3],                        [xf,y3],                            [xb+xstep/2,y2]]
#         ]
#         self.MoveLegsP2P(target_pos=pos1, run_time=T, frequency=24)
#         pos2=[[pos1[i][2],pos1[i][3],pos1[i][0],pos1[i][1]]for i in range(4)]
#         self.MoveLegsP2P(target_pos=pos2, run_time=T, frequency=24)
#
# def fsf_trot(self):
#     T = 2.0
#     flight_scale = 1.0/8
#     step_length = 0.1
#     step_height = 0.12
#     min_height = -0.4
#
#     scale_ = 1/flight_scale
#     temp = (scale_/2 - 1)/(scale_ - 1)
#     x0 = -step_length/2
#     x1 = step_length*(-1.0/2+temp)
#     x2 = step_length*(1.0/2-temp)
#     x3 = step_length/2
#     # 用于插值曲线
#     x0_ = -step_length/2
#     x1_ = -step_length*3/8
#     x2_ = 0
#     x3_ = step_length*3/8
#     x4_ = step_length/2
#     y0 = min_height
#     y0_ = min_height
#     y1_ = y0+step_height*1/2
#     y2_ = y0+step_height
#     for leg in self.LegControllers:
#         leg.Set_para(60,0.001)
#     self.MoveLegsP2P(target_pos=[[[x0_,y0_],[x2,y0],[x2,y0],[x0,y0_]]],run_time=0.3,frequency=20)
#     while not rospy.is_shutdown():
#         self.MoveLegsP2P(target_pos=[[[x1_,y1_],[x1,y0],[x1,y0],[x1_,y1_]],
#                                      [[x2_,y2_],[x1,y0],[x1,y0],[x2_,y2_]],
#                                      [[x3_,y1_],[x1,y0],[x1,y0],[x3_,y1_]],
#                                      [[x4_,y0_],[x1,y0],[x1,y0],[x4_,y0_]]],
#                          run_time=T*flight_scale,frequency=24)
#         self.MoveLegsP2P(target_pos=[[[x2,y0],[x0,y0],[x0,y0],[x2,y0]]],run_time=T*(1.0/2-flight_scale),frequency=20)
#         self.MoveLegsP2P(target_pos=[[[x1,y0],[x1_,y1_],[x1_,y1_],[x1,y0]],
#                                      [[x1,y0],[x2_,y2_],[x2_,y2_],[x1,y0]],
#                                      [[x1,y0],[x3_,y1_],[x3_,y1_],[x1,y0]],
#                                      [[x1,y0],[x4_,y0_],[x4_,y0_],[x1,y0]]],
#                          run_time=T*flight_scale,frequency=24)
#         self.MoveLegsP2P(target_pos=[[[x0,y0],[x2,y0],[x2,y0],[x0,y0]]],run_time=T*(1.0/2-flight_scale),frequency=20)
#
# def JumpInPlace(self):
#     MoveLegsP2P(target_pos=[[[0,-0.25] for i in range(4)]])
#     time.sleep(1)
#     MoveLegsP2P(target_pos=[[[0,-0.31] for i in range(4)]], run_time=0, frequency=1)
#     time.sleep(0.15)
#     MoveLegsP2P(target_pos=[[[0,-0.25] for i in range(8)]], run_time=0, frequency=1)
# def BoundInPlace(self):
#     pass
# def Bound(self):
#     pass
# def Gallop(self):
#     pass
# def OneLegCogAdjust(self,index, dx, frequency):
#     leg_contrl = self.LegControllers[index]
#     leg_length = sqrt(pow(leg_contrl.actual_pos[0],2)+pow(leg_contrl.actual_pos[1],2))
#     cog_ajust_x = np.linspace(leg_contrl.actual_pos[0],leg_contrl.actual_pos[0]+dx,frequency)
#     cog_ajust_y = np.linspace(leg_contrl.actual_pos[1],leg_contrl.actual_pos[1]*cos(asin(leg_contrl.actual_pos[0]/leg_length)),frequency)
#     cog_ajust =[[cog_ajust_x[i],cog_ajust_y[i]]for i in range(frequency)]
#     return cog_ajust
#
# def Walk(self):
#     RF_x = -0.03
#     RB_x = -0.03
#     LF_x = 0.03
#     LB_x = 0.03
#     y1 = -0.23
#     y2 = -0.25
#     xstep = 0.06
#     ystep = y1-y2
#     cog_forward = 0.01  #减往前移，加往后移
#     cog_back = 0.00 #绝对值
#     T=1
#     leg = self.LegControllers
#     for i in range(4):
#         leg[i].Set_para(80,0.002,0.001,50000,30)
#     #初始化位置
#     target_RF = [RF_x, y2]
#     target_RB = [RB_x, y2]
#     target_LF = [LF_x, y2]
#     target_LB = [LB_x, y2]
#     self.MoveLegsP2P(target_pos=[[target_RF, target_RB, target_LF, target_LB]], run_time=0.5, frequency=40)
#     time.sleep(1)
#     while 1:
#         pos1=[
#             [[RF_x-cog_forward,y2],   [RB_x+xstep/4,y2+ystep], [LF_x-cog_forward,y2],    [LB_x-cog_forward,y2]],
#             [[RF_x-cog_forward,y2],   [RB_x+xstep/2,y1],       [LF_x-cog_forward,y2],    [LB_x-cog_forward,y2]],
#             [[RF_x-cog_forward,y2],   [RB_x+xstep*3/4,y1],     [LF_x-cog_forward,y2],    [LB_x-cog_forward,y2]],
#             [[RF_x-cog_forward,y2],   [RB_x+xstep,y2],         [LF_x-cog_forward,y2],    [LB_x-cog_forward,y2]],
#         ]
#         self.MoveLegsP2P(target_pos=pos1, run_time=T, frequency=24)
#         # while not self.LegControllers[1].is_touch(17):
#         #     print("not touched yet!!!!!!!")
#         time.sleep(0.5)
#
#         pos2=[
#             [[RF_x+xstep/4,y2+ystep], [RB_x+cog_back,y2], [LF_x+cog_back,y2],    [LB_x+cog_back,y2]],
#             [[RF_x+xstep/2,y1],       [RB_x+cog_back,y2], [LF_x+cog_back,y2],    [LB_x+cog_back,y2]],
#             [[RF_x+xstep*3/4,y1],     [RB_x+cog_back,y2], [LF_x+cog_back,y2],    [LB_x+cog_back,y2]],
#             [[RF_x+xstep,y2],         [RB_x+cog_back,y2], [LF_x+cog_back,y2],    [LB_x+cog_back,y2]],
#         ]
#         self.MoveLegsP2P(target_pos=pos2, run_time=T, frequency=24)
#         # while not self.LegControllers[0].is_touch(17):
#         #     print("not touched yet!!!!!!!")
#         time.sleep(0.5)
#
#         pos3=[
#             [[RF_x-cog_forward,y2],   [RB_x-cog_forward,y2],   [LF_x-cog_forward,y2],    [LB_x+xstep/4,y2+ystep]],
#             [[RF_x-cog_forward,y2],   [RB_x-cog_forward,y2],   [LF_x-cog_forward,y2],    [LB_x+xstep/2,y1]],
#             [[RF_x-cog_forward,y2],   [RB_x-cog_forward,y2],   [LF_x-cog_forward,y2],    [LB_x+xstep*3/4,y1]],
#             [[RF_x-cog_forward,y2],   [RB_x-cog_forward,y2],   [LF_x-cog_forward,y2],    [LB_x+xstep,y2]],
#         ]
#         self.MoveLegsP2P(target_pos=pos3, run_time=T, frequency=24)
#         # while not self.LegControllers[3].is_touch(17):
#         #     print("not touched yet!!!!!!!")
#         time.sleep(0.5)
#
#         pos4=[
#             [[RF_x+cog_back,y2],   [RB_x+cog_back,y2], [LF_x+xstep/4,y2+ystep], [LB_x+cog_back,y2]],
#             [[RF_x+cog_back,y2],   [RB_x+cog_back,y2], [LF_x+xstep/2,y1],       [LB_x+cog_back,y2]],
#             [[RF_x+cog_back,y2],   [RB_x+cog_back,y2], [LF_x+xstep*3/4,y1],     [LB_x+cog_back,y2]],
#             [[RF_x+cog_back,y2],   [RB_x+cog_back,y2], [LF_x+xstep,y2],         [LB_x+cog_back,y2]],
#         ]
#         self.MoveLegsP2P(target_pos=pos4, run_time=T, frequency=24)
#         # while not self.LegControllers[2].is_touch(17):
#         #     print("not touched yet!!!!!!!")
#         time.sleep(0.5)
#
# def Turn(self,direction='LEFT'):
#     direct_dict = {'LEFT':1,"RIGHT":-1}
#     k = direct_dict[direction]
#     self.MoveLegsP2P([[              [k*0.05,-0.26]  ,[k*0.05,-0.26]  ,[k*0.05,-0.26]  ,[k*0.05,-0.26]]], run_time=0.2, frequency=8)
#     while not rospy.is_shutdown():
#         self.MoveLegsP2P(target_pos=[[[k*0.02,-0.24],[k*0.02,-0.24] ,[k*0.02,-0.26]  ,[k*0.02,-0.26]],
#                                      [[k*-0.02,-0.24],[k*-0.02,-0.24],[k*-0.02,-0.26] ,[k*-0.02,-0.26]],
#                                      [[k*-0.05,-0.26] ,[k*-0.05,-0.26] ,[k*-0.05,-0.26] ,[k*-0.05,-0.26]]], run_time=0.25, frequency=16)
#         self.MoveLegsP2P(target_pos=[[[k*-0.02,-0.26],[k*-0.02,-0.26] ,[k*-0.02,-0.24],[k*-0.02,-0.24]],
#                                      [[k*0.02,-0.26],[k*0.02,-0.26]  ,[k*0.02,-0.24] ,[k*0.02,-0.24]],
#                                      [[k*0.05,-0.26],[k*0.05,-0.26]  ,[k*0.05,-0.26]  ,[k*0.05,-0.26]]], run_time=0.25, frequency=16)
#
# def InverseWalk(self):
#     pass
# def BackFlip(self):
#     pass
# def Fly(self):
#     pass
#
# def stand_balance(self):
#     """
#     读取imu数据
#     站立时保持平衡
#     """
#     Euler = [0., 0., 0.]
#     Euler_new = [0., 0., 0.]
#     Euler_old = [0., 0., 0.]
#     minitaur_pos = [[[0., 0.] for i in range(4)]]
#     for index,leg in enumerate(self.LegControllers):       #赋初值
#         minitaur_pos[0][index] = leg.actual_pos
#     while 1:
#         for i in range(3):                      #更新欧拉角
#             Euler_new[i] = self.Euler[i]
#             Euler[i] = Euler_new[i] - Euler_old[i]
#         print 1
#         # if abs(Euler[1]) > 0.1:
#         #     for i in range (4):#考虑pitch
#         #         if i == 0 or i == 2:
#         #             minitaur_pos[0][i][0] = minitaur_pos[0][i][0] - 0.5 * BODY_LENGTH[1] * (1 - cos(Euler[1]*pi/180))
#         #             minitaur_pos[0][i][1] = minitaur_pos[0][i][1] - 0.5 * BODY_LENGTH[1] * sin(Euler[1]*pi/180)
#         #         else:
#         #             minitaur_pos[0][i][0] = minitaur_pos[0][i][0] + 0.5 * BODY_LENGTH[1] * (1 - cos(Euler[1]*pi/180))
#         #             minitaur_pos[0][i][1] = minitaur_pos[0][i][1] + 0.5 * BODY_LENGTH[1] * sin(Euler[1]*pi/180)
#         # if abs(Euler[2]) > 0.05:
#         #     for i in range(4):                      #考虑roll
#         #         if i == 0 or i == 1:
#         #             minitaur_pos[0][i][0] = minitaur_pos[0][i][0]
#         #             minitaur_pos[0][i][1] = minitaur_pos[0][i][1] - 0.5 * BODY_LENGTH[0] * sin(Euler[2]*pi/180)
#         #         else:
#         #             minitaur_pos[0][i][0] = minitaur_pos[0][i][0]
#         #             minitaur_pos[0][i][1] = minitaur_pos[0][i][1] + 0.5 * BODY_LENGTH[0] * sin(Euler[2]*pi/180)
#         #传送坐标
#         print minitaur_pos
#         self.MoveLegsP2P(target_pos = minitaur_pos, run_time = 0.1, frequency = 2)    #run_time和f需要根据实验修改
#         for i in range(3):                      #更新欧拉角
#             Euler_old[i] = Euler_new[i]
