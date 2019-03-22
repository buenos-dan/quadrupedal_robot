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
        rospy.logwarn("{}_leg current:front:{} rear:{}\n\t"
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
        self.imu_suber = rospy.Subscriber("super_minitaur/imu_data",ImuMsg,self.imu_callback)
        self.state_suber = rospy.Subscriber("super_minitaur/state_change",String,self.state_callback)
        self.Euler = [0,0,0]
        self.Init()
        self.state = 'IDLE'
        self.run()

    def Init(self):
        rospy.logwarn("Waiting for Odrive")
        while rospy.get_param('Odrive_ready_flag') == 0:
            time.sleep(0.1)
        rospy.logwarn("Odrive is Ready")

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
        while not rospy.is_shutdown():
            if self.state == 'SQUAT':
                self.Squat()
            if self.state == 'JUMP_SQUAT':
                self.JumpSquat()
            if self.state == 'TROT':
                self.Trot()
            if self.state == 'TURN':
                self.Turn()
            if self.state == 'TROT_UP_SLIP':
                self.TrotUpSlip()
            if self.state == 'TROT_DOWN_SLIP':
                self.TrotDownSlip()
            if self.state == 'JUMP':
                self.Jump()
            if self.state == 'POS_JUMP':
                self.PosJump()
            if self.state == 'POS_JUMP_UP':
                self.PosJumpUp()
            if self.state == 'BOUND_UP_SLIP':
                self.BoundUpSlip()
            if self.state == 'BOUND_DOWN_SLIP':
                self.BoundDownSlip()
            if self.state == 'JUMP_TURN':
                self.JumpTurn()
            time.sleep(0.1)

    def Squat(self):
        self.MoveLegsP2P(target_pos=[[[STAND_OFFSET[i], SQUAT_HEIGHT] for i in range(4)]],run_time=0.5,frequency=40)
        self.state = 'IDLE'

    def JumpSquat(self):
        self.MoveLegsP2P(target_pos=[[[JUMP_OFFSET[i], SQUAT_HEIGHT] for i in range(4)]],run_time=0.5,frequency=40)
        self.state = 'IDLE'

    def Trot(self):
        T = 0.5
        t1 = T/6.0
        t2 = T/6.0*5
        step_length = 0.15
        min_height = -0.27
        step_height_up = 0.04
        step_height_down = 0.02
        spin_rate = 0.02
        t = 0.0
        for index,leg in enumerate(self.LegControllers):
            leg.Set_para(60,0.001)
        self.MoveLegsP2P(target_pos=[[[TROT_OFFSET[0],min_height], [-step_length/2,min_height],
                                      [TROT_OFFSET[0],min_height], [-step_length/2,min_height]]],run_time=0.5,frequency=40)
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
            if self.state != 'TROT':
                break

    def Turn(self):
        T = 0.5
        t1 = T/6.0
        t2 = T/6.0 * 5
        step_length = 0.1
        min_height = -0.26
        step_height_up = 0.00
        step_height_down = 0.02
        spin_rate = 0.02
        t = 0.0
        turn_k = 1.00
        self.MoveLegsP2P(target_pos=[[[TROT_OFFSET[0],min_height], [-step_length/2,min_height],
                                      [TROT_OFFSET[0],min_height], [-step_length/2,min_height]]],run_time=0.5,frequency=40)

        for index,leg in enumerate(self.LegControllers):
            leg.Set_para(150)

        while not rospy.is_shutdown():
            t = t%T
            if t <t1:
                xs = t/t1*step_length-step_length/2
                ys = step_height_up*sin(t/t1*pi)+min_height
                self.LegControllers[0].WritePosMsg([xs+TROT_OFFSET[0],ys])
                self.LegControllers[3].WritePosMsg([-xs+TROT_OFFSET[1],ys*turn_k])
            elif (t-t1) < t2:
                xs = -(t-t1)/t2*step_length+step_length/2
                ys = step_height_down*-sin((t-t1)/t2*pi)+min_height
                self.LegControllers[0].WritePosMsg([xs+TROT_OFFSET[0],ys])
                self.LegControllers[3].WritePosMsg([-xs+TROT_OFFSET[1],ys*turn_k])
            t_other = t+T/2
            t_other=t_other%T
            if t_other < t1:
                xs = t_other/t1*step_length-step_length/2
                ys = step_height_up*sin(t_other/t1*pi)+min_height
                self.LegControllers[1].WritePosMsg([xs+TROT_OFFSET[1],ys])
                self.LegControllers[2].WritePosMsg([-xs+TROT_OFFSET[0],ys*turn_k])
            elif (t_other-t1) < t2:
                xs = -(t_other-t1)/t2*step_length+step_length/2
                ys = step_height_down*-sin((t_other-t1)/t2*pi)+min_height
                self.LegControllers[1].WritePosMsg([xs+TROT_OFFSET[1],ys])
                self.LegControllers[2].WritePosMsg([-xs+TROT_OFFSET[0],ys*turn_k])
            t += spin_rate
            time.sleep(spin_rate)
            if self.state != 'TURN':
                break

    def TrotUpSlip(self):
        T = 0.5
        t1 = T/6.0
        t2 = T/6.0*5
        step_length = 0.15
        min_height = -0.27
        min_height_ = -0.35
        step_height_up = 0.05
        step_height_down = 0.02
        spin_rate = 0.02
        t = 0.0
        self.MoveLegsP2P(target_pos=[[[TROT_OFFSET[0],min_height], [-step_length/2,min_height_],
                                      [TROT_OFFSET[0],min_height], [-step_length/2,min_height_]]],run_time=0.5,frequency=40)
        while not rospy.is_shutdown():
            t = t%T
            if t <t1:
                xs = t/t1*step_length-step_length/2
                ys = step_height_up*sin(t/t1*pi)+min_height
                ys_ = step_height_up*sin(t/t1*pi)+min_height_
                self.LegControllers[0].WritePosMsg([xs+TROT_OFFSET[0],ys])
                self.LegControllers[3].WritePosMsg([xs+TROT_OFFSET[1],ys_])
            elif (t-t1) < t2:
                xs = -(t-t1)/t2*step_length+step_length/2
                ys = step_height_down*-sin((t-t1)/t2*pi)+min_height
                ys_ = step_height_down*-sin((t-t1)/t2*pi)+min_height_
                self.LegControllers[0].WritePosMsg([xs+TROT_OFFSET[0],ys])
                self.LegControllers[3].WritePosMsg([xs+TROT_OFFSET[1],ys_])
            t_other = t+T/2
            t_other=t_other%T
            if t_other < t1:
                xs = t_other/t1*step_length-step_length/2
                ys = step_height_up*sin(t_other/t1*pi)+min_height
                ys_ = step_height_up*sin(t_other/t1*pi)+min_height_
                self.LegControllers[1].WritePosMsg([xs+TROT_OFFSET[1],ys_])
                self.LegControllers[2].WritePosMsg([xs+TROT_OFFSET[0],ys])
            elif (t_other-t1) < t2:
                xs = -(t_other-t1)/t2*step_length+step_length/2
                ys = step_height_down*-sin((t_other-t1)/t2*pi)+min_height
                ys_ = step_height_down*-sin((t_other-t1)/t2*pi)+min_height_
                self.LegControllers[1].WritePosMsg([xs+TROT_OFFSET[1],ys_])
                self.LegControllers[2].WritePosMsg([xs+TROT_OFFSET[0],ys])
            t += spin_rate
            time.sleep(spin_rate)
            if self.state != 'TROT_UP_SLIP':
                break

    def TrotDownSlip(self):
        T = 0.5
        t1 = T/6.0
        t2 = T/6.0*5
        step_length = 0.08
        min_height = -0.35
        min_height_ = -0.27
        step_height_up = 0.05
        step_height_down = 0.02
        spin_rate = 0.02
        t = 0.0
        self.MoveLegsP2P(target_pos=[[[TROT_OFFSET[0],min_height], [-step_length/2,min_height_],
                                      [TROT_OFFSET[0],min_height], [-step_length/2,min_height_]]],run_time=0.5,frequency=40)
        while not rospy.is_shutdown():
            t = t%T
            if t <t1:
                xs = t/t1*step_length-step_length/2
                ys = step_height_up*sin(t/t1*pi)+min_height
                ys_ = step_height_up*sin(t/t1*pi)+min_height_
                self.LegControllers[0].WritePosMsg([xs+TROT_OFFSET[0],ys])
                self.LegControllers[3].WritePosMsg([xs+TROT_OFFSET[1],ys_])
            elif (t-t1) < t2:
                xs = -(t-t1)/t2*step_length+step_length/2
                ys = step_height_down*-sin((t-t1)/t2*pi)+min_height
                ys_ = step_height_down*-sin((t-t1)/t2*pi)+min_height_
                self.LegControllers[0].WritePosMsg([xs+TROT_OFFSET[0],ys])
                self.LegControllers[3].WritePosMsg([xs+TROT_OFFSET[1],ys_])
            t_other = t+T/2
            t_other=t_other%T
            if t_other < t1:
                xs = t_other/t1*step_length-step_length/2
                ys = step_height_up*sin(t_other/t1*pi)+min_height
                ys_ = step_height_up*sin(t_other/t1*pi)+min_height_
                self.LegControllers[1].WritePosMsg([xs+TROT_OFFSET[1],ys_])
                self.LegControllers[2].WritePosMsg([xs+TROT_OFFSET[0],ys])
            elif (t_other-t1) < t2:
                xs = -(t_other-t1)/t2*step_length+step_length/2
                ys = step_height_down*-sin((t_other-t1)/t2*pi)+min_height
                ys_ = step_height_down*-sin((t_other-t1)/t2*pi)+min_height_
                self.LegControllers[1].WritePosMsg([xs+TROT_OFFSET[1],ys_])
                self.LegControllers[2].WritePosMsg([xs+TROT_OFFSET[0],ys])
            t += spin_rate
            time.sleep(spin_rate)
            if self.state != 'TROT_DOWN_SLIP':
                break
    def stand_balance(self):
        """
        读取imu数据
        站立时保持平衡
        """
        Euler = [0., 0., 0.]
        Euler_new = [0., 0., 0.]
        Euler_old = [0., 0., 0.]
        minitaur_pos = [[[0., 0.] for i in range(4)]]
        for index,leg in enumerate(self.LegControllers):       #赋初值
            minitaur_pos[0][index] = leg.actual_pos
        while 1:
            for i in range(3):                      #更新欧拉角
                Euler_new[i] = self.Euler[i]
                Euler[i] = Euler_new[i] - Euler_old[i]
            print(1)
            if abs(Euler[1]) > 0.1:
                for i in range (4):#考虑pitch
                    if i == 0 or i == 2:
                        minitaur_pos[0][i][0] = minitaur_pos[0][i][0] - 0.5 * BODY_LENGTH[1] * (1 - cos(Euler[1]*pi/180))
                        minitaur_pos[0][i][1] = minitaur_pos[0][i][1] - 0.5 * BODY_LENGTH[1] * sin(Euler[1]*pi/180)
                    else:
                        minitaur_pos[0][i][0] = minitaur_pos[0][i][0] + 0.5 * BODY_LENGTH[1] * (1 - cos(Euler[1]*pi/180))
                        minitaur_pos[0][i][1] = minitaur_pos[0][i][1] + 0.5 * BODY_LENGTH[1] * sin(Euler[1]*pi/180)
            if abs(Euler[2]) > 0.05:
                for i in range(4): #考虑roll
                    if i == 0 or i == 1:
                        minitaur_pos[0][i][0] = minitaur_pos[0][i][0]
                        minitaur_pos[0][i][1] = minitaur_pos[0][i][1] - 0.5 * BODY_LENGTH[0] * sin(Euler[2]*pi/180)
                    else:
                        minitaur_pos[0][i][0] = minitaur_pos[0][i][0]
                        minitaur_pos[0][i][1] = minitaur_pos[0][i][1] + 0.5 * BODY_LENGTH[0] * sin(Euler[2]*pi/180)
            #传送坐标
            print(minitaur_pos)
            self.MoveLegsP2P(target_pos = minitaur_pos, run_time = 0.1, frequency = 2)    #run_time和f需要根据实验修改
            for i in range(3):                      #更新欧拉角
                Euler_old[i] = Euler_new[i]

    def BoundUpSlip(self):
        T = 0.5
        t1 = T/6.0
        t2 = T/6.0*5
        step_length = 0.10
        min_height = -0.28
        min_height_ = -0.38
        step_height_up = 0.05
        step_height_down = 0.02
        spin_rate = 0.02
        t = 0.0
        self.MoveLegsP2P(target_pos=[[[-step_length/2,min_height], [-step_length/2,min_height_],
                                      [-step_length/2,min_height], [-step_length/2,min_height_]]],run_time=0.5,frequency=40)
        while not rospy.is_shutdown():
            t = t%T
            if t <t1:
                xs = t/t1*step_length-step_length/2#-0.05~0.05
                ys = step_height_up*sin(t/t1*pi)+min_height#0.05*(0~1)-0.28
                self.LegControllers[0].WritePosMsg([xs+TROT_OFFSET[0],ys])
                self.LegControllers[2].WritePosMsg([xs+TROT_OFFSET[0],ys])
            elif (t-t1) < t2:
                xs = -(t-t1)/t2*step_length+step_length/2
                ys = step_height_down*-sin((t-t1)/t2*pi)+min_height#0.02*(-1~0)-0.28
                self.LegControllers[0].WritePosMsg([xs+TROT_OFFSET[0],ys])
                self.LegControllers[2].WritePosMsg([xs+TROT_OFFSET[0],ys])
            t_other = t+T/2
            t_other=t_other%T
            if t_other < t1:
                xs = t_other/t1*step_length-step_length/2
                ys = step_height_up*sin(t_other/t1*pi)+min_height_#0.05*(0~1)-0.35
                self.LegControllers[1].WritePosMsg([xs+TROT_OFFSET[1],ys])
                self.LegControllers[3].WritePosMsg([xs+TROT_OFFSET[1],ys])
            elif (t_other-t1) < t2:
                xs = -(t_other-t1)/t2*step_length+step_length/2
                ys = step_height_down*-sin((t_other-t1)/t2*pi)+min_height_#0.05*(0~1)-0.35
                self.LegControllers[1].WritePosMsg([xs+TROT_OFFSET[1],ys])
                self.LegControllers[3].WritePosMsg([xs+TROT_OFFSET[1],ys])
            t += spin_rate
            time.sleep(spin_rate)
            if self.Euler[1] < -15/180*pi:
                self.state = "SQUAT"
            if self.state != 'BOUND_UP_SLIP':
                break

    def BoundDownSlip(self):
        T = 0.5
        t1 = T/6.0
        t2 = T/6.0*5
        step_length = 0.10
        min_height = -0.35
        min_height_ = -0.28
        step_height_up = 0.05
        step_height_down = 0.02
        spin_rate = 0.02
        t = 0.0
        self.MoveLegsP2P(target_pos=[[[-step_length/2,min_height], [-step_length/2,min_height_],
                                      [-step_length/2,min_height], [-step_length/2,min_height_]]],run_time=0.5,frequency=40)
        while not rospy.is_shutdown():
            t = t%T
            if t <t1:
                xs = t/t1*step_length-step_length/2
                ys = step_height_up*sin(t/t1*pi)+min_height
                self.LegControllers[0].WritePosMsg([xs+TROT_OFFSET[0]+0.03,ys])
                self.LegControllers[2].WritePosMsg([xs+TROT_OFFSET[0]+0.03,ys])
            elif (t-t1) < t2:
                xs = -(t-t1)/t2*step_length+step_length/2
                ys = step_height_down*-sin((t-t1)/t2*pi)+min_height
                self.LegControllers[0].WritePosMsg([xs+TROT_OFFSET[0]+0.03,ys])
                self.LegControllers[2].WritePosMsg([xs+TROT_OFFSET[0]+0.03,ys])
            t_other = t+T/2
            t_other=t_other%T
            if t_other < t1:
                xs = t_other/t1*step_length-step_length/2
                ys = step_height_up*sin(t_other/t1*pi)+min_height_
                self.LegControllers[1].WritePosMsg([xs+TROT_OFFSET[1],ys])
                self.LegControllers[3].WritePosMsg([xs+TROT_OFFSET[1],ys])
            elif (t_other-t1) < t2:
                xs = -(t_other-t1)/t2*step_length+step_length/2
                ys = step_height_down*-sin((t_other-t1)/t2*pi)+min_height_
                self.LegControllers[1].WritePosMsg([xs+TROT_OFFSET[1],ys])
                self.LegControllers[3].WritePosMsg([xs+TROT_OFFSET[1],ys])
            t += spin_rate
            time.sleep(spin_rate)
            if self.state != 'BOUND_DOWN_SLIP':
                break

    def Jump_(self): # 19.1.24:极限电流设为25时，力量不足以起跳,极限7870电流为30时，左后腿odrive会掉电
        # 站立
        self.MoveLegsP2P(target_pos=[[[JUMP_OFFSET[i],SQUAT_HEIGHT] for i in range(4)]], run_time=0.3, frequency=24)
        time.sleep(0.5)
        # 腾空
        for index,leg in enumerate(self.LegControllers):
            leg.WriteCurMsg([34,34])
        time.sleep(0.15)
        # 收腿
        # self.MoveLegsP2P(target_pos=[[[0.03,SQUAT_HEIGHT-0.05] for i in range(4)]], run_time=0.2, frequency=20)
        # 判断触地
        # while 1:
        #     break_flag = 0
        #     for leg in self.LegControllers:
        #         if leg.is_touch(15):
        #             rospy.logwarn('landing')
        #             break_flag = 1
        #             break
        #     if break_flag == 1:
        #         break
        #     rospy.logwarn('flying')
        #     time.sleep(0.01)
        # 落地站立
        self.MoveLegsP2P(target_pos=[[[JUMP_OFFSET[0]+0.05,SQUAT_HEIGHT],[JUMP_OFFSET[1],SQUAT_HEIGHT],
                                      [JUMP_OFFSET[2]+0.05,SQUAT_HEIGHT],[JUMP_OFFSET[3],SQUAT_HEIGHT]]], run_time=0.1, frequency=20)
        self.state = 'SQUAT'

    def PosJump(self):
        # 站立
        self.MoveLegsP2P(target_pos=[[[STAND_OFFSET[i],SQUAT_HEIGHT] for i in range(4)]], run_time=0.3, frequency=24)
        time.sleep(0.5)
        # 腾空
        for index,leg in enumerate(self.LegControllers):
            if index == 0 or index == 2:
                leg.WritePosMsg([-0.00,-0.3])
            else:
                leg.WritePosMsg([-0.07,-0.3])
        time.sleep(0.1)
        # 收腿
        # self.MoveLegsP2P(target_pos=[[[0.03,SQUAT_HEIGHT-0.05] for i in range(4)]], run_time=0.2, frequency=20)
        # 判断触地
        # while 1:
        #     break_flag = 0
        #     for leg in self.LegControllers:
        #         if leg.is_touch(15):
        #             rospy.logwarn('landing')
        #             break_flag = 1
        #             break
        #     if break_flag == 1:
        #         break
        #     rospy.logwarn('flying')
        #     time.sleep(0.01)
        # 落地站立
        self.MoveLegsP2P(target_pos=[[[STAND_OFFSET[i],SQUAT_HEIGHT] for i in range(4)]], run_time=0.1, frequency=20)
        self.state = 'SQUAT'

    def PosJumpUp(self):
        # 站立
        self.MoveLegsP2P(target_pos=[[[STAND_OFFSET[0]-0.1,SQUAT_HEIGHT],[STAND_OFFSET[1],SQUAT_HEIGHT],
                                       [STAND_OFFSET[2]-0.1,SQUAT_HEIGHT],[STAND_OFFSET[3],SQUAT_HEIGHT]]], run_time=0.3, frequency=24)
        time.sleep(1.5)
        # 腾空
        for index,leg in enumerate(self.LegControllers):
            if index == 0 or index == 2:
                leg.WritePosMsg([-0.12,-0.28])
            else:
                leg.WritePosMsg([-0.08,-0.3])
        time.sleep(0.1)
        # 收腿
        # self.MoveLegsP2P(target_pos=[[[0.03,SQUAT_HEIGHT-0.05] for i in range(4)]], run_time=0.2, frequency=20)
        # 判断触地
        # while 1:
        #     break_flag = 0
        #     for leg in self.LegControllers:
        #         if leg.is_touch(15):
        #             rospy.logwarn('landing')
        #             break_flag = 1
        #             break
        #     if break_flag == 1:
        #         break
        #     rospy.logwarn('flying')
        #     time.sleep(0.01)
        # 落地站立
        self.MoveLegsP2P(target_pos=[[[STAND_OFFSET[0]-0.1,SQUAT_HEIGHT],[STAND_OFFSET[1]-0.05,SQUAT_HEIGHT],
                                      [STAND_OFFSET[2]-0.1,SQUAT_HEIGHT],[STAND_OFFSET[3]-0.05,SQUAT_HEIGHT]]], run_time=0.1, frequency=20)
        self.state = 'SQUAT'

    def JumpTurnCur(self,direction='LEFT'):
        direct_dict = {'LEFT':1,"RIGHT":-1}
        k = direct_dict[direction]
        turn_offset = 0.03
        # 站立
        self.MoveLegsP2P(target_pos=[[[-k*turn_offset,SQUAT_HEIGHT],[-k*turn_offset,SQUAT_HEIGHT],[k*turn_offset,SQUAT_HEIGHT],[k*turn_offset,SQUAT_HEIGHT]]], run_time=0.3, frequency=24)
        time.sleep(1)
        while not rospy.is_shutdown():
            # 腾空
            for index,leg in enumerate(self.LegControllers):
                leg.WriteCurMsg([30,30])
            time.sleep(0.1)
            # 收腿
            self.MoveLegsP2P(target_pos=[[[STAND_OFFSET[i],SQUAT_HEIGHT-0.05] for i in range(4)]], run_time=0.2, frequency=16)
            while 1:
                break_flag = 0
                for leg in self.LegControllers:
                    if leg.is_touch(15):
                        rospy.logwarn('landing')
                        break_flag = 1
                        break
                if break_flag == 1:
                    break
                rospy.logwarn('flying')
                time.sleep(0.001)
            # 落地站立
            self.MoveLegsP2P(target_pos=[[[STAND_OFFSET[i],SQUAT_HEIGHT] for i in range(4)]], run_time=0.2, frequency=32)

    def Jump(self): # 19.1.24:极限电流设为25时，力量不足以起跳,极限电流为30时，左后腿odrive会掉电
        delta_x_jumping = 0.05
        delta_y_jumping = 0.1
        squat_height = -0.225
        # 站立
        self.MoveLegsP2P(target_pos=[[[DEFAULT_OFFSET[i],squat_height] for i in range(4)]], run_time=0.3, frequency=24)
        time.sleep(1)
        # 腾空
        for index,leg in enumerate(self.LegControllers):
            leg.Set_para(100)
            leg.WritePosMsg([DEFAULT_OFFSET[index]-delta_x_jumping,squat_height-delta_y_jumping])
        time.sleep(0.1)
        # 收腿
        for leg in self.LegControllers:
            leg.Set_para(50)
        self.MoveLegsP2P(target_pos=[[[DEFAULT_OFFSET[i]-delta_x_jumping/2,squat_height-delta_y_jumping/2] for i in range(4)],
                                     [[DEFAULT_OFFSET[i]+delta_x_jumping/2,squat_height-delta_y_jumping/2] for i in range(4)],
                                     [[DEFAULT_OFFSET[i]+delta_x_jumping,squat_height-delta_y_jumping] for i in range(4)]], run_time = 0.4, frequency = 40)
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
        for leg in self.LegControllers:
            leg.Set_para()
        self.MoveLegsP2P(target_pos=[[[DEFAULT_OFFSET[i],squat_height] for i in range(4)]], run_time=0.8, frequency=60)

    def JumpTurn(self,direction='LEFT'):
        direct_dict = {'LEFT':1,"RIGHT":-1}
        k = direct_dict[direction]
        x_offset = 0.1
        delta_x_jumping = [k*-x_offset,k*-x_offset,k*x_offset,k*x_offset]
        delta_y_jumping = 0.06
        squat_height = -0.23
        # 站立
        self.MoveLegsP2P(target_pos=[[[DEFAULT_OFFSET[i],squat_height] for i in range(4)]], run_time=0.3, frequency=24)
        time.sleep(1)
        while not rospy.is_shutdown():
            # 腾空
            for index,leg in enumerate(self.LegControllers):
                leg.WritePosMsg((DEFAULT_OFFSET[index]+delta_x_jumping[index],squat_height - delta_y_jumping))
            time.sleep(0.1)
            # 收腿
            self.MoveLegsP2P(target_pos=[[[DEFAULT_OFFSET[i],squat_height - delta_y_jumping/2] for i in range(4)]], run_time=0.2, frequency=16)
            while not rospy.is_shutdown():
                break_flag = 0
                for leg in self.LegControllers:
                    if leg.is_touch(22):
                        rospy.loginfo('landing')
                        break_flag = 1
                        break
                if break_flag == 1:
                    break
                rospy.loginfo('flying')
                time.sleep(0.001)
            self.MoveLegsP2P(target_pos=[[[DEFAULT_OFFSET[i],squat_height] for i in range(4)]], run_time=0.4, frequency=32)

    def imu_callback(self,msg):
        self.Euler = [msg.yaw,
                      msg.pitch,
                      msg.roll]

    def state_callback(self,msg):
        self.state = msg.data


if __name__ == '__main__':
    my_contorller = minitaur()
