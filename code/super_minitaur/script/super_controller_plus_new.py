#!/usr/bin/env python
# coding:utf-8
import rospy
import numpy as np
import time
import threading
from math import *
from collections import deque
from settings_new import *
from super_minitaur.msg import *
from std_msgs.msg import *
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt

class LegControl:
    def __init__(self,name):
        self.name = name + "_leg"
        self.control_puber = rospy.Publisher("Odrive/" + name + "/control",LegControlMsg,queue_size=10)
        self.para_puber = rospy.Publisher("Odrive/parameter",OdriveParamMsg,queue_size=10)
        self.cur_suber = rospy.Subscriber("Odrive/" + name + "/actual_current",FourLegCurrentMsg,self.cur_callback)
        self.pos_suber = rospy.Subscriber("Odrive/" + name + "/actual_pos",FourLegPosMsg,self.pos_callback)
        self.leg_name = name
        self.actual_pos = [0.,-0.25]  # [x,y] M
        self.actual_vel = [0,0]  # [x,y] M
        self.t = time.time()
        self.actual_cur = [0,0]  # [I_M0,I_M1] A
        self.actual_motor_angle = [0,0]  # [front, rear] rad
        self.cur_deque = deque(maxlen=3)
        self.pos_x_p = 0.
        self.pos_x_d = 0.
        self.pos_x_i = 0.

        
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
        alpha,beta = InverseForAll(pos[0],pos[1],LEG_LENGTH)
        self.control_puber.publish(LegControlMsg(3,0,alpha,beta))

#    def CloseLoopPos(self,pos):
#        """
#        闭环控制末端位置,基于电流模式
#        """
#        spin_rate = 0.01
#        pos_error_int = np.array(np.zeros(2))
#        while ：
#            pos_estimate = self.actual_pos
#            pos_error = np.array(pos) - np.array(pos_estimate)
#            pos_error_dt = pos_error / spin_rate
#            pos_error_int += pos_error
#            force_x = self.pos_x_p * pos_error[0] + self.pos_x_d * pos_error[0] + self.pos_y_i *


    def WriteCurMsg(self,cur):
        self.control_puber.publish(LegControlMsg(1,0,cur[0],cur[1]))

    def Set_para(self,pos_gain = ODRV_POS_GAIN,
                 vel_gain = ODRV_VEL_GAIN,
                 vel_integrator_gain = ODRV_VEL_INTEGRATOR_GAIN,
                 vel_limit = ODRV_VEL_LIMIT,
                 current_limit = ODRV_CURRENT_LIM):
        msg = OdriveParamMsg()
        msg.odrive_index = self.leg_name
        msg.pos_gain = pos_gain
        msg.vel_gain = vel_gain
        msg.vel_integrator_gain = vel_integrator_gain
        msg.vel_limit = vel_limit
        msg.current_limit = current_limit
        self.para_puber.publish(msg)

    def cur_callback(self,msg):
        self.actual_cur = [abs(msg.front_current),abs(msg.rear_current)]

    def pos_callback(self,msg):
        self.actual_motor_angle = [msg.alpha,msg.beta]
        # 正运动学
        alpha = -msg.alpha
        beta = -msg.beta
        theta1 = (alpha+beta)/2
        theta2 = (alpha-beta)/2
        l1 = LEG_LENGTH[0]
        l2 = LEG_LENGTH[1]
        L = -l1*sin(theta1)+sqrt(pow(l2,2)-pow(l1,2)*pow(cos(theta1),2))
        x = L * sin(theta2)
        y = - L * cos(theta2)
        theta3 = theta2 + asin(l1 * cos(theta1) / l2)
        # theta3 = theta2 - asin(l1 * cos(theta1) / l2)
        x = x + DELTA_L2 * sin(theta3)
        y = y - DELTA_L2 * cos(theta3)

        vel = self.actual_vel
        self.actual_vel =(np.array([x,y])-self.actual_pos)/(time.time()-self.t)
        self.actual_acc =(self.actual_vel-vel)/(time.time()-self.t)
        self.actual_pos = [x,y]
        self.t = time.time()

    def leg_length(self):
        return sqrt(pow(self.actual_pos[0], 2) + pow(self.actual_pos[1], 2))

    def leg_squat(self,squat_pos=np.array([0,-0.27])):
        t = 0
        T = 0.1
        spin_rate = 0.05
        while t<T:
            xs = (-self.actual_pos[0] + squat_pos[0])*t/T+self.actual_pos[0]
            ys = (-self.actual_pos[1] + squat_pos[1])*t/T+self.actual_pos[1]
            self.WritePosMsg([xs,ys])
            t+=spin_rate
            # time.sleep(spin_rate)
    def leg_check(self,index,ADJUST_OFFSET=[-0.01,-0.01,-0.01,-0.01],ADJUST_HEIGHT=-0.25,step_height=0.03):
        if abs(self.actual_pos[0]-ADJUST_OFFSET[index])>0.02 or abs(self.actual_pos[1]-ADJUST_HEIGHT)>0.02:
            self.WritePosMsg([ADJUST_OFFSET[index],ADJUST_HEIGHT+step_height])
            time.sleep(0.02)
            self.WritePosMsg([ADJUST_OFFSET[index],ADJUST_HEIGHT])
            return 1
        return 0


def InverseForAll(x_f, y_f, LEG_LENGTH):  # x_f,y_f 为足端球心坐标
    C1 = pow(x_f,2) + pow(y_f,2) + pow(LEG_LENGTH[0],2) - pow((LEG_LENGTH[1]+DELTA_L2),2)  # 中间常量
    A = 1 + pow(x_f,2) / pow(y_f,2)
    B = -C1 * x_f / pow(y_f,2)
    C = pow(C1,2) / (4 * pow(y_f,2)) - pow(LEG_LENGTH[0],2)
    x_knee = (-B - sqrt(pow(B,2) - 4*A*C))/(2 * A)
    # x_knee = (-B + sqrt(pow(B,2) - 4*A*C))/(2 * A)
    y_knee = (C1 - 2*x_f*x_knee)/(2*y_f)
    scall = (DELTA_L2) / (LEG_LENGTH[1] + DELTA_L2)
    x = x_f + (x_knee-x_f)*scall
    y = y_f + (y_knee-y_f)*scall
    # 以下的xy为脚踝关节xy
    x = -x
    l1 = LEG_LENGTH[0]
    l2 = LEG_LENGTH[1]
    L = sqrt(x*x+y*y)
    psi = asin(x/L)
    phi = acos((l1*l1+L*L-l2*l2)/(2*l1*L))
    theta1 = phi - psi
    theta2 = phi + psi
    alpha = pi/2 - theta1
    beta = pi/2 - theta2
    return alpha, beta


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
        print("Waiting for Odrive")
        while rospy.get_param('Odrive_ready_flag') == 0:
            time.sleep(0.1)
        print("Odrive is Ready")

    def imu_callback(self,msg):
        self.Euler = [msg.yaw,
                      msg.pitch,
                      msg.roll]

    def state_callback(self,msg):
        self.state = msg.data

    DEFAULT_POS = [[[0.03,-0.25],
                    [-0.03,-0.25],
                    [0.03,-0.25],
                    [-0.03,-0.25]]]

    def MoveLegsP2P(self,target_pos= DEFAULT_POS ,run_time = 0.5,frequency = 40):
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
            if self.state == 'TRIAL1':
                self.Trial1()
            if self.state == 'TRIAL2':
                self.Trial2()
            if self.state == 'TRIAL3':
                self.Trial3()
            if self.state == 'TRIAL4':
                self.Trial4()
            if self.state == 'TROT':
                self.Trot()
            if self.state == 'TROT_IN_PLACE':
                self.Trot_in_place()
            if self.state == 'CRAWL':
                self.Crawl()
            if self.state == 'TURN':
                self.Turn()
            if self.state == 'JUMP':
                self.JumpCurrent()
            if self.state == 'TEST':
                t = time.time()
                for i in range(1000):
                    print(self.Euler)
                print(time.time()-t)
                self.state = 'IDLE'
            if self.state == 'ADJUST':
                self.Adjust()
            if self.state == 'UPSLOPE':
                self.Upslope()
            if self.state == 'ZERO':
                self.ZeroForceControl(self.LegControllers[3])
            time.sleep(0.1)

    def Adjust(self,offset=[0.03,-0.03,0.03,-0.03]):
        flag=[1,1,1,1]
        while sum(flag):
            for index, leg in enumerate(self.LegControllers):
                flag[index]=leg.leg_check(index,offset)
            time.sleep(0.1)
            print(flag)
        print('FINISH')
        self.state = 'IDLE'

    def PID_test(self):
        PID = [np.array([270, 240]), np.array([10, 8]), np.array([20, 20])]
        leg = self.LegControllers[3]
        posErrSum = 0.
        theo_acc = np.array([0, 0])
        theo_pos = leg.actual_pos + np.array([0.05, 0.])
        theo_vel = np.array([0.,0.])
        t = time.time()
        rate = 0.0025
        xs = []
        ys = []
        T=1
        old_time = time.time()
        while time.time()-t<T:
            xs.append(leg.actual_pos[0])
            ys.append(leg.actual_pos[1])
            posErrSum += (theo_pos-leg.actual_pos)*(time.time() - old_time)
            old_time = time.time()
            force = PID[0]*(theo_pos-leg.actual_pos)+PID[1]*(theo_vel-leg.actual_vel)+PID[2]*posErrSum+theo_acc
            self.WriteForce(leg, force)
            time.sleep(rate)
        print (theo_pos - leg.actual_pos)
        self.WriteForce(leg, [0, 0])
        ts = np.linspace(0, T, len(xs))
        plt.plot(ts, xs, 'b')
        # plt.plot(ts, ys, 'r')
        plt.show()
        self.state = "IDLE"

    def PIDControl(self,T,x,y,x_,y_):
        PID = [np.array([270, 240]), np.array([10, 8]), np.array([20, 20])]
        leg = self.LegControllers[3]
        posErrSum = 0.
        theo_acc = np.array([0, 0])
        theo_pos = np.array([x,y])
        theo_vel = np.array([x_, y_])
        t = time.time()
        rate = 0.0025
        old_time = time.time()
        while time.time()-t<T:
            posErrSum += (theo_pos-leg.actual_pos)*(time.time() - old_time)
            old_time = time.time()
            force = PID[0]*(theo_pos-leg.actual_pos)+PID[1]*(theo_vel-leg.actual_vel)+PID[2]*posErrSum+theo_acc
            self.WriteForce(leg, force)
            time.sleep(rate)

    def Circle_test(self):
        t=0
        T=2.5
        spin_rate=0.01
        a = 0.05
        leg=self.LegControllers[3]
        offset=leg.actual_pos

        xs = []
        ys = []
        xs_theo = []
        ys_theo = []
        while t<T:
            xs.append(leg.actual_pos[0])
            ys.append(leg.actual_pos[1])

            x = a*sin(2*pi*t/T)+offset[0]
            y = a*cos(2*pi*t/T)+offset[1]

            xs_theo.append(x)
            ys_theo.append(y)

            x_ = 0.75*a*2*pi/T*cos(2*pi*t/T)
            y_ = -0.75*a*2*pi/T*sin(2*pi*t/T)
            self.PIDControl(spin_rate,x,y,x_,y_)
            t = t+spin_rate
        self.LegControllers[3].WriteCurMsg([0, 0])
        self.state = 'IDLE'

        plt.plot(xs, ys, 'b')
        plt.plot(xs_theo, ys_theo, 'r')
        plt.show()

    def Squat(self):
        self.MoveLegsP2P(target_pos=[[[STAND_OFFSET[i], SQUAT_HEIGHT] for i in range(4)]],run_time=0.5,frequency=40)
        self.state = 'IDLE'

    def Trot(self):
        T = 0.5
        t1 = T/6.0
        t2 = T/6.0*5
        step_length = 0.1
        min_height = -0.24
        step_height_up = 0.04
        step_height_down = 0.02
        spin_rate = 0.01
        t = 0.0
        TROT_OFFSET = [0.03, -0.03, 0.03, -0.03]

        for index,leg in enumerate(self.LegControllers):
            leg.Set_para(pos_gain = 60, vel_gain = 0.001)
        self.MoveLegsP2P(target_pos=[[[TROT_OFFSET[0],min_height], [-step_length/2,min_height],
                                      [TROT_OFFSET[0],min_height], [-step_length/2,min_height]]],run_time=0.5,frequency=40)
        while not rospy.is_shutdown():
            t = t%T
            if t < t1:
                xs = t/t1*step_length-step_length/2
                ys = step_height_up*sin(t/t1*pi)+min_height
                self.LegControllers[0].WritePosMsg([xs+TROT_OFFSET[0], ys])
                self.LegControllers[3].WritePosMsg([xs+TROT_OFFSET[1], ys])
            elif (t - t1) < t2:
                xs = -(t-t1)/t2*step_length+step_length/2
                ys = step_height_down*-sin((t-t1)/t2*pi)+min_height
                self.LegControllers[0].WritePosMsg([xs+TROT_OFFSET[0], ys])
                self.LegControllers[3].WritePosMsg([xs+TROT_OFFSET[1], ys])
            t_other = t+T/2
            t_other=t_other%T
            if t_other < t1:
                xs = t_other/t1*step_length-step_length/2
                ys = step_height_up*sin(t_other/t1*pi)+min_height
                self.LegControllers[1].WritePosMsg([xs+TROT_OFFSET[1], ys])
                self.LegControllers[2].WritePosMsg([xs+TROT_OFFSET[0], ys])
            elif (t_other-t1) < t2:
                xs = -(t_other-t1)/t2*step_length+step_length/2
                ys = step_height_down*-sin((t_other-t1)/t2*pi)+min_height
                self.LegControllers[1].WritePosMsg([xs+TROT_OFFSET[1], ys])
                self.LegControllers[2].WritePosMsg([xs+TROT_OFFSET[0], ys])
            t += spin_rate
            time.sleep(spin_rate)
            if self.state != 'TROT':
                break

    def Upslope(self):
        T = 0.5
        t1 = T/6.0
        t2 = T/6.0*5
        step_length = 0.05
        min_height = -0.25
        step_height_up = 0.03
        step_height_down = 0.02
        spin_rate = 0.01
        t = 0.0
        TROT_OFFSET = [-0.0, -0.0, -0.0, -0.0]
        self.Adjust(TROT_OFFSET)
        self.state = 'UPSLOPE'
        while not rospy.is_shutdown():
            x_offset = 1.2*min_height*tan(self.Euler[1]/180*pi)
            print(t)
            print(x_offset)
            t = t%T
            if t < t1:
                xs = t/t1*step_length-step_length/2+x_offset
                ys = step_height_up*sin(t/t1*pi)+min_height
                self.LegControllers[0].WritePosMsg([xs+TROT_OFFSET[0], ys])
                self.LegControllers[3].WritePosMsg([xs+TROT_OFFSET[1], ys])
            elif (t - t1) < t2:
                xs = -(t-t1)/t2*step_length+step_length/2+x_offset
                ys = step_height_down*-sin((t-t1)/t2*pi)+min_height
                self.LegControllers[0].WritePosMsg([xs+TROT_OFFSET[0], ys])
                self.LegControllers[3].WritePosMsg([xs+TROT_OFFSET[1], ys])
            t_other = t+T/2
            t_other=t_other%T
            if t_other < t1:
                xs = t_other/t1*step_length-step_length/2+x_offset
                ys = step_height_up*sin(t_other/t1*pi)+min_height
                self.LegControllers[1].WritePosMsg([xs+TROT_OFFSET[1], ys])
                self.LegControllers[2].WritePosMsg([xs+TROT_OFFSET[0], ys])
            elif (t_other-t1) < t2:
                xs = -(t_other-t1)/t2*step_length+step_length/2+x_offset
                ys = step_height_down*-sin((t_other-t1)/t2*pi)+min_height
                self.LegControllers[1].WritePosMsg([xs+TROT_OFFSET[1], ys])
                self.LegControllers[2].WritePosMsg([xs+TROT_OFFSET[0], ys])
            t += spin_rate
            time.sleep(spin_rate)
            if self.state != 'UPSLOPE':
                break

    def Trot_in_place(self):
        T = 0.5
        t1 = T/6.0
        t2 = T/6.0*5

        min_height = -0.24
        step_height_up = 0.04
        step_height_down = 0.02
        spin_rate = 0.01
        t = 0.0
        TROT_OFFSET = [0.05, -0.05]

        for index,leg in enumerate(self.LegControllers):
            leg.Set_para(pos_gain = 60, vel_gain = 0.001)
        self.MoveLegsP2P(target_pos=[[[TROT_OFFSET[0],min_height], [TROT_OFFSET[1],min_height],
                                      [TROT_OFFSET[0],min_height], [TROT_OFFSET[1],min_height]]],run_time=0.5,frequency=40)
        while not rospy.is_shutdown():
            t = t%T
            if t < t1:
                xs = 0
                ys = step_height_up*sin(t/t1*pi)+min_height
                self.LegControllers[0].WritePosMsg([xs+TROT_OFFSET[0], ys])
                self.LegControllers[3].WritePosMsg([xs+TROT_OFFSET[1], ys])
            elif (t - t1) < t2:
                xs = 0
                ys = step_height_down*-sin((t-t1)/t2*pi)+min_height
                self.LegControllers[0].WritePosMsg([xs+TROT_OFFSET[0], ys])
                self.LegControllers[3].WritePosMsg([xs+TROT_OFFSET[1], ys])
            t_other = t+T/2
            t_other=t_other%T
            if t_other < t1:
                xs = 0
                ys = step_height_up*sin(t_other/t1*pi)+min_height
                self.LegControllers[1].WritePosMsg([xs+TROT_OFFSET[1], ys])
                self.LegControllers[2].WritePosMsg([xs+TROT_OFFSET[0], ys])
            elif (t_other-t1) < t2:
                xs = 0
                ys = step_height_down*-sin((t_other-t1)/t2*pi)+min_height
                self.LegControllers[1].WritePosMsg([xs+TROT_OFFSET[1], ys])
                self.LegControllers[2].WritePosMsg([xs+TROT_OFFSET[0], ys])
            t += spin_rate
            time.sleep(spin_rate)
            if self.state != 'TROT_IN_PLACE':
                break
    def Crawl(self):
        """
        爬行步态：一次迈出一条腿
        """
        T = 1.0
        t1 = T/4.0      # 腾空相时间
        t2 = T/4.0*3    # 触地相时间
        step_length = 0.15      # 步长
        min_height = -0.35      # y基准位置
        step_height_up = 0.13
        step_height_down = 0.02
        xs_A = 0.02    # x方向sin函数幅值
        spin_rate = 0.02
        t = 0.0
        CRAWL_OFFSET = [0.03, -0.03, 0.03, -0.03]

        self.MoveLegsP2P(target_pos = [[[-0.5*step_length, min_height], [CRAWL_OFFSET[1], min_height],
                                     [CRAWL_OFFSET[2], min_height], [CRAWL_OFFSET[3], min_height]]], run_time = 0.5, frequency = 40)
        while not rospy.is_shutdown():
            t = t % T
            xs_offset = sin(t/T*2.0*pi) * xs_A    # xs方向偏置
            if t < t1:      # 腿0腾空相
                xs = t/t1*step_length-step_length/2
                xs = xs + xs_offset
                ys = step_height_up*sin(t/t1*pi)+min_height
                self.LegControllers[0].WritePosMsg([xs+CRAWL_OFFSET[0], ys])
            elif (t - t1) < t2:     # 腿0触地相
                xs = -(t-t1)/t2*step_length+step_length/2
                xs = xs + xs_offset
                # print xs_offset
                ys = step_height_down*-sin((t-t1)/t2*pi)+min_height
                self.LegControllers[0].WritePosMsg([xs+CRAWL_OFFSET[0], ys])

            t_other1 = t + T/2     # 腿1
            t_other1 = t_other1 % T
            if t_other1 < t1:      # 腿1腾空相
                xs = t_other1/t1*step_length-step_length/2
                xs = xs + xs_offset
                ys = step_height_up*sin(t_other1/t1*pi)+min_height
                self.LegControllers[1].WritePosMsg([xs+CRAWL_OFFSET[1], ys])
            elif (t_other1-t1) < t2:    # 腿1触地相
                xs = -(t_other1-t1)/t2*step_length+step_length/2
                xs = xs + xs_offset
                # print xs_offset
                ys = step_height_down*-sin((t_other1-t1)/t2*pi)+min_height
                self.LegControllers[1].WritePosMsg([xs+CRAWL_OFFSET[1], ys])

            t_other2 = t + T/4 * 3     # 腿2
            t_other2 = t_other2 % T
            if t_other2 < t1:      # 腿2腾空相
                xs = t_other2/t1*step_length-step_length/2
                xs = xs + xs_offset
                ys = step_height_up*sin(t_other2/t1*pi)+min_height
                self.LegControllers[2].WritePosMsg([xs+CRAWL_OFFSET[2], ys])
            elif (t_other2-t1) < t2:    # 腿2触地相
                xs = -(t_other2-t1)/t2*step_length+step_length/2
                xs = xs + xs_offset
                # print xs_offset
                ys = step_height_down*-sin((t_other2-t1)/t2*pi)+min_height
                self.LegControllers[2].WritePosMsg([xs+CRAWL_OFFSET[2], ys])

            t_other3 = t + T/4     # 腿3
            t_other3 = t_other3 % T
            if t_other3 < t1:      # 腿3腾空相
                xs = t_other3/t1*step_length-step_length/2
                xs = xs + xs_offset
                ys = step_height_up*sin(t_other3/t1*pi)+min_height
                self.LegControllers[3].WritePosMsg([xs+CRAWL_OFFSET[3], ys])
            elif (t_other3-t1) < t2:    # 腿3触地相
                xs = -(t_other3-t1)/t2*step_length+step_length/2
                xs = xs + xs_offset
                # print xs_offset
                ys = step_height_down*-sin((t_other3-t1)/t2*pi)+min_height
                self.LegControllers[3].WritePosMsg([xs+CRAWL_OFFSET[3], ys])

            t += spin_rate
            time.sleep(spin_rate)
            if self.state != 'CRAWL':
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
        self.MoveLegsP2P(target_pos=[[[-step_length/2,min_height],
                                      [-(T/2-t1)/t2*step_length+step_length/2,step_height_down*-sin((T/2-t1)/t2*pi)+min_height],
                                      [-(-(T/2-t1)/t2*step_length+step_length/2),step_height_down*-sin((T/2-t1)/t2*pi)+min_height],
                                      [step_length/2,min_height]]],run_time=0.5,frequency=40)
        for index,leg in enumerate(self.LegControllers):
            leg.Set_para(150)
        while not rospy.is_shutdown():
            t = t%T
            if t <t1:
                xs = t/t1*step_length-step_length/2
                ys = step_height_up*sin(t/t1*pi)+min_height
                self.LegControllers[0].WritePosMsg([xs+0,ys])
                self.LegControllers[3].WritePosMsg([-xs+0,ys])
            elif (t-t1) < t2:
                xs = -(t-t1)/t2*step_length+step_length/2
                ys = step_height_down*-sin((t-t1)/t2*pi)+min_height
                self.LegControllers[0].WritePosMsg([xs+0,ys])
                self.LegControllers[3].WritePosMsg([-xs+0,ys])
            t_other = t+T/2
            t_other=t_other%T
            if t_other < t1:
                xs = t_other/t1*step_length-step_length/2
                ys = step_height_up*sin(t_other/t1*pi)+min_height
                self.LegControllers[1].WritePosMsg([xs+0,ys])
                self.LegControllers[2].WritePosMsg([-xs+0,ys])
            elif (t_other-t1) < t2:
                xs = -(t_other-t1)/t2*step_length+step_length/2
                ys = step_height_down*-sin((t_other-t1)/t2*pi)+min_height
                self.LegControllers[1].WritePosMsg([xs+0,ys])
                self.LegControllers[2].WritePosMsg([-xs+0,ys])
            t += spin_rate
            time.sleep(spin_rate)
            if self.state != 'TURN':
                break
        for index,leg in enumerate(self.LegControllers):
            leg.Set_para()

    def Trial1(self):
        x_delta = 0.1
        y_delta = 0.12
        # 站立
        STAND_OFFSET_ = [STAND_OFFSET[0]-0.04,STAND_OFFSET[1],STAND_OFFSET[2]-0.04,STAND_OFFSET[3]]
        self.MoveLegsP2P(target_pos=[[[STAND_OFFSET_[i],SQUAT_HEIGHT] for i in range(4)]], run_time=0.3, frequency=24)
        time.sleep(0.2)
        # 腾空
        for index,leg in enumerate(self.LegControllers):
            leg.WritePosMsg([STAND_OFFSET[index]-x_delta,SQUAT_HEIGHT-y_delta])
        time.sleep(0.1)
        # 收腿
        STAND_OFFSET_ = [STAND_OFFSET[0],STAND_OFFSET[1]-0.03,STAND_OFFSET[2],STAND_OFFSET[3]-0.03]
        self.MoveLegsP2P(target_pos=[[[STAND_OFFSET_[i],SQUAT_HEIGHT-y_delta/4] for i in range(4)]], run_time=0.2, frequency=20)
        # 判断触地
        while not rospy.is_shutdown():
            break_flag = 0
            for leg in self.LegControllers:
                if leg.is_touch(25):
                    rospy.logwarn('landing')
                    break_flag = 1
                    break
            if break_flag == 1:
                break
            rospy.logwarn('flying')
            time.sleep(0.01)
        # 落地站立
        self.MoveLegsP2P(target_pos=[[[STAND_OFFSET_[i],SQUAT_HEIGHT] for i in range(4)]], run_time=0.1, frequency=20)
        self.state = 'IDLE'

    def Trial2(self):
        x_delta = 0.1
        y_delta = 0.1
        # 站立
        STAND_OFFSET_ = [STAND_OFFSET[0]-0.06,STAND_OFFSET[1]-0.08,STAND_OFFSET[2]-0.06,STAND_OFFSET[3]-0.08]
        self.MoveLegsP2P(target_pos=[[[STAND_OFFSET_[i],SQUAT_HEIGHT] for i in range(4)]], run_time=0.3, frequency=24)
        time.sleep(0.8)
        # 腾空
        for index,leg in enumerate(self.LegControllers):
            if index ==1 or index == 3:
                leg.WritePosMsg([STAND_OFFSET[index]-x_delta,SQUAT_HEIGHT-y_delta-0.07])
            else:
                leg.WritePosMsg([STAND_OFFSET[index]-x_delta,SQUAT_HEIGHT-y_delta])
        time.sleep(0.1)
        # 收腿
        STAND_OFFSET_ = [STAND_OFFSET[0]+0.05,STAND_OFFSET[1]-0.02,STAND_OFFSET[2]+0.05,STAND_OFFSET[3]-0.02]
        self.MoveLegsP2P(target_pos=[[[STAND_OFFSET_[i],SQUAT_HEIGHT-y_delta/4] for i in range(4)]], run_time=0.2, frequency=20)
        # 判断触地
        while not rospy.is_shutdown():
            break_flag = 0
            for leg in self.LegControllers:
                if leg.is_touch(25):
                    rospy.logwarn('landing')
                    break_flag = 1
                    break
            if break_flag == 1:
                break
            rospy.logwarn('flying')
            time.sleep(0.01)
        # 落地站立
        self.MoveLegsP2P(target_pos=[[[STAND_OFFSET_[i],SQUAT_HEIGHT] for i in range(4)]], run_time=0.1, frequency=20)
        self.state = 'IDLE'

    def Trial3(self):
        x_delta = 0.1
        y_delta = 0.12
        # 站立
        STAND_OFFSET_ = [STAND_OFFSET[0]-0.03,STAND_OFFSET[1],STAND_OFFSET[2]-0.03,STAND_OFFSET[3]]
        self.MoveLegsP2P(target_pos=[[[STAND_OFFSET_[i],SQUAT_HEIGHT] for i in range(4)]], run_time=0.3, frequency=24)
        time.sleep(0.5)
        # 腾空
        for index,leg in enumerate(self.LegControllers):
            leg.WritePosMsg([STAND_OFFSET[index]-x_delta,SQUAT_HEIGHT-y_delta])
        time.sleep(0.15)
        # 收腿
        STAND_OFFSET_ = [STAND_OFFSET[0]+0.02,STAND_OFFSET[1],STAND_OFFSET[2]+0.02,STAND_OFFSET[3]]
        self.MoveLegsP2P(target_pos=[[[STAND_OFFSET_[i],SQUAT_HEIGHT-y_delta/4] for i in range(4)]], run_time=0.2, frequency=20)
        # 判断触地
        while not rospy.is_shutdown():
            break_flag = 0
            for leg in self.LegControllers:
                if leg.is_touch(25):
                    rospy.logwarn('landing')
                    break_flag = 1
                    break
            if break_flag == 1:
                break
            rospy.logwarn('flying')
            time.sleep(0.01)
        # 落地站立
        self.MoveLegsP2P(target_pos=[[[STAND_OFFSET[i],SQUAT_HEIGHT] for i in range(4)]], run_time=0.1, frequency=20)
        self.state = 'SQUAT'

    def Trial4(self):
        self.Adjust([0.03,0.03,0.03,0.03])
        time.sleep(0.5)
        x_delta = -0.1
        y_delta = 0.08
        # 腾空
        for index,leg in enumerate(self.LegControllers):
            leg.WritePosMsg([STAND_OFFSET[index]-x_delta,SQUAT_HEIGHT-y_delta])
        time.sleep(0.05)
        # 收腿
        self.MoveLegsP2P(target_pos=[[[STAND_OFFSET[i],SQUAT_HEIGHT] for i in range(4)]], run_time=0.1, frequency=20)
        self.state = 'IDLE'


    def JumpCurrent(self):
        """
        电流控制跳跃
        """
        # self.Adjust([-0.01,-0.02,-0.01,-0.02])#跳绳子
        self.Adjust([-0.01,-0.02,-0.01,-0.02])#跳沙丘
        # 计算电流
        # force = np.array([[-20., -35.],[-20,-35],[-20,-35],[-20,-35]])#跳绳子
        force = np.array([[-30.,-40.],[-30,-40],[-30,-40],[-30,-40]])#跳沙丘
        # 腿长度判断电流施加时间，一条腿过长，即停止施加电流
        leg_flag = [1, 1, 1, 1]
        leg_len = [0.36,0.36,0.36,0.36]
        time.sleep(1)
        while sum(leg_flag):
            for index, leg in enumerate(self.LegControllers):
                if leg.leg_length() > leg_len[index] and leg_flag[index] == 1:
                    leg.WriteCurMsg([0,0])
                    if index == 0 or index == 2:
                        leg.WritePosMsg([0.08,-0.23])
                    else:
                        leg.WritePosMsg([-0.05,-0.23])
                    leg_flag[index] = 0
                elif leg_flag[index] == 1:
                    if leg.leg_length() <0.3:
                        self.WriteForce(leg, 1.3*force[index])
                    else:
                        self.WriteForce(leg,force[index])
            time.sleep(0.01)

        # t = 0.
        # while t<0.15:
        #     for index, leg in enumerate(self.LegControllers):
        #         if leg.leg_length() <0.3:
        #             self.WriteForce(leg, 1.3*force[index])
        #         else:
        #             self.WriteForce(leg,force[index])
        #     t+=0.01
        #     time.sleep(0.01)
        #     print(t)
        #
        # self.LegControllers[3].WriteCurMsg([0,0])
        # self.LegControllers[1].WriteCurMsg([0,0])
        # self.LegControllers[0].WriteCurMsg([0,0])
        # self.LegControllers[2].WriteCurMsg([0,0])
        #
        # self.LegControllers[3].WritePosMsg([-0.03,-0.25])
        # self.LegControllers[1].WritePosMsg([-0.03,-0.25])
        # self.LegControllers[0].WritePosMsg([0.03,-0.25])
        # self.LegControllers[2].WritePosMsg([0.03,-0.25])

        print(leg_flag)
        self.state = 'IDLE'
        print('FINISH')

    def ForceDetect(self):
        """
        通过拉格朗日方程计算理论力矩，同时读取电机电流，得到电机电流和力矩的关系
        return:计算得到的力矩
        """
        g = 9.8015
        d = 0.066145
        theta = self.LegControllers[0].actual_motor_angle[0]
        theta_ = self.LegControllers[0].actual_motor_vel[0]
        theta__ = self.LegControllers[0].actual_motor_accel[0]
        current = self.LegControllers[0].actual_cur[0]
        # 电机外壳质量及直径
        m1 = 0.1476
        r1 = 0.088
        j1 = 1.89e-4
        # 大腿质量及长度
        m2 = 0.0458
        l2 = 0.161
        j2 = 1.158e-4
        # 拉格朗日方程结果
        torque = (j1 + j2) * theta__ + m2 * g * d * cos(theta)
        return torque

    def ForceControl(self, force=np.mat([[-0.], [-0.]])):
        """
        实现单腿力控制，LB腿
        :param force:施加的末端力，类型为矩阵类型
        """
        while 1:
            print self.LegControllers[3].actual_motor_angle
            jacob = self.Jacobian2(self.LegControllers[3].actual_motor_angle[0], self.LegControllers[3].actual_motor_angle[1])
            torque = jacob.T * force
            # print jacob
            current = self.TorqueToCur(torque)
            current = [float(current[0][0]), float(current[1][0])]
            self.LegControllers[3].WriteCurMsg(current)
            # print current
            time.sleep(0.05)

    def ZeroForceControl(self, LegController):
        while 1:
            torque = self.ForceToTorque(np.array([0., 0.]), LegController.actual_motor_angle[0], LegController.actual_motor_angle[1])
            current = self.TorqueToCur(torque)
            LegController.WriteCurMsg(current)
            time.sleep(0.01)
            if self.state == 'IDLE':
                break

    def WriteForce(self,LegController,force):
            torque = self.ForceToTorque(force, LegController.actual_motor_angle[0], LegController.actual_motor_angle[1])
            # print torque
            current = self.TorqueToCur(torque)
            print current
            LegController.WriteCurMsg(current)


    def ForceToTorque(self, force, alpha, beta):
        """
        将末端输出的力转换为电机需要施加的力矩
        :return: 电机所需要施加的torque
        """
        temp = alpha
        alpha = beta
        beta = temp
        m1 = 0.0458  # 0.0458
        m2 = 0.165  # 0.165
        lm1 = 0.065
        lm2 = 0.15
        m_all = 2 * (m1 + m2)
        torque = []
        l1 = LEG_LENGTH[0]
        l2 = LEG_LENGTH[1]
        dl2 = DELTA_L2
        theta1 = -(beta+alpha)/2.0
        theta2 = (alpha-beta)/2.0
        theta4 = asin(l1/l2*cos(theta1))
        theta3 = theta2 + theta4
        leg_length = -l1 * sin(theta1) + sqrt(pow(l2, 2) - pow((l1*cos(theta1)), 2))
        x1 = lm1 * cos(theta1 + pi/2)
        x3 = lm2 * cos(theta4)
        x2 = leg_length - x1 - x3
        xm = x1 + ((m2 + m1 + 0.05) / (m1 + m2)) * x2
        gy = m_all * g
        torque_g = gy * (leg_length - xm) * sin(theta2)
        gy_r = gy * cos(theta2)          # 重力旋转后在y方向分力
        gx_r = (gy * sin(theta2) - torque_g / leg_length)    # 重力旋转后在x方向分力
        torque_r = (force[0] * cos(theta3) + force[1] * sin(theta3)) * dl2 * 0.
        force_y_r = gy_r + force[1] * cos(theta2) - force[0] * sin(theta2)
        force_x_r = gx_r + force[1] * sin(theta2) + force[0] * cos(theta2) + torque_r / leg_length
        torque0 = -leg_length/2.0 * tan(theta4) * (force_y_r + force_x_r)
        torque.append(torque0)
        torque1 = leg_length/2.0 * tan(theta4) * (-force_y_r + force_x_r)
        torque.append(torque1)
        return torque

    def Jacobian2(self,alpha,beta):
        """
        2*2速度雅可比矩阵
        alpha,beta：电机转角
        :return: 雅可比矩阵，通过雅可比矩阵可以得到电机施加力矩和足端施加的力的关系
        """
        dl2 = DELTA_L2
        l1 = LEG_LENGTH[0]
        l2 = LEG_LENGTH[1]
        theta1 = -(beta+alpha)/2.0
        theta2 = -(beta-alpha)/2.0
        Q = sqrt(pow(l2, 2)-pow(l1*cos(theta1), 2))
        # A = 1.0/2*sin(theta2)*(l1*cos(theta1)+pow(l1, 2)*cos(theta1)*sin(theta1)/Q)
        # B = 1.0/2*cos(theta2)*(l1*sin(theta1)+Q)
        # C = -1.0/2*cos(theta2)*(l1*cos(theta1)+pow(l1, 2)*cos(theta1)*sin(theta1)/Q)
        # D = 1.0/2*sin(theta2)*(l1*sin(theta1)+Q)
        # 足端雅可比
        A = 1.0/2 * (sin(theta2) * (l1 * cos(theta1) + pow(l1, 2) * cos(theta1) * sin(theta1)/Q) + dl2 * cos(theta2 + asin(l1/l2*cos(theta1))) * (-(l1/l2 * sin(theta1))/sqrt(1-pow((l1/l2*cos(theta1)), 2))))
        B = 1.0/2 * (cos(theta2)*(l1*sin(theta1)+Q) + dl2 * cos(theta2 + asin(l1/l2*cos(theta1))))
        C = -1.0/2*(cos(theta2)*(l1*cos(theta1)+pow(l1, 2)*cos(theta1)*sin(theta1)/Q)+dl2 * sin(theta2 + asin(l1/l2*cos(theta1))) * (-(l1/l2 * sin(theta1))/sqrt(1-pow((l1/l2*cos(theta1)), 2))))
        D = 1.0/2 * (-sin(theta2)*(l1*sin(theta1)+Q) + dl2 * sin(theta2 + asin(l1/l2*cos(theta1))))
        jaco = [[0 for i in range(2)] for j in range(2)]
        jaco[0][0] = A-B
        jaco[0][1] = A+B
        jaco[1][0] = C-D
        jaco[1][1] = C+D
        jaco = np.mat(jaco)
        return jaco

    # def x_test(self):
    #     t = 0.
    #     T = 1.0
    #     num = 0
    #     spin_rate = 0.02
    #     v_s = []
    #     motor_angle_old = [0., 0.]
    #     flg = 0
    #     self.MoveLegsP2P(target_pos=[[[0,-25] for i in range(4)]],run_time=0.5,frequency=40)
    #     while num < 4:
    #         x_pos = 0.07 * sin(t / T * 2 * pi)
    #         self.LegControllers[3].WritePosMsg([x_pos, -0.25])
    #         motor_angle = self.LegControllers[3].actual_motor_angle
    #         if flg == 0:
    #             motor_angle_old = self.LegControllers[3].actual_motor_angle
    #             flg = 1
    #             continue
    #         motor_vel = (np.array([[motor_angle[0]],[motor_angle[1]]]) - np.array([
    #                                                                              [motor_angle_old[0]],
    #                                                                              [motor_angle_old[1]]
    #                                                                              ])) / spin_rate
    #         motor_angle_old = motor_angle
    #         jacob = self.Jacobian2(self.LegControllers[3].actual_motor_angle[0], self.LegControllers[3].actual_motor_angle[1])
    #         foot_vel = jacob * np.mat(motor_vel)
    #         foot_vel = np.array(foot_vel)
    #         v_s.append(foot_vel[0][0])
    #         t = t + spin_rate
    #         time.sleep(spin_rate)
    #         if t >= T:
    #             num += 1
    #         t = t % T
    #     x_s = np.linspace(0, (num+1) * T, len(v_s))
    #     plt.figure()
    #     plt.plot(x_s, v_s)
    #     plt.show()

    def TorqueToCur(self, torque):
        """
        力矩转换成电流
        :param torque:
        :return:
        """
        return np.array(torque) / KI


if __name__ == '__main__':
    my_contorller = minitaur()
