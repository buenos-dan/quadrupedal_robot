#!/usr/bin/env python
# coding:utf-8
import rospy
import numpy as np
import time
from math import *
from settings import *
from collections import deque
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
        self.leg_name = name
        self.actual_pos = [0,0]  # [x,y]
        self.actual_cur = [0,0]  # [I_M0,I_M1]
        self.control_pos = [0,0]  # [x,y]
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
        msg = LegControlMsg()
        msg.control_mode = 3
        msg.x = pos[0]
        msg.y = pos[1]
        self.control_puber.publish(msg)

    def WriteCurMsg(self,cur):
        msg = LegControlMsg()
        msg.control_mode = 1
        msg.x = cur[0]
        msg.y = cur[1]
        self.control_puber.publish(msg)

    def WriteVelMsg(self):
        pass
    def is_PosSecurity(self):
        '''
        test position whether in range
        '''
        pass

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
        self.time1 = 0.
        self.time2 = 0.
        self.run()




    def InitMinitaur(self):
        while rospy.get_param('~/imu_ready_flag') == 0:
            time.sleep(0.1)
        rospy.loginfo("[+]imu is ready")

        while rospy.get_param('~/Odrive_ready_flag') == 0:
            time.sleep(0.1)
        rospy.loginfo("[+]odrive is ready")

        for leg in self.LegControllers:
            leg.InitLeg()
            rospy.loginfo("[+]{}_leg is ready".format(leg.name))

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
            if command == 'test':
                self.MoveLegsP2P(DEFAULT_POS, 0.2)
                DEFAULT_POS1 = [[[-0.03, -0.27] for i in range(4)],
                                [[0.03, -0.27] for i in range(4)],
                                [[0.06, -0.3] for i in range(4)]]
                self.MoveLegsP2P(DEFAULT_POS1, 0.3)
            if command == 'jac1':
                print(self.Jacobian2(0,0))
                force = [[0.0], [-5.0]]
                force = np.mat(force)
                torque = self.Jacobian2(0./180 * pi, -48./180*pi).T * force
                print ('angle is -45., -45.')
                print ('force is:')
                print (force)
                print ('torque is:')
                print (torque)
                print ('current is:')
                print (self.TorqueToCur(torque))
                # torque2 = self.Jacobian2Foot(-43./180*pi, -48./180*pi).T * force
                # print 'foot torque is'
                # print torque2
                # print 'foot current is'
                # print self.TorqueToCur(torque2)
            if command == 'jac2':
                torque = [[1.0], [1.0]]
                torque = np.mat(torque)
                force = self.Jacobian2(0., 0.).T.I * torque
                print 'jacobian ankle force is '
                print force
                force2 = self.Jacobian2Foot(0., 0.).T.I * torque
                print 'jacobian foot force is '
                print force2
            if command == 'time':
                self.time1 = time.time()
                for i in range(10000):
                    torque = [[1.0], [1.0]]
                    torque = np.mat(torque)
                    force = self.Jacobian2(0., 0.).T.I * torque
                self.time2 = time.time()
                print(self.time2-self.time1)

            if command == 'oput':

                self.TorqueCal(0, -25)
            time.sleep(0.1)

    def Jacobian2(self, alpha, beta):
        """
        2*2速度雅克比和力雅克比矩阵,计算结果为脚踝处
        alpha,beta：电机转角
        :return: 雅可比矩阵，通过雅可比矩阵可以得到电机施加力矩和足端施加的力的关系
        """
        l1 = LEG_LENGTH[0]
        l2 = LEG_LENGTH[1]
        theta1 = (beta + alpha) / 2
        theta2 = (alpha - beta) / 2
        Q = sqrt(pow(l2, 2)-pow(l1*cos(theta1), 2))
        A = 1.0/2*sin(theta2)*(l1*cos(theta1)+pow(l1, 2)*cos(theta1)*sin(theta1)/Q)
        B = 1.0/2*cos(theta2)*(l1*sin(theta1)+Q)
        C = -1.0/2*cos(theta2)*(l1*cos(theta1)+pow(l1, 2)*cos(theta1)*sin(theta1)/Q)
        D = 1.0/2*sin(theta2)*(l1*sin(theta1)+Q)
        jaco = [[0 for i in range(2)] for j in range(2)]
        jaco[0][0] = A-B
        jaco[0][1] = A+B
        jaco[1][0] = C+D
        jaco[1][1] = C-D
        jaco = np.mat(jaco)
        # jaco_T = jaco.T
        # jaco_I = jaco_T.I
        #
        # mat_torque = [[0 for i in range(1)] for j in range(2)]
        # mat_torque[0][0] = t1
        # mat_torque[1][0] = t2
        # mat_force = jaco_I * mat_torque

        #  print(A,B,C,D,Q)
        #  print(jaco)
        #  print(mat_torque)
        return jaco
        #  return mat_force

    def Jacobian2Foot(self, alpha, beta):
        """
        2*2速度雅克比和力雅克比矩阵,计算结果为足端
        alpha,beta：电机转角
        :return: 雅可比矩阵，通过雅可比矩阵可以得到电机施加力矩和足端施加的力的关系
        """
        dl2 = DELTA_L2
        l1 = LEG_LENGTH[0]
        l2 = LEG_LENGTH[1]
        theta1 = -(alpha+beta)/2
        theta2 = (beta-alpha)/2
        Q = sqrt(pow(l2, 2)-pow(l1*cos(theta1), 2))
        A = 1.0/2 * (sin(theta2) * (l1 * cos(theta1) + pow(l1, 2) * cos(theta1) * sin(theta1)/Q) + dl2 * cos(theta2 + asin(l1/l2*cos(theta1))) * (-(l1/l2 * sin(theta1))/sqrt(1-pow((l1/l2*cos(theta1)), 2))))
        B = 1.0/2 * (cos(theta2)*(l1*sin(theta1)+Q) + dl2 * cos(theta2 + asin(l1/l2*cos(theta1))))
        C = -1.0/2*(cos(theta2)*(l1*cos(theta1)+pow(l1, 2)*cos(theta1)*sin(theta1)/Q)+dl2 * sin(theta2 + asin(l1/l2*cos(theta1))) * (-(l1/l2 * sin(theta1))/sqrt(1-pow((l1/l2*cos(theta1)), 2))))
        D = 1.0/2 * (sin(theta2)*(l1*sin(theta1)+Q) + dl2 * sin(theta2 + asin(l1/l2*cos(theta1))))
        jaco = [[0 for i in range(2)] for j in range(2)]
        jaco[0][0] = A-B
        jaco[0][1] = A+B
        jaco[1][0] = C+D
        jaco[1][1] = C-D
        jaco = np.mat(jaco)
        jaco_T = jaco.T
        jaco_I = jaco_T.I

        mat_torque = [[0 for i in range(1)] for j in range(2)]
        # mat_torque[0][0] = t1
        # mat_torque[1][0] = t2
        mat_force = jaco_I * mat_torque

        #  print(A,B,C,D,Q)
        #  print(jaco)
        #  print(mat_torque)
        return jaco
        #  return mat_force
    def Jacobian3(self, alpha, beta):
        """
        3*3雅可比矩阵
        :param alpha,beta:电机转角
        :return: 3*3雅可比矩阵
        """
        l1 = LEG_LENGTH[0]
        l2 = LEG_LENGTH[1]
        dl2 = DELTA_L2
        theta1 = (alpha+beta)/2
        theta2 = (beta-alpha)/2
        Q = sqrt(pow(l2, 2)-pow(l1*cos(theta1), 2))
        A = 1.0/2 * (sin(theta2) * (l1 * cos(theta1) + pow(l1, 2) * cos(theta1) * sin(theta1)/Q) + dl2 * cos(theta2 + asin(l1/l2*cos(theta1))) * (-(l1/l2 * sin(theta1))/sqrt(1-pow((l1/l2*cos(theta1)), 2))))
        B = 1.0/2 * (cos(theta2)*(l1*sin(theta1)+Q) + dl2 * cos(theta2 + asin(l1/l2*cos(theta1))))
        C = -1.0/2*(cos(theta2)*(l1*cos(theta1)+pow(l1, 2)*cos(theta1)*sin(theta1)/Q)+dl2 * sin(theta2 + asin(l1/l2*cos(theta1))) * (-(l1/l2 * sin(theta1))/sqrt(1-pow((l1/l2*cos(theta1)), 2))))
        D = 1.0/2 * (sin(theta2)*(l1*sin(theta1)+Q) + dl2 * sin(theta2 + asin(l1/l2*cos(theta1))))
        jaco = [[0 for i in range(3)] for j in range(3)]
        jaco[0][0] = A-B
        jaco[0][1] = 0.
        jaco[0][2] = A+B
        jaco[1] = [0., 0., 0.]
        jaco[2][0] = C+D
        jaco[2][1] = 0.
        jaco[2][2] = C-D
        jaco = np.mat(jaco)
        jaco_T = jaco.T
        jaco_I = jaco_T.I

        # mat_torque = [[0 for i in range(1)] for j in range(2)]
        # # mat_torque[0][0] = t1
        # # mat_torque[1][0] = t2
        # mat_force = jaco_I * mat_torque

        #  print(A,B,C,D,Q)
        #  print(jaco)
        #  print(mat_torque)
        return jaco
        #  return mat_force

    def TorqueCal(self, force_x, force_y):
        """
        计算输出力矩
        :return:
        """
        if True:
            self.MoveLegsP2P(target_pos=[[[0, -0.30] for i in range(4)]])
            time.sleep(5)
            torque = [[0.], [0.]]
            torque = np.mat(torque)
            force = [[float(force_x)], [float(force_y)]]
            force = np.mat(force)
            jaco = self.Jacobian2(0., 0.)
            cur = [[0.], [0.]]
            cur = np.mat(cur)
            torque = jaco.T * force
            cur = self.TorqueToCur(torque)
            for leg in self.LegControllers:
                leg.WriteCurMsg(cur)
        else:
            minitaur_tor = [[0.] for i in range(6)]
            minitaur_tor = np.mat(minitaur_tor)
            minitaur_jacob = [[0. for i in range(6)] for j in range(6)]



    def mat_Q(self,act_pos_xf, act_pos_zf, act_pos_xb, act_pos_zb):
        xf = 0.23+act_pos_xf
        yf = 0.142
        zf = act_pos_zf

        xb = -0.23+act_pos_xb
        yb = -0.142
        zb = act_pos_zb

        mat_Q = [[0 for i in range(6)] for j in range(6)]
        mat_Q[0][0] = 1
        mat_Q[0][3] = 1
        mat_Q[1][2] = 1
        mat_Q[1][5] = 1
        mat_Q[2][1] = -zf
        mat_Q[2][2] = yf
        mat_Q[2][4] = -zb
        mat_Q[2][5] = yb
        mat_Q[3][0] = zf
        mat_Q[3][2] = -xf
        mat_Q[3][3] = zb
        mat_Q[3][5] = -xb
        mat_Q[4][0] = -yf
        mat_Q[4][1] = xf
        mat_Q[4][3] = -yb
        mat_Q[4][4] = xb
        mat_Q[5][1] = 1
        mat_Q[5][4] = -1
        mat_Q = np.mat(mat_Q)
        return mat_Q.I

    def CurToTorque(self, cur):
        """
        电流和力矩的转换关系（待测试）
        :param cur: 电流
        :return: 关节力矩
        """
        return 6.5/(5*13.3)*cur

    def TorqueToCur(self, torque):
        """
        力矩转换成电流
        :param torque:
        :return:
        """
        return (13.3*5)/6.5*torque

    def imu_callback(self,msg):
        self.Euler = [msg.yaw,
                      msg.pitch,
                      msg.roll]

    def task_callback(self,msg):
        self.tasks.append(msg.data)


if __name__ == '__main__':
    my_contorller = minitaur()
