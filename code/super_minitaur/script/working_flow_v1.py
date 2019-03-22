#!/usr/bin/env python
# coding:utf-8
import rospy, time, re, serial
import numpy as np
from math import *
from collections import deque
from settings_new import *
from super_minitaur.msg import *
from super_minitaur.srv import *
from std_msgs.msg import *
from scipy.interpolate import interp1d

field = "blue"


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
        spin_rate = 0.01
        while t<T:
            xs = (-self.actual_pos[0] + squat_pos[0])*t/T+self.actual_pos[0]
            ys = (-self.actual_pos[1] + squat_pos[1])*t/T+self.actual_pos[1]
            self.WritePosMsg([xs,ys])
            t+=spin_rate
            # time.sleep(spin_rate)

    def leg_check(self,index,step_height=0.02):
        ADJUST_OFFSET = [-0.00,0.02,-0.00,0.02]
        ADJUST_HEIGHT = -0.23
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
        self.state_suber = rospy.Subscriber("super_minitaur/state_change",String,self.state_callback)
        self.vision_server = rospy.Service("/state_flag_vision",StateFlag,self.vision_flag_handler)
        self.lower_server = rospy.Service("/state_flag_lower",StateFlag,self.lower_flag_handler)
        self.imu_server = rospy.Service("/state_flag_imu",StateFlag,self.imu_flag_handler)
        self.vision_flag = [0,0,0,0,0,0]
        self.lower_flag = [0,0,0,0,0]
        self.imu_flag = [0]
        self.Init()
        self.state = 'WAITING'
        self.run()

    def Init(self):
        print("Waiting for Odrive")
        while rospy.get_param('Odrive_ready_flag') == 0:
            time.sleep(0.1)
        print("Odrive is Ready")

    def state_callback(self,msg):
        self.state = msg.data

    def vision_flag_handler(self,req):
        self.vision_flag = req.flag
        return 0

    def lower_flag_handler(self,req):
        self.lower_flag = req.flag
        return 0

    def imu_flag_handler(self,req):
        self.imu_flag = req.flag
        return 0

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
            if self.state == 'WAITING':
                self.Waiting()
            if self.state == 'TROT':
                self.Trot()
            if self.state[:4] == 'TURN':
                try:
                    dir = int(re.split(r"\)",re.split(r"\(",self.state)[1])[0])
                except:
                    dir = 1
                self.Turn(dir)
            if self.state == 'JUMP_1':
                self.Jump_1()
            if self.state == 'JUMP_2':
                self.Jump_2()
            if self.state == 'UPSLOPE':
                self.Upslope()
            time.sleep(0.1)

    def Waiting(self):
        self.MoveLegsP2P(target_pos=[[[STAND_OFFSET[i], SQUAT_HEIGHT] for i in range(4)]],run_time=0.5,frequency=40)
        while not rospy.is_shutdown():
            time.sleep(0.1)
            print("[{}]\tI'm waiting".format(time.time()))
            if self.lower_flag[0]:
                self.lower_flag[0] = 0
                self.state = "JUMP_1"
                return
            elif self.lower_flag[1]:
                self.lower_flag[1] = 0
                self.state = "JUMP_2"
                return
            elif self.lower_flag[2]:
                self.lower_flag[2] = 0
                self.state = "UPSLOPE"
                return
            elif self.lower_flag[3]:
                self.lower_flag[3] = 0
                self.state = "TROT"
                return
            elif self.state != "WAITING":
                return

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
            print("[{}]\tI'm troting".format(time.time()))
            if self.lower_flag[4]:
                self.lower_flag[4] = 0
                self.state = "WAITING"
                return
            elif self.vision_flag[0]:
                self.vision_flag[0] = 0
                self.state = "TURN("+str(field_cipher[field])+")"
                return
            elif self.vision_flag[3]:
                self.vision_flag[3] = 0
                self.state = "TURN(-"+str(field_cipher[field])+")"
                return
            elif self.state != "TROT":
                return

    def Turn(self,dir):
        # dir = 1左, dir = -1右
        T = 0.4
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
                self.LegControllers[0].WritePosMsg([dir*xs,ys])
                self.LegControllers[3].WritePosMsg([dir*-xs,ys])
            elif (t-t1) < t2:
                xs = -(t-t1)/t2*step_length+step_length/2
                ys = step_height_down*-sin((t-t1)/t2*pi)+min_height
                self.LegControllers[0].WritePosMsg([dir*xs,ys])
                self.LegControllers[3].WritePosMsg([dir*-xs,ys])
            t_other = t+T/2
            t_other=t_other%T
            if t_other < t1:
                xs = t_other/t1*step_length-step_length/2
                ys = step_height_up*sin(t_other/t1*pi)+min_height
                self.LegControllers[1].WritePosMsg([dir*xs,ys])
                self.LegControllers[2].WritePosMsg([dir*-xs,ys])
            elif (t_other-t1) < t2:
                xs = -(t_other-t1)/t2*step_length+step_length/2
                ys = step_height_down*-sin((t_other-t1)/t2*pi)+min_height
                self.LegControllers[1].WritePosMsg([dir*xs,ys])
                self.LegControllers[2].WritePosMsg([dir*-xs,ys])
            t += spin_rate
            time.sleep(spin_rate)
            print("[{}]\tI'm turning".format(time.time()))
            if self.lower_flag[4]:
                self.lower_flag[4] = 0
                self.state = "WAITING"
                break
            elif self.vision_flag[1]:
                self.vision_flag[1] = 0
                self.state = "MOVEFORWARDSLOWLY"
                break
            elif self.vision_flag[4]:
                self.vision_flag[4] = 0
                self.state = "MOVEFORWARDSLOWLY"
                break
            elif self.state[:4] != 'TURN':
                break
        for index,leg in enumerate(self.LegControllers):
            leg.Set_para()

    def Jump_1(self):
        """
        电流控制跳跃
        """
        # 计算电流
        force = np.array([[-35., -35.],[-30., -35.],[-35., -35.],[-35., -40.]])
        # 腿长度判断电流施加时间，一条腿过长，即停止施加电流
        leg_flag = [1, 1, 1, 1]
        leg_len = [0.36,0.37,0.36,0.38]
        print("[{}]\tI'm going to jump_1".format(time.time()))
        time.sleep(1)
        while sum(leg_flag):
            for index, leg in enumerate(self.LegControllers):
                if leg.leg_length() > leg_len[index] and leg_flag[index] == 1:
                    leg.WriteCurMsg([0,0])
                    if index == 0 or index == 2:
                        leg.leg_squat([0.06,-0.23])
                    else:
                        leg.leg_squat([-0.0,-0.23])
                    leg_flag[index] = 0
                elif leg_flag[index] == 1:
                    if leg.leg_length() <0.3:
                        self.WriteForce(leg, 1.3*force[index])
                    else:
                        self.WriteForce(leg,force[index])
                    time.sleep(0.01)
        print(leg_flag)
        self.state = 'TROT'

    def Jump_2(self):
        print("[{}]\tI'm going to jump_2".format(time.time()))
        self.state = 'WAITING'

    def Upslope(self):
        while not rospy.is_shutdown():
            time.sleep(0.1)
            print("[{}]\tI'm upsloping".format(time.time()))
            if self.lower_flag[4]:
                self.lower_flag[4] = 0
                self.state = "WAITING"
                break
            elif self.imu_flag[0]:
                self.imu_flag[0] = 0
                self.state = "WAITING"
                port = serial.Serial("/dev/stm32",115200,timeout=0.5)
                port.close()
                port.open()
                port.write("lift")
                break
            elif self.state != "UPSLOPE":
                break

    def MoveForwardSlowly(self):
        while not rospy.is_shutdown():
            time.sleep(0.1)
            print("[{}]\tI'm moving forward slowly".format(time.time()))
            if self.lower_flag[4]:
                self.lower_flag[4] = 0
                self.state = "WAITING"
                break
            if self.vision_flag[2]:
                self.vision_flag[2] = 0
                self.state = "JUMP_1"
                break
            elif self.vision_flag[5]:
                self.vision_flag[5] = 0
                self.state = "JUMP_2"
                break
            elif self.state != "MOVEFORWARDSLOWLY":
                break

    def WriteForce(self,LegController,force):
        torque = self.ForceToTorque(force, LegController.actual_motor_angle[0], LegController.actual_motor_angle[1])
        # print torque
        current = self.TorqueToCur(torque)
        print(current)
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

    def TorqueToCur(self, torque):
        """
        力矩转换成电流
        :param torque:
        :return:
        """
        return np.array(torque) / KI


if __name__ == '__main__':
    my_contorller = minitaur()
