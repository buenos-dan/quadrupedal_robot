#!/usr/bin/env python3
# coding:utf-8

import re,rospy,odrive,os,rospkg,threading,time,fibre
from settings_new import *
from odrive.enums import *
from super_minitaur.msg import *
import numpy as np
from math import *


class OdriveDriver:
    def __init__(self):
        rospy.set_param('Odrive_ready_flag',0)
        self.ros_init()
        self.odrive_init()
        self.loop_rate_s = CONTROL_RATE
        self.spin()

    def ros_init(self):
        rospy.init_node("Odrive_dirver",sys.argv,anonymous=True)
        self.control_para_suber = rospy.Subscriber('Odrive/parameter',OdriveParamMsg,self.control_parameter_callback)
        self.control_subbers = [0,0,0,0]
        self.control_callbacks = [self.RF_callback,self.RB_callback,self.LF_callback,self.LB_callback]

    def odrive_init(self):  # RB和LF的M0和M1是反的
        # 找odrive
        rp = rospkg.RosPack()
        root_path = rp.get_path('super_minitaur')
        os.system("lsusb > "+root_path+"/unused_script/lsusb.txt")
        dev_nums = find_dev_num(root_path+"/unused_script/lsusb.txt")
        rospy.loginfo("Waiting for Odrives, There are {} odrives".format(len(dev_nums)) if len(dev_nums) > 1 else
                      "Waiting for Odrives, There is only {} odrive".format(len(dev_nums)))
        self.odrive_arr_org = [odrive.find_any('usb:003:%.3d' % (dev_nums[i])) for i in range(len(dev_nums))]
        self.odrives = [0, 0, 0, 0]
        for i in range(len(self.odrive_arr_org)):
            if self.odrive_arr_org[i].serial_number == 35580456622152:
                self.odrives[0] = self.odrive_arr_org[i]
            elif self.odrive_arr_org[i].serial_number == 35644881197128:
                self.odrives[1] = self.odrive_arr_org[i]
            elif self.odrive_arr_org[i].serial_number == 35593342310472:
                self.odrives[2] = self.odrive_arr_org[i]
            elif self.odrive_arr_org[i].serial_number == 35644881131592:
                self.odrives[3] = self.odrive_arr_org[i]
        # 赋初值
        for index,leg in enumerate(["RF","RB","LF","FB"]):
            # 错误检查
            if self.odrives[index] == 0:
                rospy.logwarn("{}_leg not found !".format(leg))
                continue
            self.odrives[index].axis0.controller.config.control_mode = 3
            self.odrives[index].axis0.controller.config.pos_gain = ODRV_POS_GAIN
            self.odrives[index].axis0.controller.config.vel_gain = ODRV_VEL_GAIN
            self.odrives[index].axis0.controller.config.vel_integrator_gain = ODRV_VEL_INTEGRATOR_GAIN
            self.odrives[index].axis0.controller.config.vel_limit = ODRV_VEL_LIMIT
            self.odrives[index].axis0.motor.config.current_lim = ODRV_CURRENT_LIM
            self.odrives[index].axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            self.odrives[index].axis1.controller.config.control_mode = 3
            self.odrives[index].axis1.controller.config.pos_gain = ODRV_POS_GAIN
            self.odrives[index].axis1.controller.config.vel_gain = ODRV_VEL_GAIN
            self.odrives[index].axis1.controller.config.vel_integrator_gain = ODRV_VEL_INTEGRATOR_GAIN
            self.odrives[index].axis1.controller.config.vel_limit = ODRV_VEL_LIMIT
            self.odrives[index].axis1.motor.config.current_lim = ODRV_CURRENT_LIM
            self.odrives[index].axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        for i in range(4):
            if self.odrives[i] == 0:
                pass
            else:
                self.control_subbers[i] = rospy.Subscriber('Odrive/{}/control'.format(leg_order[i]),LegControlMsg,self.control_callbacks[i])
        if not self.odrives == [0,0,0,0]:
            rospy.set_param('Odrive_ready_flag',1)
            rospy.loginfo('OK!')
        else:
            rospy.logerr('There is less than one Odrive ! Exit !')
            exit()

    def control_parameter_callback(self,msg):
        if self.odrives[msg.odrive_index] == 0:
            return
        self.odrives[msg.odrive_index].axis0.controller.config.pos_gain = msg.pos_gain
        self.odrives[msg.odrive_index].axis0.controller.config.vel_gain = msg.vel_gain
        self.odrives[msg.odrive_index].axis0.controller.config.vel_integrator_gain = msg.vel_integrator_gain
        self.odrives[msg.odrive_index].axis0.controller.config.vel_limit = msg.vel_limit
        self.odrives[msg.odrive_index].axis0.motor.config.current_lim = msg.current_limit
        self.odrives[msg.odrive_index].axis1.controller.config.pos_gain = msg.pos_gain
        self.odrives[msg.odrive_index].axis1.controller.config.vel_gain = msg.vel_gain
        self.odrives[msg.odrive_index].axis1.controller.config.vel_integrator_gain = msg.vel_integrator_gain
        self.odrives[msg.odrive_index].axis1.controller.config.vel_limit = msg.vel_limit
        self.odrives[msg.odrive_index].axis1.motor.config.current_lim = msg.current_limit

    def RF_callback(self,msg):
        if self.odrives[0] == 0:
            return
        self.odrives[0].axis0.controller.config.control_mode = msg.control_mode
        self.odrives[0].axis1.controller.config.control_mode = msg.control_mode
        if msg.control_mode == 3:  # 位置模式
            alpha,beta = InverseForAll(msg.x,msg.y,LEG_LENGTH)
            self.odrives[0].axis0.controller.pos_setpoint = int(alpha/(2*pi) * 8192)+ODRIVE_OFFSET[0]
            self.odrives[0].axis1.controller.pos_setpoint = int(beta/(2*pi) * 8192)+ODRIVE_OFFSET[1]
        if msg.control_mode == 2:  # 速度模式
            self.odrives[0].axis0.controller.vel_setpoint = int(msg.x/(2*pi) * 8192)
            self.odrives[0].axis1.controller.vel_setpoint = int(msg.y/(2*pi) * 8192)
        if msg.control_mode == 1:  # 电流模式
            self.odrives[0].axis0.controller.current_setpoint = msg.x
            self.odrives[0].axis1.controller.current_setpoint = msg.y

    def RB_callback(self,msg):  # M0,M1反了
        if self.odrives[1] == 0:
            return
        self.odrives[1].axis0.controller.config.control_mode = msg.control_mode
        self.odrives[1].axis1.controller.config.control_mode = msg.control_mode
        if msg.control_mode == 3:  # 位置模式
            alpha,beta = InverseForAll(msg.x,msg.y,LEG_LENGTH)
            self.odrives[1].axis1.controller.pos_setpoint = int(alpha/(2*pi) * 8192)+ODRIVE_OFFSET[2]
            self.odrives[1].axis0.controller.pos_setpoint = int(beta/(2*pi) * 8192)+ODRIVE_OFFSET[3]
        if msg.control_mode == 2:  # 速度模式,xy代表前后角速度
            self.odrives[1].axis1.controller.vel_setpoint = int(msg.x/(2*pi) * 8192)
            self.odrives[1].axis0.controller.vel_setpoint = int(msg.y/(2*pi) * 8192)
        if msg.control_mode == 1:  # 电流模式
            self.odrives[1].axis1.controller.current_setpoint = msg.x
            self.odrives[1].axis0.controller.current_setpoint = msg.y

    def LF_callback(self,msg):  # M0,M1反了
        if self.odrives[2] == 0:
            return
        self.odrives[2].axis0.controller.config.control_mode = msg.control_mode
        self.odrives[2].axis1.controller.config.control_mode = msg.control_mode
        if msg.control_mode == 3:  # 位置模式
            alpha,beta = InverseForAll(msg.x,msg.y,LEG_LENGTH)
            self.odrives[2].axis1.controller.pos_setpoint = int(alpha/(2*pi) * 8192)+ODRIVE_OFFSET[4]
            self.odrives[2].axis0.controller.pos_setpoint = int(beta/(2*pi) * 8192)+ODRIVE_OFFSET[5]
        if msg.control_mode == 2:  # 速度模式
            self.odrives[2].axis1.controller.vel_setpoint = int(msg.x/(2*pi) * 8192)
            self.odrives[2].axis0.controller.vel_setpoint = int(msg.y/(2*pi) * 8192)
        if msg.control_mode == 1:  # 电流模式
            self.odrives[2].axis1.controller.current_setpoint = msg.x
            self.odrives[2].axis0.controller.current_setpoint = msg.y

    def LB_callback(self,msg):
        if self.odrives[3] == 0:
            return
        self.odrives[3].axis0.controller.config.control_mode = msg.control_mode
        self.odrives[3].axis1.controller.config.control_mode = msg.control_mode
        if msg.control_mode == 3:  # 位置模式
            alpha,beta = InverseForAll(msg.x,msg.y,LEG_LENGTH)
            self.odrives[3].axis0.controller.pos_setpoint = int(alpha/(2*pi) * 8192)+ODRIVE_OFFSET[6]
            self.odrives[3].axis1.controller.pos_setpoint = int(beta/(2*pi) * 8192)+ODRIVE_OFFSET[7]
        if msg.control_mode == 2:  # 速度模式
            self.odrives[3].axis0.controller.vel_setpoint = int(msg.x/(2*pi) * 8192)
            self.odrives[3].axis1.controller.vel_setpoint = int(msg.y/(2*pi) * 8192)
        if msg.control_mode == 1:  # 电流模式
            self.odrives[3].axis0.controller.current_setpoint = msg.x
            self.odrives[3].axis1.controller.current_setpoint = msg.y

    def spin(self):
        leg_threads = [0,0,0,0]
        for index,odrive in enumerate(self.odrives):
            if odrive == 0:
                continue
            # 开四个线程发送机器人测得实际数据
            leg_threads[index] = actualPubThread(leg_order[index],self.odrives[index],(ODRIVE_OFFSET[index*2],ODRIVE_OFFSET[index*2+1]),self.loop_rate_s)
            leg_threads[index].start()
        # 检测是否有腿掉电
        while not rospy.is_shutdown():
            for index,thread in enumerate(leg_threads):
                if self.odrives[index] != 0:
                    try:
                        thread.odrive.config
                    except:
                        self.odrives[index] = 0
                        if sum(self.odrives) == 0:
                            rospy.logerr("There is no leg alive! Exit!")
                            exit()
                rospy.sleep(0.1)


def InverseForAll(x_f, y_f, LEG_LENGTH):  # x_f,y_f 为足端球心坐标
    C1 = pow(x_f,2) + pow(y_f,2) + pow(LEG_LENGTH[0],2) - pow((LEG_LENGTH[1]+DELTA_L2),2)  # 中间常量
    A = 1 + pow(x_f,2) / pow(y_f,2)
    B = -C1 * x_f / pow(y_f,2)
    C = pow(C1,2) / (4 * pow(y_f,2)) - pow(LEG_LENGTH[0],2)
    # x_knee = (-B - sqrt(pow(B,2) - 4*A*C))/(2 * A)
    x_knee = (-B + sqrt(pow(B,2) - 4*A*C))/(2 * A)
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


def find_dev_num(filename):
    a = []
    f = open(filename)
    buff = f.read()
    p = re.compile(r"Bus 003 Device \d+: ID 1209:0d32 InterBiometrics")
    for bu in p.findall(buff):
        p = re.compile(r"\d+")
        a.append(int(p.findall(bu)[1]))
    f.close()
    return a


class actualPubThread(threading.Thread):
    def __init__(self,which_leg = 'RF', odrive = 0, m_ODRIVE_OFFSET = (0,0),spin_rate_s = 0.01):
        threading.Thread.__init__(self)
        self.which_leg = which_leg
        self.current_publisher = rospy.Publisher('Odrive/'+which_leg+'/actual_current',FourLegCurrentMsg,queue_size=10)
        self.pos_publisher = rospy.Publisher('Odrive/'+which_leg+'/actual_pos',FourLegPosMsg,queue_size=10)
        self.motor_angle_publisher = rospy.Publisher('Odrive/'+which_leg+'/actual_angle',FourLegPosMsg,queue_size=10)
        self.odrive = odrive
        self.m_ODRIVE_OFFSET = m_ODRIVE_OFFSET
        self.spin_rate = spin_rate_s
        self.time_old = time.time()-60

    def run(self):
        while not rospy.is_shutdown():
            try:
                # 电压检测
                if time.time() - self.time_old >= 60:
                    voltage = self.odrive.vbus_voltage
                    if voltage <= 11.4 or (voltage > 12.8 and voltage <= 15.6):
                        rospy.logwarn("{}_leg battery voltage : {}\r".format(self.which_leg,voltage))
                    else:
                        rospy.loginfo("{}_leg battery voltage : {}\r".format(self.which_leg,voltage))
                    self.time_old = time.time()
                # 发布当前各个状态
                if leg_cipher[self.which_leg][1] == 0:
                    self.current_publisher.publish(FourLegCurrentMsg(self.odrive.axis0.motor.current_control.Iq_measured,
                                                                     -self.odrive.axis1.motor.current_control.Iq_measured))
                    motor_pos = [self.odrive.axis0.encoder.pos_estimate - self.m_ODRIVE_OFFSET[0],
                                 self.odrive.axis1.encoder.pos_estimate - self.m_ODRIVE_OFFSET[1]]
                else:
                    self.current_publisher.publish(FourLegCurrentMsg(self.odrive.axis1.motor.current_control.Iq_measured,
                                                                     self.odrive.axis0.motor.current_control.Iq_measured))
                    motor_pos = [self.odrive.axis1.encoder.pos_estimate - self.m_ODRIVE_OFFSET[0],
                                 self.odrive.axis0.encoder.pos_estimate - self.m_ODRIVE_OFFSET[1]]
                self.motor_angle_publisher.publish(FourLegPosMsg(motor_pos[0]/8192*360,
                                                                 motor_pos[1]/8192*360))
                # 各腿独立爆毙检测
                if motor_pos[0] + motor_pos[1] >= (180 - MIN_ANGLE_BETWEEN_TWO_LEGS)/360*8192 or \
                        motor_pos[0] + motor_pos[1] <= -(180 - MIN_ANGLE_BETWEEN_TWO_LEGS)/360*8192:
                    rospy.logerr(self.which_leg + " LEGS MAY HAVE CONFLICT!")
                    self.odrive.axis0.requested_state = AXIS_STATE_IDLE
                    self.odrive.axis1.requested_state = AXIS_STATE_IDLE
                # 变弧度
                motor_pos[0] = motor_pos[0]/8192*2*pi
                motor_pos[1] = motor_pos[1]/8192*2*pi
                # 正运动学
                alpha = -motor_pos[0]
                beta = -motor_pos[1]
                theta1 = (alpha+beta)/2
                theta2 = (alpha-beta)/2
                l1 = LEG_LENGTH[0]
                l2 = LEG_LENGTH[1]
                L = -l1*sin(theta1)+sqrt(pow(l2,2)-pow(l1,2)*pow(cos(theta1),2))
                x = L * sin(theta2)
                y = - L * cos(theta2)
                # theta3 = theta2 + asin(l1 * cos(theta1) / l2)
                theta3 = theta2 - asin(l1 * cos(theta1) / l2)
                x = x + DELTA_L2 * sin(theta3)
                y = y - DELTA_L2 * cos(theta3)
                self.pos_publisher.publish(FourLegPosMsg(x,y))
            except fibre.protocol.ChannelBrokenException:
                rospy.logwarn("{}_leg has lost !".format(self.which_leg))
                return


if __name__ == '__main__':
    mOdriveDriver = OdriveDriver()