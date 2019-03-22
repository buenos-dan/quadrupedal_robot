#!/usr/bin/env python3
# coding:utf-8

import re,rospy,odrive,os,rospkg,threading,time,fibre,subprocess
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
        self.leg_threads = {}
        self.spin()

    def ros_init(self):
        rospy.init_node("Odrive_dirver",sys.argv,anonymous=True)
        self.control_para_suber = rospy.Subscriber('Odrive/parameter',OdriveParamMsg,self.control_parameter_callback)
        self.control_subbers = {}

    def odrive_init(self):  # RB和LF的M0和M1是反的
        # 找odrive
        rospy.loginfo("There is {} odrive waiting for connecting".format(len([info for info in \
                        str(subprocess.check_output("lsusb")).split("\\n") if info.endswith("InterBiometrics ")])))

        # self.odrives = {"RF":(class_odrive,"usb:001:010")}
        self.odrives = {ODRIVE_ID[str(driver[0].serial_number)]:driver for driver in \
                        [(odrive.find_any(device),device) for device in \
                        ["usb:%s:%s"%(ID.group(1),ID.group(2))for ID in \
                        [re.search("Bus (\d+) Device (\d+)",info) for info in \
                        str(subprocess.check_output("lsusb")).split("\\n") if info.endswith("InterBiometrics ")]]] \
                        if str(driver[0].serial_number) in ODRIVE_ID}

        # [usb:001:010, usb:001:011, usb:001:012]
        self.old_odrives = ["usb:%s:%s"%(ID.group(1),ID.group(2))for ID in \
                            [re.search("Bus (\d+) Device (\d+)",info) for info in \
                             str(subprocess.check_output("lsusb")).split("\\n") if info.endswith("InterBiometrics ")]]
        # 赋初值
        for key in self.odrives:
            self.init_set_para(key)
            # rospy.set_param("Odrive/"+key+"/ready_flag",1)
            rospy.set_param('Odrive_ready_flag',1)
        rospy.loginfo('OK!')

    def leg_control(self, key, msg):
        if key in self.odrives:
            self.odrives[key][0].axis0.controller.config.control_mode = msg.control_mode
            self.odrives[key][0].axis1.controller.config.control_mode = msg.control_mode
            if ODRIVE_PARA[key][2]:
                if msg.control_mode == 3:  # 位置模式 rad
                    self.odrives[key][0].axis1.controller.pos_setpoint = int(msg.second/(2*pi) * 8192)+ODRIVE_PARA[key][0]
                    self.odrives[key][0].axis0.controller.pos_setpoint = int(msg.third/(2*pi) * 8192)+ODRIVE_PARA[key][1]
                if msg.control_mode == 2:  # 速度模式 rad/s
                    self.odrives[key][0].axis1.controller.vel_setpoint = int(msg.second/(2*pi) * 8192)
                    self.odrives[key][0].axis0.controller.vel_setpoint = int(msg.third/(2*pi) * 8192)
                if msg.control_mode == 1:  # 电流模式 A
                    self.odrives[key][0].axis1.controller.current_setpoint = msg.second
                    self.odrives[key][0].axis0.controller.current_setpoint = msg.third
            else:
                if msg.control_mode == 3:  # 位置模式
                    self.odrives[key][0].axis0.controller.pos_setpoint = int(msg.second/(2*pi) * 8192)+ODRIVE_PARA[key][0]
                    self.odrives[key][0].axis1.controller.pos_setpoint = int(msg.third/(2*pi) * 8192)+ODRIVE_PARA[key][1]
                if msg.control_mode == 2:  # 速度模式
                    self.odrives[key][0].axis0.controller.vel_setpoint = int(msg.second/(2*pi) * 8192)
                    self.odrives[key][0].axis1.controller.vel_setpoint = int(msg.third/(2*pi) * 8192)
                if msg.control_mode == 1:  # 电流模式
                    self.odrives[key][0].axis0.controller.current_setpoint = msg.second
                    self.odrives[key][0].axis1.controller.current_setpoint = msg.third

    def disable_ready_flag(self):
        for key in self.odrives:
            rospy.set_param("Odrive/"+key+"/ready_flag",0)

    def enable_ready_flag(self):
        for key in self.odrives:
            rospy.set_param("Odrive/"+key+"/ready_flag",1)

    def RF_callback(self,msg):
        self.leg_control("RF",msg)

    def RB_callback(self,msg):  # M0,M1反了
        self.leg_control("RB",msg)

    def LF_callback(self,msg):  # M0,M1反了
        self.leg_control("LF",msg)

    def LB_callback(self,msg):
        self.leg_control("LB",msg)

    def control_parameter_callback(self,msg):
        if not msg.odrive_index in self.odrives:
            return
        self.odrives[msg.odrive_index][0].axis0.controller.config.pos_gain = msg.pos_gain
        self.odrives[msg.odrive_index][0].axis0.controller.config.vel_gain = msg.vel_gain
        self.odrives[msg.odrive_index][0].axis0.controller.config.vel_integrator_gain = msg.vel_integrator_gain
        self.odrives[msg.odrive_index][0].axis0.controller.config.vel_limit = msg.vel_limit
        self.odrives[msg.odrive_index][0].axis0.motor.config.current_lim = msg.current_limit
        self.odrives[msg.odrive_index][0].axis1.controller.config.pos_gain = msg.pos_gain
        self.odrives[msg.odrive_index][0].axis1.controller.config.vel_gain = msg.vel_gain
        self.odrives[msg.odrive_index][0].axis1.controller.config.vel_integrator_gain = msg.vel_integrator_gain
        self.odrives[msg.odrive_index][0].axis1.controller.config.vel_limit = msg.vel_limit
        self.odrives[msg.odrive_index][0].axis1.motor.config.current_lim = msg.current_limit

    def check_new_odrive(self):
        new_odrives = ["usb:%s:%s"%(ID.group(1),ID.group(2))for ID in \
                       [re.search("Bus (\d+) Device (\d+)",info) for info in \
                        str(subprocess.check_output("lsusb")).split("\\n") if info.endswith("InterBiometrics ")]]
        for dev in list(set(new_odrives) - set(self.old_odrives)):
            rospy.logwarn("There is a new odrive, connectting!")
            odrive_tmp = odrive.find_any(dev)
            key = ODRIVE_ID[str(odrive_tmp.serial_number)]
            self.odrives[key] = (odrive_tmp,dev)
            self.init_set_para(key)
            self.leg_threads[key] = actualPubThread(key,self.odrives[key][0])
            self.leg_threads[key].start()
            rospy.logwarn(key + "_leg connected")

        for dev in list(set(self.old_odrives) - set(new_odrives)):
            disconnect_flag = 0  # 不可在dict循环中对自己执行pop
            for leg_key in self.odrives:
                if self.odrives[leg_key][1] == dev:
                    disconnect_flag = leg_key
            if disconnect_flag != 0:
                self.odrives.pop(disconnect_flag)
                self.leg_threads.pop(disconnect_flag)
        self.old_odrives = new_odrives

    def spin(self):
        for key in self.odrives:
            # 开线程发送机器人测得实际数据
            self.leg_threads[key] = actualPubThread(key,self.odrives[key][0])
            self.leg_threads[key].start()
        while not rospy.is_shutdown():
            self.check_new_odrive()
            rospy.sleep(0.1)

    def init_set_para(self,key):
        self.odrives[key][0].axis0.controller.config.control_mode = 3
        self.odrives[key][0].axis0.controller.config.pos_gain = ODRV_POS_GAIN
        self.odrives[key][0].axis0.controller.config.vel_gain = ODRV_VEL_GAIN
        self.odrives[key][0].axis0.controller.config.vel_integrator_gain = ODRV_VEL_INTEGRATOR_GAIN
        self.odrives[key][0].axis0.controller.config.vel_limit = ODRV_VEL_LIMIT
        self.odrives[key][0].axis0.motor.config.current_lim = ODRV_CURRENT_LIM
        self.odrives[key][0].axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrives[key][0].axis1.controller.config.control_mode = 3
        self.odrives[key][0].axis1.controller.config.pos_gain = ODRV_POS_GAIN
        self.odrives[key][0].axis1.controller.config.vel_gain = ODRV_VEL_GAIN
        self.odrives[key][0].axis1.controller.config.vel_integrator_gain = ODRV_VEL_INTEGRATOR_GAIN
        self.odrives[key][0].axis1.controller.config.vel_limit = ODRV_VEL_LIMIT
        self.odrives[key][0].axis1.motor.config.current_lim = ODRV_CURRENT_LIM
        self.odrives[key][0].axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.control_subbers[key] = rospy.Subscriber('Odrive/'+key+'/control',LegControlMsg,eval("self."+key+"_callback"))


class actualPubThread(threading.Thread):
    def __init__(self,key = 'RF', odrive = 0):
        threading.Thread.__init__(self)
        self.key = key
        self.current_publisher = rospy.Publisher('Odrive/'+key+'/actual_current',FourLegCurrentMsg,queue_size=10)
        self.pos_publisher = rospy.Publisher('Odrive/'+key+'/actual_pos',FourLegPosMsg,queue_size=10)
        self.motor_angle_publisher = rospy.Publisher('Odrive/'+key+'/actual_angle',FourLegPosMsg,queue_size=10)
        self.odrive = odrive
        self.time_old = time.time()-60

    def run(self):
        while not rospy.is_shutdown():
            try:
                # 电压检测
                if time.time() - self.time_old >= 60:
                    voltage = self.odrive.vbus_voltage
                    if voltage <= 11.4 or (voltage > 12.8 and voltage <= 15.6) or (voltage > 17.2 and voltage <= 22.8):
                        rospy.logwarn("{}_leg battery voltage : {:.1f}\r".format(self.key,voltage))
                    else:
                        rospy.loginfo("{}_leg battery voltage : {:.1f}\r".format(self.key,voltage))
                    self.time_old = time.time()
                # 发布当前各个状态
                if ODRIVE_PARA[self.key][2]:  # 前后腿对应轴反了
                    self.current_publisher.publish(FourLegCurrentMsg(ODRIVE_PARA[self.key][4]*self.odrive.axis1.motor.current_control.Iq_measured,
                                                                     ODRIVE_PARA[self.key][3]*self.odrive.axis0.motor.current_control.Iq_measured))
                    motor_pos = [self.odrive.axis1.encoder.pos_estimate - ODRIVE_PARA[self.key][0],
                                 self.odrive.axis0.encoder.pos_estimate - ODRIVE_PARA[self.key][1]]
                else:
                    self.current_publisher.publish(FourLegCurrentMsg(ODRIVE_PARA[self.key][3]*self.odrive.axis0.motor.current_control.Iq_measured,
                                                                     ODRIVE_PARA[self.key][4]*self.odrive.axis1.motor.current_control.Iq_measured))
                    motor_pos = [self.odrive.axis0.encoder.pos_estimate - ODRIVE_PARA[self.key][0],
                                 self.odrive.axis1.encoder.pos_estimate - ODRIVE_PARA[self.key][1]]
                self.motor_angle_publisher.publish(FourLegPosMsg(motor_pos[0]/8192*360,
                                                                 motor_pos[1]/8192*360))
                # 各腿独立爆毙检测
                if motor_pos[0] + motor_pos[1] >= (180 - MIN_ANGLE_BETWEEN_TWO_LEGS)/360*8192 or \
                        motor_pos[0] + motor_pos[1] <= -(180 - MIN_ANGLE_BETWEEN_TWO_LEGS)/360*8192:
                    rospy.logerr(self.key + " LEGS MAY HAVE CONFLICT!")
                    self.odrive.axis0.requested_state = AXIS_STATE_IDLE
                    self.odrive.axis1.requested_state = AXIS_STATE_IDLE
                # 变弧度
                motor_pos[0] = motor_pos[0]/8192*2*pi
                motor_pos[1] = motor_pos[1]/8192*2*pi
                self.pos_publisher.publish(FourLegPosMsg(motor_pos[0],motor_pos[1]))
            except fibre.protocol.ChannelBrokenException:
                rospy.logerr("{}_leg has lost !".format(self.key))
                return


if __name__ == '__main__':
    mOdriveDriver = OdriveDriver()