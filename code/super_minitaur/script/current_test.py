#!/usr/bin/env python3
# coding:utf-8
import odrive
import time
import matplotlib.pyplot as plt
import numpy as np
from settings_new import *
from math import *

g = 9.8015
d = 0.066145
m2 = 0.0458
k_friction = 1e-5
j1 = 1.2e-4
#j2 = 1.158e-4
j2 = 0.2e-4

spin_rate = 0.002
T = 0.2

if __name__ == '__main__':
    RF_leg = odrive.find_any()
    if RF_leg != 0:
        print("Odrive finded !")
    t = 0.
    num = 0
    curs = []
    theo_torques = []
    actual_torques = []
    #nihe
    theo_thetas = []
    listTheta__ = []
    thetas = []

    v_s = []
    theo_v_s = []
    a_s = []
    k_s = []
    theo_a_s = []
    d_torques = []
    old_theta_ = 0.
    old_theta__ = 0.
    old_cur = 0.
    first_flag = 1
    theta__alpha = 0.9
    current__alpha = 0.5
    RF_leg.axis0.controller.config.control_mode = 3
    RF_leg.axis0.controller.pos_setpoint = ODRIVE_OFFSET[0]
    time.sleep(1)
    t_start = time.time()
    while 1:
        # 设电流
        # RF_leg.axis0.controller.config.control_mode = 1
        # current_set = 5.0 # * 5 * sin(t/T * 2 * pi)
        # RF_leg.axis0.controller.current_setpoint = current_set
        # 设速度
        # vel_set = 8192 * 5 * sin(t/T * 2 * pi)
        # RF_leg.axis0.controller.config.control_mode = 2
        # RF_leg.axis0.controller.vel_setpoint = vel_set
        # 设位置
        friction = k_friction*RF_leg.axis0.encoder.vel_estimate /8192 * 2 * pi
        pos_set = -5.0 * sin(2 * pi/T * t)
        theo_thetas.append(pos_set)
        pos_set = pos_set * 8192.0 / (2 * pi) + ODRIVE_OFFSET[0]
        RF_leg.axis0.controller.config.control_mode = 3
        RF_leg.axis0.controller.pos_setpoint = pos_set
        # 计算
        motor_pos = RF_leg.axis0.encoder.pos_estimate - ODRIVE_OFFSET[0]
        theta = motor_pos/8192.0 *2*pi     # rad
        thetas.append(theta)
        # theo_theta = -5 * T * cos(2 * t * pi / T) + 5 * T - pi
        theta_ = RF_leg.axis0.encoder.vel_estimate /8192 * 2 * pi
        theta__ = (theta_ - old_theta_)/spin_rate
        theta__ = (1 - theta__alpha) * old_theta__ + theta__alpha * theta__
        listTheta__.append(theta__)
        old_theta_ = theta_
        old_theta__ = theta__

        # 拉格朗日方程结果
        torque_cal = (j1 + j2) * theta__ + m2 * g * d * cos(theta+pi)+friction
        
        # 将计算出的力矩转换为理论电流
        current_theo = torque_cal / KI
        # 存数组
        if first_flag:
            old_cur = -RF_leg.axis0.motor.current_control.Iq_measured
            old_theta_ = RF_leg.axis0.encoder.vel_estimate / 8192 *2*pi
            first_flag = 0
            continue
        
        theo_theta__ =  2 * 2 * pi * 2 * pi / T * cos(t/T * 2 * pi)
        # theo_a_s.append(theo_theta__)
        # v_s.append(theta_)
        # theo_v_s.append(vel_set/8192. * 2* pi)
        # a_s.append(theta__)
        current = (1-current__alpha) * old_cur + current__alpha * (-RF_leg.axis0.motor.current_control.Iq_measured)
        torque_actual = current * KI
        actual_torques.append(torque_actual)
        # print ("theoretical current is %f, actual current is %f" %(current_theo, current))
        # current = RF_leg.axis0.motor.current_control.Iq_measured
        # print (current)
        # curs.append(current)
        old_cur = current
        theo_torques.append(torque_cal)
        # if abs(current) > 1e-7:
        #     k_data = torque_cal / current
        #     # k_s.append(k_data)
        #     print (k_data)
        # print (torque_cal)
        t += spin_rate
        if t >= T:
            num += 1
        t = t % T
        time.sleep(spin_rate)
        if time.time()-t_start <1:
            continue
        if abs(torque_cal - torque_actual) > 0.6:
            print("detect collision!")
            break

    # 做图写文件
    # RF_leg.axis0.controller.current_setpoint = 0
    RF_leg.axis0.controller.vel_setpoint = 0
    time.sleep(0.5)
    # RF_leg.axis0.controller.config.control_mode = 2
    # RF_leg.axis0.controller.vel_setpoint = 0
    err = np.array(actual_torques)-np.array(theo_torques)
    relative_err =[err[i]/actual_torques[i] for i in range(len(err)) if abs(actual_torques[i]) > 1e-4] 
    t_x = np.linspace(0, T * 4, len(err))
    print (np.mean(relative_err))
    #torque_cal = (j1 + j2) * theta__ + m2 * g * d * cos(theta+pi)
    #size = min(len(listTheta__),len(actual_torques),len(thetas))
    #listTheta__ = listTheta__[len(listTheta__)-size:]
    #actual_torques = actual_torques[len(actual_torques)-size:]
    #thetas = thetas[len(thetas)-size:]
    X = np.array(listTheta__[101:])
    Y = np.array(actual_torques[100:]) - m2 * g * d * np.cos(np.array(thetas[101:])+pi)-friction
    z  =  np.polyfit(X,Y,1)
    print(z)
    plt.figure()
    # # plt.plot(t_x, k_s, 'r')
    # plt.plot(t_x, v_s, 'g')     # 速度
    # plt.plot(t_x, a_s, 'b')
    plt.plot(t_x, actual_torques, 'g:')     # 力矩
    plt.plot(t_x, theo_torques, 'b:')
    #plt.plot(t_x, err, 'r')

    # plt.plot(t_x, a_s, 'r')       # 理论和实际加速度
    # plt.plot(t_x, theo_a_s, 'b:')
    plt.show()
    # file = open("outcome.txt","w")
    # file.write("current\t\ttorques\n")
    # if file != 0:
    #     print("File has opened !")
    #     for i in range(len(curs)):
    #         file.write("{:f}\t\t{:f}\n".format(curs[i], torques[i]))
    #     file.close()
    print("FINISH")

# if __name__ == '__main__':
#     RF_leg = odrive.find_any()
#     if RF_leg != 0:
#         print("Odrive finded !")
#     t = 0
#     num = 0
#     curs = []
#     torques = []
#     v_s = []
#     a_s = []
#     old_theta_ = 0
#     first_flag = 1
#     while num < 5:
#         # 设位置
#         theta_set = pi/2 * sin(t/T * 2 * pi)
#         pos_set = int(theta_set / pi * 8192/2) + ODRIVE_OFFSET[0]
#         RF_leg.axis0.controller.pos_setpoint = pos_set
#         # 计算
#         motor_pos = RF_leg.axis0.encoder.pos_estimate - ODRIVE_OFFSET[0]
#         theta = motor_pos/8192*2*pi  # rad
#         theta_ = RF_leg.axis0.encoder.vel_estimate / 8192 * 2 * pi
#         theta__ = (theta_ - old_theta_)/spin_rate
#         old_theta_ = theta_
#         # 拉格朗日方程结果
#         torque_cal = (j1 + j2) * theta__ + m2 * g * d * cos(theta)
#         # 存数组
#         if first_flag:
#             first_flag = 0
#             continue
#         v_s.append(theta_)
#         a_s.append(theta__/100)
#         curs.append(RF_leg.axis0.motor.current_control.Iq_measured)
#         torques.append(torque_cal)
#         t += spin_rate
#         time.sleep(spin_rate)
#         if t >= T:
#             num += 1
#         t = t % T
#     # 做图写文件
#     plt.figure()
#     t_x = np.linspace(0,T*5,len(curs))
#     plt.plot(curs,torques)
#     plt.plot(t_x,v_s,'r:')
#     plt.plot(t_x,a_s,'b:')
#     plt.show()
#     file = open("outcome.txt","w")
#     file.write("current\t\ttorques\n")
#     if file != 0:
#         print("File has opened !")
#         for i in range(len(curs)):
#             file.write("{:.3f}\t\t{:.3f}\n".format(curs[i],torques[i]))
#         file.close()
#     print("FINISH")

