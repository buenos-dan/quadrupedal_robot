#!/usr/bin/env python
# -*-coding:utf-8-*-
# **********************  1_22测试结果最高位置 y = -0.225  ***********************

# ***********************  total parameters  ***********************
field_cipher = {"red":1,"blue":-1}
g = 9.8015
leg_order = ['RF','RB','LF','LB']
CONTROL_RATE = 0.01
PUBLISHERS_NAME = {
    'four_leg':['/Odrive/RF/control',
                '/Odrive/RB/control',
                '/Odrive/LF/control',
                '/Odrive/LB/control'],

    'imu'     :['imu_driver'],
}
LEG_LENGTH = [0.15, 0.3]
BODY_LENGTH = [0.284, 0.46]
DELTA_L2 = 0.05
END_BALL_R = 0.02

# ***********************  安全参数  ***********************
MIN_ANGLE_BETWEEN_TWO_LEGS = 5

# ***********************  步态参数  ***********************
STAND_OFFSET = [0.03,-0.03,0.03,-0.03]
# STAND_OFFSET = [-0.05,-0.08,-0.05,-0.08]
# STAND_OFFSET = [-0.01,-0.01,-0.01,-0.01]
SQUAT_HEIGHT = -0.23
# ***********************  ORIVE参数  ***********************
ODRIVE_ID = {"35580456622152":"RF",
             "35644881197128":"RB",
             "35593342310472":"LF",
             "35644881131592":"LB"}
leg_cipher = {
    "RF":[0,0],
    "RB":[1,1],
    "LF":[2,1],
    "LB":[3,0]
}
# {"leg_key":(front_offset, back_offset, axis_reverse_flag, front_cur_reverse_flag, back_cur_reverse_flag)}
ODRIVE_PARA = {"RF":(400,0,0,1,-1),
                 "RB":(-325,-300,1,-1,-1),
                 "LF":(0,750,1,-1,-1),
                 "LB":(600,1200,0,-1,1)}
ODRV_POS_GAIN = 90
ODRV_VEL_GAIN = 0.001
ODRV_VEL_INTEGRATOR_GAIN = 0.00
ODRV_VEL_LIMIT = 100000
ODRV_CURRENT_LIM = 50


# ***********************  IMU参数  ***********************
IMU_PORT = '/dev/imu'
IMU_BAUDRATE = 115200
IMU_FREQENCY = 50.0

# *********************** 电机参数 **************************
KI = 0.095  # torque = KI * current
