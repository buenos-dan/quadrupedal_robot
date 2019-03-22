#!/usr/bin/env python
# -*-coding:utf-8-*-
# **********************  1_22测试结果最高位置 y = -0.225  ***********************

# ***********************  total parameters  ***********************
leg_order = ['RF','RB','LF','LB']
MINITAUR_VERSION = 0.1
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
STEP_START_POINT = 0
STEP_END_POINT = 0.01
STEP_MIDDLE_POINT = 0
STEP_GROUND_HEIGHT = -0.38
STEP_HEIGHT = 0.08
STEP_FREQUENCY= 100         
DEFAULT_POS = [[[-0.06,STEP_GROUND_HEIGHT+STEP_HEIGHT],
				[-0.06,STEP_GROUND_HEIGHT+STEP_HEIGHT],
				[-0.06,STEP_GROUND_HEIGHT+STEP_HEIGHT],
				[-0.06,STEP_GROUND_HEIGHT+STEP_HEIGHT]]]

TROT_OFFSET = [0.01,-0.01,0.01,-0.01]
STAND_OFFSET = [0.03,-0.03,0.03,-0.03]
JUMP_OFFSET = [-0.03,-0.04,-0.03,-0.04]
SQUAT_HEIGHT = -0.23
DEFAULT_OFFSET = [0.03,-0.03,0.03,-0.03]
TIGER_OFFSET = [0.03,-0.03]
LEG_LENGTH_MAX = 0.42
LEG_LENGTH_MIN = 0.18
T_STAND = 0.5
T_RUN = 0.2
STEP_LENGTH = 0.05

# ***********************  ORIVE参数  ***********************
# [逻辑位置，硬件M1,M2是否装反]
leg_cipher = {
	"RF":[0,0],
	"RB":[1,1],
	"LF":[2,1],
	"LB":[3,0]
}
ODRIVE_OFFSET = [550,300,-400,1000,100,-150,500,350]  # 先前后后
ODRV_POS_GAIN = 90
ODRV_VEL_GAIN = 0.001
ODRV_VEL_INTEGRATOR_GAIN = 0.00
ODRV_VEL_LIMIT = 100000
ODRV_CURRENT_LIM = 40


# ***********************  IMU参数  ***********************
IMU_PORT = '/dev/ttyUSB0'
IMU_BAUDRATE = 115200
IMU_FREQENCY = 50.0
