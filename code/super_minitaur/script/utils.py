#!/usr/bin/env python
# coding:utf-8
import rospy
import numpy as np
from math import *
from settings import *


def TorqueCal(f_x,f_y,angle_x,angle_y):
	'''
    :param Fx:足端x分力 N
    :param Fy:足端y分力 N
    :param angle_x:前电机转角 rad
    :param angle_y:后电机转角 rad
    :return:[[前电机电流，
              后电机电流]]
    '''
	force = np.mat([[f_x],
					[0],
					[f_y]])
	jaco = Jacobian3(angle_x, angle_y)
	torque = jaco.T * force
	cur = TorqueToCur(torque)
	return [float(cur[0][0]),float(cur[2][0])]


def Jacobian3(alpha, beta):
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
	jaco = [[A-B, 	0., 	A+B],
			[0,		0.,		0.],
			[C+D,	0.,		C-D]]
	jaco = np.mat(jaco)
	return jaco

def mat_Q(act_pos_xf, act_pos_zf, act_pos_xb, act_pos_zb):
	xf = 0.23+act_pos_xf
	yf = 0.142
	zf = act_pos_zf

	xb = -0.23+act_pos_xb
	yb = -0.142
	zb = act_pos_zb

	mat_Q = [[  1,   0,   0,   1,   0,   0],
			 [  0,   0,   1,   0,   0,   1],
			 [  0, -zf,  yf,   0, -zb,  yb],
			 [ zf,   0, -xf,  zb,   0, -xb],
			 [-yf,  xf,   0, -yb,  xb,   0],
			 [  0,   1,   0,   0,  -1,   0]
			 ]
	mat_Q = np.mat(mat_Q)

	return mat_Q.I

def CurToTorque(cur):
	return 6.5/(5*13.3)*cur


def TorqueToCur(torque):
	return (13.3*5)/6.5*torque


def createPub(name,msgType):
	self.pub_name = name
	self.pub_type = msgType
	return rospy.Publisher(name,msgType,queue_size=10)


if __name__ == '__main__':
	# test
	cur = TorqueCal(0,-50,-0/180.0*pi,-0/180.0*pi)
	print(cur)
