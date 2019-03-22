#!/usr/bin/env python 
# -*-coding:utf-8-*-
import rospy
from super_minitaur.msg import *
from settings import *
import numpy as np
import time
from math import *

class Controller:
    def __init__(self):
        rospy.init_node("controller",anonymous=True)
        self.leg_pubs = [rospy.Publisher(name, LegControlMsg, queue_size=10) for name in PUBLISHERS_NAME['four_leg']]
        self.x_arrays = [np.linspace(STEP_START_POINT,STEP_END_POINT,STEP_FREQUENCY/2),
                         np.linspace(STEP_END_POINT,STEP_MIDDLE_POINT,STEP_FREQUENCY/4),
                         np.linspace(STEP_MIDDLE_POINT,STEP_START_POINT,STEP_FREQUENCY/4)]
        self.y_array = np.linspace(STEP_GROUND_HEIGHT,STEP_GROUND_HEIGHT+STEP_HEIGHT,STEP_FREQUENCY/4)
        self.subers = [rospy.Subscriber("Odrive/RF/actual_pos",FourLegPosMsg,self.RF_pos_callback),
                        rospy.Subscriber("Odrive/RB/actual_pos",FourLegPosMsg,self.RB_pos_callback),
                        rospy.Subscriber("Odrive/LF/actual_pos",FourLegPosMsg,self.LF_pos_callback),
                        rospy.Subscriber("Odrive/LB/actual_pos",FourLegPosMsg,self.LB_pos_callback)]
        self.actual_pos =[0 for i in range(8)] 

    def run(self):
        while not rospy.is_shutdown():
            state = raw_input("please type command:")
            if state == "init":
                self.init_minitaur()
                print (T_RUN/(STEP_FREQUENCY/2))
            if state == "trot":
                self.trot_init()
                while 1:
                    self.trot()

    def trot(self):
        for i in range(STEP_FREQUENCY/2):
            position_arr = np.zeros(8)
            for j in range(4):
                if j == 0 or j == 3:
                    position_arr[j*2] = self.x_arrays[0][i] 
                    position_arr[j*2+1] = STEP_GROUND_HEIGHT
                else:
                    if i//(STEP_FREQUENCY/4)==0:
                        position_arr[j*2] = self.x_arrays[1][i%(STEP_FREQUENCY/4)] 
                        position_arr[j*2+1] = self.y_array[i%(STEP_FREQUENCY/4)] 
                    if i//(STEP_FREQUENCY/4)==1:
                        position_arr[j*2] = self.x_arrays[2][i%(STEP_FREQUENCY/4)] 
                        position_arr[j*2+1] = self.y_array[::-1][i%(STEP_FREQUENCY/4)]
            time.sleep(T_RUN/(STEP_FREQUENCY/2.0))        #T_STAND
            self.pub_msg(position_arr)

        for i in range(STEP_FREQUENCY/2):
            position_arr = np.zeros(8)
            for j in range(4):
                if j == 1 or j == 2:
                    position_arr[j*2] = self.x_arrays[0][i] 
                    position_arr[j*2+1] = STEP_GROUND_HEIGHT
                else:
                    if i//(STEP_FREQUENCY/4)==0:
                        position_arr[j*2] = self.x_arrays[1][i%(STEP_FREQUENCY/4)] 
                        position_arr[j*2+1] = self.y_array[i%(STEP_FREQUENCY/4)] 
                    if i//(STEP_FREQUENCY/4)==1:
                        position_arr[j*2] = self.x_arrays[2][i%(STEP_FREQUENCY/4)] 
                        position_arr[j*2+1] = self.y_array[::-1][i%(STEP_FREQUENCY/4)]
            time.sleep(T_RUN/(STEP_FREQUENCY/2.0))        #T_STAND
            self.pub_msg(position_arr)
        

    def trot_init(self):
        temp_ys_array = [np.linspace(self.actual_pos[i*2+1],STEP_GROUND_HEIGHT,STEP_FREQUENCY) for i in range(4)]
        x_pos_begin = self.actual_pos
        for i in range(STEP_FREQUENCY):
            position_y_arr = [item[i] for item in temp_ys_array]
            position_arr = []
            for i in range(4):
                position_arr.append(x_pos_begin[i*2])
                position_arr.append(position_y_arr[i])
            self.pub_msg(position_arr)  
            time.sleep(T_RUN/(STEP_FREQUENCY+0.1))        #T_STAND


    def init_minitaur(self):
        temp_xs_array = [np.linspace(self.actual_pos[i*2],DEFAULT_POSITION[i*2],STEP_FREQUENCY) for i in range(4)]
        temp_ys_array = [np.linspace(self.actual_pos[i*2+1],DEFAULT_POSITION[i*2+1],STEP_FREQUENCY) for i in range(4)]
        for i in range(STEP_FREQUENCY):
            position_y_arr = [item[i] for item in temp_ys_array]
            position_x_arr = [item[i] for item in temp_xs_array]
            position_arr = []
            for i in range(4):
                position_arr.append(position_x_arr[i])
                position_arr.append(position_y_arr[i])
            self.pub_msg(position_arr)  
            time.sleep(T_STAND/STEP_FREQUENCY)        #T_STAND
    
        

    def pub_msg(self,position_array):
        msgs = [LegControlMsg(3,position_array[i*2]+DEFAULT_OFFSET[i],position_array[i*2+1]) for i in range(4)]
        #security detect
        legs_L = [sqrt(pow(position_array[i*2], 2) + pow(position_array[i*2+1], 2)) for i in range(4)]
        for leg_L in legs_L:
            if leg_L > LEG_LENGTH_MAX or leg_L < LEG_LENGTH_MIN:
                rospy.logwarn("position out of range!")
                rospy.loginfo(leg_L)
                rospy.loginfo(position_array)
                return -1

        
        for index,msg in enumerate(msgs):
            self.leg_pubs[index].publish(msg)
    
    
        
        

    def RF_pos_callback(self,msg):
        self.actual_pos[0]=msg.x
        self.actual_pos[1]=msg.y
    def RB_pos_callback(self,msg):
        self.actual_pos[2]=msg.x
        self.actual_pos[3]=msg.y
    def LF_pos_callback(self,msg):
        self.actual_pos[4]=msg.x
        self.actual_pos[5]=msg.y
    def LB_pos_callback(self,msg):
        self.actual_pos[6]=msg.x
        self.actual_pos[7]=msg.y
        






if __name__=="__main__":
    controller = Controller()
    controller.run()

