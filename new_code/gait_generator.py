#!/usr/bin/env python
import rospy
from std_msgs.msg import *
import time


class gaitGenerator:
    def __init__(self):
	rospy.init_node("robot/gait_generator",anonymous = True)
	self.state_suber = rospy.Subscriber("robot/statechange",String,self.tempstate_callback)
	self.temp_state = "IDLE"

    def tempstate_callback(self,msg):
	self.temp_state = msg.data

    def run(self):
	while 1:
	    time.sleep(2)    
	    rospy.set_param("current_state",self.temp_state)
	    print "set tempstate to "+self.temp_state	



if __name__=="__main__":
    gaitgenerator = gaitGenerator()
    gaitgenerator.run()
    
