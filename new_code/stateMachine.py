import rospy
import time
from std_msgs.msg import *
from settings import *


class stateMechine:
    def __init__(self):
	rospy.init_node("state_mechine",anonymous = True)
	rospy.set_param("current_state","IDLE")
	rospy.set_param("temp_state","IDLE")
	self.motion_suber = rospy.Subscriber("robot/motion_plan",String,self.setstate_callback)
	self.state_puber = rospy.Publisher("robot/statechange",String,queue_size = 10)
        self.state_graph = STATE_GRAPH
	self.path = ["IDLE"]

    def calculateStatePath(self,target_state):
	current_state = rospy.get_param("current_state")
        path = find_shortest_path(self.state_graph,current_state,target_state)
        return path


    def run(self):
        while 1:
	    current_state = rospy.get_param("current_state")
	    temp_state = rospy.get_param("temp_state")
            if current_state == temp_state and self.path:
                state = self.getState()
                self.sendState(state)
            time.sleep(0.01)

    def sendState(self,state):
	self.state_puber.publish(state)
	

    def getState(self):
        state = self.path[0]
        del self.path[0]
        return state

    def setstate_callback(self,msg):
	self.path = calculateStatePath(msg.data)
	

if __name__=="__main__":
    statemechine = stateMechine()
    statemechine.run()
