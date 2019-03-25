from settings import *


class stateMechine:
    def __init__(self):
        self.current_state = "IDLE"
        self.set_state = "IDLE"
        self.state_graph = STATE_GRAPH

    def calculateStatePath(self,set_state):
        path = find_shortest_path(self.state_graph,self.current_state,set_state)
        return path

    def changeState(self,path):
        while self.current_state !=  :
        pass

    def run(self):
        while 1:
            if cur_state == set_state and path:
                state = self.getState()
                sendState(state)
            time.sleep(0.01)

    def sendState(self):
        pass

    def getState(self):
        state = self.path[0]
        del self.path[0]
        return state

    def motion_callback(self):
        pass


if __name__=="__main__":
    statemechine = stateMechine()
    statemechine.run()
