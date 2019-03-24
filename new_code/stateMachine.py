from settings import *


class stateMechine:
    def __init__(self):
        self.current_state = "IDLE"
        self.state_graph = STATE_GRAPH

    def calculateStatePath(self,set_state):
        path = find_shortest_path(self.state_graph,self.current_state,set_state)
        return path

    def changeState(self,path):
        pass
            
        
        

    
    

if __name__=="__main__":
    print "hello world"
