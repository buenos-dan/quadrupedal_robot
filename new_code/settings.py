STATE_GRAPH = {"IDLE":["WALK","JUMP"],
               "WALK":["IDLE"],
               "JUMP":["IDLE"],}

def find_shortest_path(graph,start,end,path = []):
    path = path + [start]
    if start == end:
        return path
    if not graph.has_key(start):
        return None
    shortest = None
    for node in graph[start]:
        if node not in path:
            newpath = find_shortest_path(graph,node,end,path)
            if newpath:
                if not shortest or len(newpath)<len(shortest):
                    shortest = newpath
    return shortest

if __name__=="__main__":
    print find_shortest_path(STATE_GRAPH,"WALK","JUMP")
    print find_shortest_path(STATE_GRAPH,"IDLE","JUMP")
    print find_shortest_path(STATE_GRAPH,"JUMP","WALK")



        
