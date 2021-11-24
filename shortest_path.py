# -*- coding: utf-8 -*-
"""
Created on Wed Nov 24 13:22:07 2021

@author: sebas
"""

def find_shortest_path(list_vertices, list_neighbours, start=(-1,-1), goal=(-1,-1)):
    """Finds the shortest path on a graph.
    Inputs: 
        - list_vertices, list of lists of tuples [[(x1,y1),(x2,y2),...,(x_k_,y_k)],...,[(xx1,yy1),(xx2,yy2),...,(xx_n,yy_n)]]. 
        Each embedded list corresponds to an obstacle.
        Each tuple (x,y) contains the coordinates of a vertex of the obstacle.
    
        - list_neighbours, list of lists of tuples [[(vertex,distance),(vertex,distance),...],...,[(vertex,distance),(vertex,distance),...,(vertex,distance)]]
        Each embedded list corresponds to the neighbours of a given vertex.
        Each tuple (vertex,distance) contains the name of the neighbour and its distance from that vertex.
        
        - goal and start: coordinates (x,y) of the goal and start. If not specified, a Djikstra algorithm will be used ; otherwise, A* with Euclidean distance will be used.
    
    Output:
        best_path, ordered list that contains the optimal path for the graph. 
        Example: [0,1,3,5,6,7,8] where 0 is start and 8 is goal
        
    Pseudo-code (inspired from course lectures and partially taken from https://fr.wikipedia.org/wiki/Algorithme_de_Dijkstra , consulted 24.11.2021):
        
    """
    
    
    #%%Initialisation
    import math
    #attribute a name to each vertex
    list_vertices_flattened = [item for sublist in list_vertices for item in sublist]
    N = len(list_vertices_flattened)                                            #number of vertices
    name_vertices = list(range(N+2))
    
    #if A* is used: calculate the heuristic as euclidean distance. Otherwise, leave =0 for all vertices
    heuristic = [0]*(N+2)
    dist = lambda p1,p2: math.sqrt((p1[1]-p2[1])**2+(p1[0]-p2[0])**2)
    if goal!=(-1,-1):
        heuristic = [dist(goal,vertex) for vertex in list_vertices_flattened]
        heuristic.append(0)
        heuristic.insert(0,dist(goal,start))

    #list of vertices to be explored
    unexplored_names = name_vertices.copy()
    unexplored_names.remove(N+1)                                                #we don't want to explore the goal
    #unexplored_names.reverse()                                                  #check: works even is numbers are not ordered
    
    #attribute initial weights to each vertex
    infinity = 100000
    weights = [infinity]*(N+2)                                                  #minimal distance to reach each vertex. Before exploration, =infinity
    weights[0]=0
    predecessors = [-1]*(N+2)                                                   #best predecessor to reach each vertex (unknown at the beginning)
        
    
    #%% Applying Djikstra or A*
    while unexplored_names:                                                     #while there are still vertices to explore (vector non-empty)
        weights_unexplored_names = []
        for vertex in unexplored_names:
            weights_unexplored_names.append(weights[vertex])
        
        #set the current vertex to the one with lowest weight
        min_distance = min(weights_unexplored_names)
        current_vertex = unexplored_names[weights_unexplored_names.index(min_distance)]
        current_neighbours = list_neighbours[current_vertex]                    #get its neighbours with their associated distance
        
        #if needed, update the weight of neighbours and set current_vertex as their best predecessor
        for ngb in current_neighbours:
            weight_candidate = weights[current_vertex] + ngb[1] + heuristic[ngb[0]]
            if weight_candidate<weights[ngb[0]]:
                weights[ngb[0]] = weight_candidate
                predecessors[ngb[0]] = current_vertex
                
                #if this neighbour was already explored, re-explore it to update weights
                if ngb[0] not in unexplored_names:
                    unexplored_names.append(ngb[0])
        
        #remove the current vertex from unexplored vertices
        unexplored_names.remove(current_vertex)
        
        if ((N+1) in current_neighbours) and goal!=(-1,-1):                                     #if A* is used, no need to explore all graph
            print(unexplored_names)
            break
        
    
    #%% Retrace the best path
    best_path = []
    origin = predecessors[-1]
    while origin!=-1:
        best_path.insert(0,origin)
        origin = predecessors[origin]
    best_path.append(name_vertices[-1])

    
    return best_path

#---------------------------------------------------------
#%% Test
import time
import math

start_time = time.time()
list_vertices = [[(1,1),(2,2),(3,3)],[(4,4),(5,5),(6,6),(7,7)]]
start = (0,0)
goal = (10,10)

list_neighbours = [[(1,5),(2,10)],[(0,5),(3,4)],[(0,10),(3,15),(4,25)],[(1,4),(2,15),(5,30)],\
                   [(2,25),(5,15)],[(3,30),(4,15),(6,2),(7,7)],[(5,2),(7,3),(8,15)],[(5,7),(6,3),(8,2)],[(6,15),(7,2)]]
    
print("The shortest path is:{}".format(find_shortest_path(list_vertices, list_neighbours)))
print("Elapsed time: %s seconds" % (time.time() - start_time))

start_time = time.time()
list_vertices = [[(2,1),(2,3),(4,1)],[(4,6),(6,8),(8,6),(8,4)]] 
start = (0,0)
goal = (10,10)

dist = lambda p1,p2: math.sqrt((p1[1]-p2[1])**2+(p1[0]-p2[0])**2)
list_neighbours = [[(1,dist(start,list_vertices[0][0])), (2,dist(start,list_vertices[0][1])), (3, dist(start,list_vertices[0][2]))],\
                    [(0,dist(list_vertices[0][0],start)),(2,dist(list_vertices[0][0],list_vertices[0][1])),(3,dist(list_vertices[0][0],list_vertices[0][2]))],\
                    [(0,dist(list_vertices[0][1],start)),(1,dist(list_vertices[0][1],list_vertices[0][0])),(3,dist(list_vertices[0][1],list_vertices[0][2])),(4,dist(list_vertices[0][1],list_vertices[1][0])),(7,dist(list_vertices[0][1],list_vertices[1][3]))],\
                    [(0,dist(list_vertices[0][2],start)),(1,dist(list_vertices[0][2],list_vertices[0][0])),(2,dist(list_vertices[0][2],list_vertices[0][1])),(4,dist(list_vertices[0][2],list_vertices[1][0])),(7,dist(list_vertices[0][2],list_vertices[1][3]))],\
                    [(2,dist(list_vertices[1][0],list_vertices[0][1])),(3,dist(list_vertices[1][0],list_vertices[0][2])),(5,dist(list_vertices[1][0],list_vertices[1][1])),(7,dist(list_vertices[1][0],list_vertices[1][3]))],\
                    [(4,dist(list_vertices[1][1],list_vertices[1][0])),(6,dist(list_vertices[1][1],list_vertices[1][2])),(8,dist(list_vertices[1][1],goal))],\
                    [(5,dist(list_vertices[1][2],list_vertices[1][1])),(7,dist(list_vertices[1][2],list_vertices[1][3])),(8,dist(list_vertices[1][2],goal))],\
                    [(2,dist(list_vertices[1][3],list_vertices[0][1])),(3,dist(list_vertices[1][3],list_vertices[0][2])),(4,dist(list_vertices[1][3],list_vertices[1][0])),(6,dist(list_vertices[1][3],list_vertices[1][2])),(8,dist(list_vertices[1][3],goal))],\
                    [(5,dist(goal,list_vertices[1][1])),(6,dist(goal,list_vertices[1][2])),(7,dist(goal,list_vertices[1][3]))]\
                       ]    
print("The shortest path (real graph) is:{}".format(find_shortest_path(list_vertices, list_neighbours, start, goal)))
print("Elapsed time: %s seconds" % (time.time() - start_time))