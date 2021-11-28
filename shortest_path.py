# -*- coding: utf-8 -*-
"""
Created on Wed Nov 24 13:22:07 2021
Finds the shortest path on a graph.
Inputs: 
    - obstacle_vertices, list of lists of tuples [[(x1,y1),(x2,y2),...,(x_k_,y_k)],...,[(xx1,yy1),(xx2,yy2),...,(xx_n,yy_n)]]. 
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

def find_shortest_path(obstacle_vertices, list_neighbours, start=("unspecified","unspecified"), goal=("unspecified","unspecified")):
    """
    """
    
    
    #%%Initialisation
    import math
    
    #attribute a name to each vertex: start (vertex 0), obstacle vertices (vertices 1...N), and goal (vertex N+1)
    obstacle_vertices_flattened = [item for sublist in obstacle_vertices for item in sublist]
    N = len(obstacle_vertices_flattened)                                            
    name_vertices = list(range(N+2))
    
    #if A* is used (goal is specified): calculate the heuristic as euclidean distance. Otherwise, leave =0 for all vertices
    heuristic = [0]*(N+2)
    dist = lambda p1,p2: math.sqrt((p1[1]-p2[1])**2+(p1[0]-p2[0])**2)
    if goal!=("unspecified","unspecified"):
        heuristic = [dist(goal,vertex) for vertex in obstacle_vertices_flattened]
        heuristic.append(0)
        heuristic.insert(0,dist(goal,start))

    #attribute initial weights to each vertex
    infinity = 100000
    weights_djikstra = [infinity]*(N+2)                                         # minimal distance to reach each vertex from start. Before exploration, =infinity
    weights_astar = weights_djikstra.copy()                                     # weights_astar = weights_djikstra + heuristic
    weights_djikstra[0] = 0
    weights_astar[0] = heuristic[0]
    
    #initialize vertex of predecessors, used to trace back the best path
    predecessors = ["unknown"]*(N+2)                                            #best predecessor to reach each vertex
    predecessors[0] = "reached_start"
    
    #list of vertices to be explored
    unexplored_vertices = name_vertices.copy()
    unexplored_vertices.remove(N+1)                                                #we don't need to explore the goal
    
    
    #%% Applying Djikstra or A*
    while unexplored_vertices:                                                     #while there are still vertices to explore (list non-empty)
        weights_unexplored = []
        for vertex in unexplored_vertices:
            weights_unexplored.append(weights_astar[vertex])
        
        #set the current vertex to the one with lowest weight
        current_vertex = unexplored_vertices[weights_unexplored.index(min(weights_unexplored))]
        current_neighbours = list_neighbours[current_vertex]                    #get its neighbours' names and their associated distance (name,dist)
        
        #if needed, update the weight of neighbours and set current_vertex as their best predecessor
        for ngb in current_neighbours:
            ngb_name = ngb[0]
            ngb_dist = ngb[1]
            
            weight_candidate = weights_djikstra[current_vertex] + ngb_dist + heuristic[ngb_name]
            if weight_candidate<weights_astar[ngb_name]:
                weights_astar[ngb_name] = weight_candidate
                weights_djikstra[ngb_name] = weights_djikstra[current_vertex] + ngb_dist
                predecessors[ngb_name] = current_vertex
                
                #if this neighbour was already explored, allow to re-explore it to update weights
                if ngb_name not in unexplored_vertices:
                    unexplored_vertices.append(ngb_name)
        
        #remove the current vertex from unexplored vertices
        unexplored_vertices.remove(current_vertex)
        
        #if A* is used, exit the loop once the goal is reached (no need to explore the entire graph)
        current_neighbours_flat = [x for sublist in current_neighbours for x in sublist]
        name_current_neighbours = current_neighbours_flat[0::2]                 #extract the name of neighbours
        if goal!=("unspecified","unspecified") and (name_vertices[-1] in name_current_neighbours):                
            #print("Unexplored nodes: {}".format(unexplored_vertices))
            break
        
    
    #%% Retrace the best path going backwards (origin = where we came from. predecessor = best origin for a given vertex)
    best_path = []
    origin = predecessors[-1]                                                   #predecessor of goal
    while origin!="reached_start":
        best_path.insert(0,origin)                                              #insert predecessor of vertex i at the beginning of best_path
        origin = predecessors[origin]                                           #for next loop: change the value of "origin"
    best_path.append(name_vertices[-1])                                         #add vertex "goal" to the best_path

    return best_path

#---------------------------------------------------------
#%% Test
import sys
import time
import math

sys.path.insert(1, 'Global_navigation')
import global_path

list_vertices = [[(10,10),(20,10),(15,18.66)],\
                  [(30,25),(40,15),(53.66,18.66),(57.32,32.32),(47.32,42.32),(33.66,38.66)]]
start = (0,0)
goal = (70,60)
start_time = time.time()
list_neighbours = global_path.find_all_paths(list_vertices, start, goal)

print("The shortest path is:{}".format(find_shortest_path(list_vertices, list_neighbours,start, goal)))
print("Elapsed time: %s seconds" % (time.time() - start_time))


# start_time = time.time()
# obstacle_vertices = [[(2,1),(2,3),(3,1)],[(4,6),(6,8),(8,6),(8,4)]] 
# start = (0,0)
# goal = (10,10)

# dist = lambda p1,p2: math.sqrt((p1[1]-p2[1])**2+(p1[0]-p2[0])**2)
# list_neighbours = [[(1,dist(start,obstacle_vertices[0][0])), (2,dist(start,obstacle_vertices[0][1])), (3, dist(start,obstacle_vertices[0][2]))],\
#                     [(0,dist(obstacle_vertices[0][0],start)),(2,dist(obstacle_vertices[0][0],obstacle_vertices[0][1])),(3,dist(obstacle_vertices[0][0],obstacle_vertices[0][2]))],\
#                     [(0,dist(obstacle_vertices[0][1],start)),(1,dist(obstacle_vertices[0][1],obstacle_vertices[0][0])),(3,dist(obstacle_vertices[0][1],obstacle_vertices[0][2])),(4,dist(obstacle_vertices[0][1],obstacle_vertices[1][0])),(7,dist(obstacle_vertices[0][1],obstacle_vertices[1][3]))],\
#                     [(0,dist(obstacle_vertices[0][2],start)),(1,dist(obstacle_vertices[0][2],obstacle_vertices[0][0])),(2,dist(obstacle_vertices[0][2],obstacle_vertices[0][1])),(4,dist(obstacle_vertices[0][2],obstacle_vertices[1][0])),(7,dist(obstacle_vertices[0][2],obstacle_vertices[1][3]))],\
#                     [(2,dist(obstacle_vertices[1][0],obstacle_vertices[0][1])),(3,dist(obstacle_vertices[1][0],obstacle_vertices[0][2])),(5,dist(obstacle_vertices[1][0],obstacle_vertices[1][1])),(7,dist(obstacle_vertices[1][0],obstacle_vertices[1][3]))],\
#                     [(4,dist(obstacle_vertices[1][1],obstacle_vertices[1][0])),(6,dist(obstacle_vertices[1][1],obstacle_vertices[1][2])),(8,dist(obstacle_vertices[1][1],goal))],\
#                     [(5,dist(obstacle_vertices[1][2],obstacle_vertices[1][1])),(7,dist(obstacle_vertices[1][2],obstacle_vertices[1][3])),(8,dist(obstacle_vertices[1][2],goal))],\
#                     [(2,dist(obstacle_vertices[1][3],obstacle_vertices[0][1])),(3,dist(obstacle_vertices[1][3],obstacle_vertices[0][2])),(4,dist(obstacle_vertices[1][3],obstacle_vertices[1][0])),(6,dist(obstacle_vertices[1][3],obstacle_vertices[1][2])),(8,dist(obstacle_vertices[1][3],goal))],\
#                     [(5,dist(goal,obstacle_vertices[1][1])),(6,dist(goal,obstacle_vertices[1][2])),(7,dist(goal,obstacle_vertices[1][3]))]\
#                        ]    
# print("The shortest path (real graph) is:{}".format(find_shortest_path(obstacle_vertices, list_neighbours, start, goal)))
# print("Elapsed time: %s seconds" % (time.time() - start_time))