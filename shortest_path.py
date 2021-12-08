# -*- coding: utf-8 -*-
"""
Created on Wed Nov 24 13:22:07 2021

Main function which can be run in other scripts: find_shortest_path
Finds the shortest path on a graph, given the position of the points in the graph.
Inputs: 
    - obstacle_vertices, list of lists of tuples [[(x1,y1),(x2,y2),...,(x_k_,y_k)],...,[(xx1,yy1),(xx2,yy2),...,(xx_n,yy_n)]]. 
    Each embedded list corresponds to an obstacle.
    Each tuple (x,y) contains the coordinates of a vertex of the obstacle.

    - list_neighbours, list of lists of tuples [[(point,distance),(point,distance),...],...,[(point,distance),(point,distance),...,(point,distance)]]
    Each embedded list corresponds to the neighbours of a given point.
    Each tuple (point,distance) contains the name of the neighbour and its distance from that point.
    
    - goal and start: coordinates (x,y) of the goal and start. If not specified, a Dijkstra algorithm will be used ; 
    otherwise, A* with Euclidean distance will be used.

Output:
    names_full_path, ordered list that contains the optimal path for the graph. 
    Example: [0,1,3,5,6,7,8] where 0 is start and 8 is goal
"""

import sys
import time
import math
sys.path.insert(1, 'Global_navigation')
import global_path



INFINITY = 100000

def find_shortest_path(obstacle_vertices, start=("unspecified","unspecified"), goal=("unspecified","unspecified")):
    # 1. Initialisation
    #find all neighbours
    list_neighbours = global_path.find_all_paths(obstacle_vertices, start, goal)

    #attribute a name to each point: start (point 0), obstacle vertices (vertices 1...N), and goal (point N+1)
    obstacle_vertices_flattened = [vertex for sublist in obstacle_vertices for vertex in sublist]
    nb_vertices = len(obstacle_vertices_flattened)                                            
    name_points = list(range(nb_vertices+2))

    #attribute heuristics to each point (Euclidean distance if A*, 0 otherwise)
    heuristic = get_heuristics(nb_vertices, obstacle_vertices_flattened, start, goal)

    #attribute initial weights to each point
    weights_Dijkstra = [INFINITY]*(nb_vertices+2)      # min distance to reach each point from start. Before exploration, =infinity
    weights_total = weights_Dijkstra.copy()  # weights_total = weights_Dijkstra + heuristic
    weights_Dijkstra[0] = 0
    weights_total[0] = heuristic[0]
    
    #initialize list of predecessors, used to trace back the best path
    predecessors = ["unknown"]*(nb_vertices+2)
    predecessors[0] = "reached_start"
    
    #list of points to be explored
    points_to_explore = name_points.copy()
    points_to_explore.remove(nb_vertices+1)             #no need to explore the goal
    
    
    
    # 2. Apply Dijkstra or A*
    explore_graph(points_to_explore, name_points, list_neighbours,\
                  weights_total, weights_Dijkstra, heuristic,\
                      predecessors, goal)
    
        
        
    # 3. Retrace the best path going backwards 
    #(origin = where we came from. predecessor = best origin for a given point)
    names_full_path = []
    print("test")
    print("predecessors {}".format(predecessors))
    origin = predecessors[-1]                      #predecessor of goal
    while origin!="reached_start":
        print(origin)
        names_full_path.insert(0,origin)           #insert predecessor of point i
        origin = predecessors[origin]              #for next loop: change the value of "origin"
    names_full_path.append(name_points[-1])      #add point "goal" to the names_full_path


    return names_full_path



#%%
def get_heuristics(nb_vertices, obstacle_vertices_flattened, start, goal):
    """Assigns a heuristic to each point. 
    If the goal is not specified, then standard Dijkstra is applied (heuristic=0).
    Otherwise, the heuristic is the euclidean distance to the goal.
    
    Inputs: 
        - goal, start: identical to ones given to find_shortest_path
        - N, obstacle_vertices_flattened: obtained from find_shortest_path
        
    Outputs: heuristic for each point [h0, h1, ..., hN]
    """
    heuristic = [0]*(nb_vertices+2)
    dist = lambda p1,p2: math.sqrt((p1[1]-p2[1])**2+(p1[0]-p2[0])**2)
    if goal!=("unspecified","unspecified"):
        heuristic = [dist(goal,vertex) for vertex in obstacle_vertices_flattened]
        heuristic.append(0)
        heuristic.insert(0,dist(goal,start))
        
    return heuristic



#%%
def explore_graph(points_to_explore, name_points, list_neighbours, weights_total, weights_Dijkstra, heuristic, predecessors, goal):
    """" This function applies the Dijkstra (or A*) algorithm:
        at each step, it chooses the point with smallest weight
        and explores from there.
        It does so until reaching the goal (case A*) 
        or until the whole graph is explored (case Dijkstra)
        
        Inputs: 
            - list_points, goal: identical to ones given to find_shortest_path
            - other inputs: taken directly from find_shortest_path
            
        Output: 
            predecessors, list of ideal predecessor for each node [p0,p1,...,pN],
            where each entry is the name of a node
    """
    #the while loop stops when points_to_explore is empty
    while points_to_explore:                                
        weights_unexplored = []
        for point in points_to_explore:
            weights_unexplored.append(weights_total[point])
        
        #set the current point to the one with lowest weight. 
        #get its neighbours' names and their associated distance (name,dist)
        current_point = points_to_explore[weights_unexplored.index(min(weights_unexplored))]
        current_neighbours = list_neighbours[current_point]
        
        #for each neighbour of the current point, calculate its new total weight.
        for ngb in current_neighbours:
            ngb_name = ngb[0]
            ngb_dist = ngb[1]
            
            weight_candidate = weights_Dijkstra[current_point] + ngb_dist + heuristic[ngb_name]
            
            #if this weight is better than previous, 
            #update it and set current_point as best predecessor for this neighbour.
            if weight_candidate<weights_total[ngb_name]:
                weights_total[ngb_name] = weight_candidate
                weights_Dijkstra[ngb_name] = weights_Dijkstra[current_point] + ngb_dist
                predecessors[ngb_name] = current_point
                
                #if this neighbour was already explored, allow to re-explore it to update weights
                if ngb_name not in points_to_explore:
                    points_to_explore.append(ngb_name)
        
        #remove the current point from unexplored points
        points_to_explore.remove(current_point)
        
        
        
        #if A* is used, exit the loop once the goal is reached (no need to explore the entire graph)
        current_neighbours_flat = [x for sublist in current_neighbours for x in sublist]
        name_current_neighbours = current_neighbours_flat[0::2]                 #name of neighbours
        
        if goal!=("unspecified","unspecified")\
            and (name_points[-1] in name_current_neighbours):                
            #print("Unexplored nodes: {}".format(points_to_explore)) #some nodes are unexplored
            break
        
        
    
    return predecessors



#%%
def pathname_to_coords(names_path, list_vertices, start, goal):
    """"Converts the return of shortest_path into coordinates.
    
    Inputs:
        similar as previous functions. names_path is the return from shortest_path.
        
    Output:
        - coords = [(node0.x, node0.y), (nodeK.x,nodeK.y),..., (nodeP.x, nodeP.y), (node(N+1).x, node(N+1).y)]
        where the robot must do node0 --> nodeK --> ...--> nodeP --> node(N+1)
        """
    obstacle_vertices_flattened = [vertex for sublist in list_vertices for vertex in sublist]

    coords = [start]
    for i in names_path[1:-1]:
        coords.append(obstacle_vertices_flattened[i - 1])
    coords.append(goal)

    return coords


#----------------------------
#%% Tests
# list_vertices = [[(10,10),(20,10),(15,18.66)],\
#                   [(30,25),(40,15),(53.66,18.66),(57.32,32.32),(47.32,42.32),(33.66,38.66)],\
#                       [(59.27,27.55),(54.63,70.85),(67.09,69.57),(73.97,36.5)]]
# start = (0,0)
# goal = (75.88,42.09)
# start_time = time.time()
# list_neighbours = global_path.find_all_paths(list_vertices, start, goal)
# res = find_shortest_path(list_vertices, list_neighbours, start, goal)

# print("The shortest path is:{}".format(res))


# x, y, xy = names_to_subpaths(res, list_vertices, start, goal)
# print("x-coordinates of substarts: {}".format(x))
# print("y-coordinates of substarts: {}".format(y))
# print("xy-coordinates of subgoals: {}".format(xy))
# print("Elapsed time: %s seconds" % (time.time() - start_time))
