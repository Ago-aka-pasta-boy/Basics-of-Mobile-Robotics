import sys
import math
sys.path.insert(1, 'Global_navigation')
import global_path

INFINITY = 100000

def find_shortest_path(obstacle_vertices, start=("unspecified","unspecified"),\
                       goal=("unspecified","unspecified")):
    """
    ---
    Description: Finds the shortest path 
                 Relies on Dijkstra or A* (heuristic: Euclidean distance to 
                 goal) algorithm.
    ---
    Inputs:
        - obstacle_vertices:
        [[(x1,y1),...,(x_k_,y_k)],...,[(xx1,yy1),(xx2,yy2),...,(xx_n,yy_n)]] ???
        Each embedded list corresponds to an obstacle.
        Each tuple (x,y) contains the coordinates of an obstacle's vertex.
    
        - goal and start: coordinates (x,y) of the goal and start.
            If not specified, a Dijkstra algorithm will be used ; 
            if specified, A* algorithm will be used.
    ---
    Output:
        - names_full_path: 
            ordered list that contains the optimal path for the graph.
            Example: [0,3,1,5,7,6,8] where 0 is start and 8 is goal.
    """
    
    # 1. Initialisation
    #obtain the visibility graph
    list_neighbours = \
        global_path.find_all_paths(obstacle_vertices, start, goal)

    #give a name to each point: 
    #start (point 0), obstacle vertices (vertices 1...N), and goal (point N+1)
    obstacle_vertices_flattened = \
        [vertex for sublist in obstacle_vertices for vertex in sublist]
    nb_vertices = len(obstacle_vertices_flattened)                                            
    name_points = list(range(nb_vertices + 2))

    #give heuristics and initial weights to each point
    #weights_Dijkstra = min. distance to reach a point from start
    #weights_total = weights_Dijkstra + heuristic
    heuristic = \
        get_heuristics(nb_vertices, obstacle_vertices_flattened, start, goal)
    weights_Dijkstra = [INFINITY]*(nb_vertices + 2)
    weights_total = weights_Dijkstra.copy()
    weights_Dijkstra[0] = 0
    weights_total[0] = heuristic[0]
    
    #initialize list of predecessors, used to trace back the best path
    predecessors = ["unknown"]*(nb_vertices + 2)
    predecessors[0] = "reached_start"
    
    points_to_explore = name_points.copy()
    points_to_explore.remove(nb_vertices + 1)   #do not explore the goal
    
    
    # 2. Graph search
    predecessors = explore_graph(points_to_explore, name_points,\
                   list_neighbours, weights_total, weights_Dijkstra,\
                   heuristic, predecessors, goal)
    
        
        
    # 3. Retrace the best path going backwards (goal to start)
    #origin = where we came from
    #predecessor = best origin for a given point
    names_full_path = []
    origin = predecessors[-1]
    while origin!="reached_start":
        names_full_path.insert(0, origin)
        origin = predecessors[origin]
    names_full_path.append(name_points[-1])     #add the goal


    return names_full_path



#%%
def get_heuristics(nb_vertices, obstacle_vertices_flattened, start, goal):
    """
    ---
    Description: Assigns a heuristic to each point.
                If the goal is not specified, 
                standard Dijkstra is applied (heuristic=0).
                If the goal is specified, the heuristic is the Euclidean 
                distance to the goal.
                
    ---
    Inputs: - nb_vertices: number of obstacle's vertices
            - obstacle_vertices_flattened: list of obstacle's vertices,
              NOT grouped by obstacle
            - goal, start: coordinates (x,y) of the goal and start
            
    ---
    Output: - heuristic for each point [h0, h1, ..., hN,0]
    """
    heuristic = [0]*(nb_vertices + 2)
    dist = lambda p1,p2: math.sqrt((p1[1] - p2[1])**2 + (p1[0] - p2[0])**2)
    if goal!=("unspecified","unspecified"):
        heuristic = \
            [dist(goal,vertex) for vertex in obstacle_vertices_flattened]
        heuristic.append(0)
        heuristic.insert(0, dist(goal,start))
        
    return heuristic



#%%
def explore_graph(points_to_explore, name_points, list_neighbours, \
                  weights_total, weights_Dijkstra, heuristic, predecessors, \
                  goal):
    """" 
    ---
    Description: Applies the Dijkstra or A* algorithm:
                at each step, it chooses the point with smallest weight
                and explores from there.
                It does so until reaching the goal (case A*) 
                or until the whole graph is explored (case Dijkstra)
    ---
    
    Inputs: - points_to_explore: list of all points' names except goal
                                (elements will be removed from it) ???
            - name_points: ordered list of all points' names
                           (this list will stay intact)???
            - list_neighbours: list with the name and distance of all 
                                points' neighbours
            - weights_Dijkstra: min. distance to reach points from the start
            - weights_total = weights_Dijkstra + heuristic
            - heuristic: heuristic of each points (0 if Dijkstra is used,
                        Euclidean distance to goal if A* is used)
            - predecessors: ["reached_goal", "unknown", ..., "unknown"]
                        will be filled to become the output
            - goal: coordinates (x,y) of the goal
    ---
    
    Output: - predecessors: list of ideal predecessor for each point 
            ["reached_goal",p1,...,pN, p(N+1)]    
    """
    #stops when points_to_explore is empty
    while points_to_explore:                                
        weights_unexplored = []
        for point in points_to_explore:
            weights_unexplored.append(weights_total[point])
        
        #explore the point with lowest weight 
        current_point = points_to_explore\
            [weights_unexplored.index(min(weights_unexplored))]
        current_neighbours = list_neighbours[current_point]
        
        
        for ngb in current_neighbours:
            ngb_name = ngb[0]
            ngb_dist = ngb[1]
            
            weight_candidate = \
               weights_Dijkstra[current_point] + ngb_dist + heuristic[ngb_name]
            
            #if this weight is better than previous, update it
            #and set current_point as best predecessor for this neighbour.
            if weight_candidate<weights_total[ngb_name]:
                weights_total[ngb_name] = weight_candidate
                weights_Dijkstra[ngb_name] = weights_Dijkstra[current_point]\
                                            + ngb_dist
                predecessors[ngb_name] = current_point
                
                #if this neighbour was already explored, 
                #allow to re-explore it to update weights
                if ngb_name not in points_to_explore:
                    points_to_explore.append(ngb_name)
        
        points_to_explore.remove(current_point)
        
        
        #if A* is used, exit the loop once the goal is reached 
        #(no need to explore the entire graph)
        current_neighbours_flat = \
            [x for sublist in current_neighbours for x in sublist]
            
        #neighbours = (name,dist) --> only take even values
        name_current_neighbours = current_neighbours_flat[0::2]
        
        if goal!=("unspecified","unspecified")\
            and (name_points[-1] in name_current_neighbours):                
            break
        
        
    return predecessors



#%%
def pathname_to_coords(names_path, list_vertices, start, goal):
    """"
    ---
    Description: converts points' names into points' coordinates
    ---
    Inputs: - names_path: ordered list that contains the optimal path
                for the graph, stored as names.
                Example: [0,3,1,5,7,6,8] where 0 is start and 8 is goal.
            - list_vertices: coordinates of obstacle vertices, sorted
                by obstacle
            - start, goal: (x,y) coordinates of the start and goal
        
    ---
    Output: - coords: list of coordinates of points along the path.
                Example: [(10,15), (20,30),..., (50,50)]
                where start = (10,15) and goal = (50,50)
    """
    obstacle_vertices_flattened = \
        [vertex for sublist in list_vertices for vertex in sublist]

    coords = [start]
    for i in names_path[1:-1]:
        coords.append(obstacle_vertices_flattened[i - 1])
    coords.append(goal)

    return coords