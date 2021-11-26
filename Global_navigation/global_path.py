import math
from itertools import combinations

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y


#test :

#inputs data and useful data to have

#obstacle1
Sommet1 = Point(1, 1)
Sommet2 = Point(10, 1)
Sommet3 = Point(1, 2)
#obstacle2
Sommet4 = Point(10, 2)
Sommet5 = Point(10, 0)
Sommet6 = Point(0, 10)
Sommet7 = Point(0, 0)
Sommet8 = Point(10, 10)
Sommet9 = Point(10, 10)

#liste des obstacles avant agrandissement
list_vertices = [[Sommet1, Sommet2, Sommet3],[Sommet3, Sommet4, Sommet5, Sommet6, Sommet7, Sommet8, Sommet9]]

#liste des obstacles après agrandissement
list_shifted_vertices = list_vertices*0,1

#liste des cotés des obstacles = matrice B
#--> comment faire à part le faire à la main ?
list_obstacles_sides = [[Sommet1]]

#liste de tous les chemins possibles pour passer d'un sommet à l'autre = matrice A
list_all_path = [i for i in combinations(list_shifted_vertices, 2)]
#--> ça va nous donner quoi, une liste, un array, une matrice ??

# Given three collinear points p, q, r, the function checks if
# point q lies on line segment 'pr'
def onSegment(p, q, r):
    if ((q.x <= max(p.x, r.x)) and (q.x >= min(p.x, r.x)) and
            (q.y <= max(p.y, r.y)) and (q.y >= min(p.y, r.y))):
        return True
    return False


def orientation(p, q, r):
    # to find the orientation of an ordered triplet (p,q,r)
    # function returns the following values:
    # 0 : Collinear points
    # 1 : Clockwise points
    # 2 : Counterclockwise

    # See https://www.geeksforgeeks.org/orientation-3-ordered-points/amp/
    # for details of below formula.

    val = (float(q.y - p.y) * (r.x - q.x)) - (float(q.x - p.x) * (r.y - q.y))
    if (val > 0):

        # Clockwise orientation
        return 1
    elif (val < 0):

        # Counterclockwise orientation
        return 2
    else:

        # Collinear orientation
        return 0


# The main function that returns true if
# the line segment 'p1q1' and 'p2q2' intersect.
def doIntersect(p1, q1, p2, q2):
    # Find the 4 orientations required for
    # the general and special cases
    o1 = orientation(p1, q1, p2)
    o2 = orientation(p1, q1, q2)
    o3 = orientation(p2, q2, p1)
    o4 = orientation(p2, q2, q1)

    # General case
    if ((o1 != o2) and (o3 != o4)):
        return True

    # Special Cases

    # p1 , q1 and p2 are collinear and p2 lies on segment p1q1
    if ((o1 == 0) and onSegment(p1, p2, q1)):
        return True

    # p1 , q1 and q2 are collinear and q2 lies on segment p1q1
    if ((o2 == 0) and onSegment(p1, q2, q1)):
        return True

    # p2 , q2 and p1 are collinear and p1 lies on segment p2q2
    if ((o3 == 0) and onSegment(p2, p1, q2)):
        return True

    # p2 , q2 and q1 are collinear and q1 lies on segment p2q2
    if ((o4 == 0) and onSegment(p2, q1, q2)):
        return True

    # If none of the cases
    return False

#algo
for k in list_all_path:
    #for i = 1 jusqu'à nombre obstacles
        if doIntersect(je sais pas comment prendre toutes les valeurs de la matA et les comparer à celles de la matB)
            print("Yes")
            list_possible_path[[i][k]] = list_all_path[[i][k]]

        else:
            print("No")
            list_possible_path[[i][k]] = 666-666

    #list_possible_path = matrice C

for i in list_possible_path
    list_distance_path = math.dist(list_possible_path[i])
    #list_distance_path = matrice D



# Driver program to test above functions:
# p1 = Point(1, 1)
# q1 = Point(10, 1)
# p2 = Point(1, 2)
# q2 = Point(10, 2)
#
# if doIntersect(p1, q1, p2, q2):
#     print("Yes")
# else:
#     print("No")
#
# p1 = Point(10, 0)
# q1 = Point(0, 10)
# p2 = Point(0, 0)
# q2 = Point(10, 10)
#
# if doIntersect(p1, q1, p2, q2):
#     print("Yes")
# else:
#     print("No")
#
# p1 = Point(-5, -5)
# q1 = Point(0, 0)
# p2 = Point(1, 1)
# q2 = Point(10, 10)
#
# if doIntersect(p1, q1, p2, q2):
#     print("Yes")
# else:
#     print("No")
#
# # This code is contributed by Ansh Riyal
# #https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/?ref=lbp
