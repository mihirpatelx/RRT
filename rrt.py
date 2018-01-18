import sys

import random
import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches
import numpy as np
import math

'''
Set up matplotlib to create a plot with an empty square
'''
def setupPlot():
    fig = plt.figure(num=None, figsize=(5, 5), dpi=120, facecolor='w', edgecolor='k')
    plt.autoscale(False)
    plt.axis('off')
    ax = fig.add_subplot(1,1,1)
    ax.set_axis_off()
    ax.add_patch(patches.Rectangle(
        (0,0),   # (x,y)
        1,          # width
        1,          # height
        fill=False
        ))
    return fig, ax

'''
Make a patch for a single pology 
'''
def createPolygonPatch(polygon, color):
    verts = []
    codes= []
    for v in range(0, len(polygon)):
        xy = polygon[v]
        verts.append((xy[0]/10., xy[1]/10.))
        if v == 0:
            codes.append(Path.MOVETO)
        else:
            codes.append(Path.LINETO)
    verts.append(verts[0])
    codes.append(Path.CLOSEPOLY)
    path = Path(verts, codes)
    patch = patches.PathPatch(path, facecolor=color, lw=1)

    return patch
    

'''
Render the problem  
'''
def drawProblem(robotStart, robotGoal, polygons):
    fig, ax = setupPlot()
    patch = createPolygonPatch(robotStart, 'green')
    ax.add_patch(patch)    
    patch = createPolygonPatch(robotGoal, 'red')
    ax.add_patch(patch)
    for p in range(0, len(polygons)):
        patch = createPolygonPatch(polygons[p], 'gray')
        ax.add_patch(patch)    
    plt.show()

'''
Compute a point's distance to other vertices in the tree
'''
def computeDistances(newPoints, count):
    # data structure to hold the distances from each point in list to the other vertices
    # already in the tree
    distances = dict()
    for j in range(1, len(newPoints)):
        x1, y1 = newPoints[j]
        x2, y2 = newPoints[count-1]
        distances[j] = math.sqrt(((x2 - x1) ** 2) + ((y2 - y1) ** 2))

    return distances

'''
Find the smallest numerical element in a list
'''
def findSmallest(distances):
    dist1 = distances[1]
    index = 1
    for i in range(2, len(distances) + 1):
        if(distances[i] < dist1):
            dist1 = distances[i]
            index = i
        i = i + 1
    return dist1, index

'''
Convert adjacency linked list to a tree
'''
def makeTree(adjListMap, root, visited, alreadyVisited):
    while visited:
        count2 = 0
        list = adjListMap[root]
        if not list:
            alreadyVisited.append(visited.pop())
            root = visited[len(visited) - 1]
            continue
        for i in range(0, len(alreadyVisited)):
            if alreadyVisited[i] in list:
                count2 = 1
                break
            i = i + 1
        if count2 == 0:
            if list:
                for i in range(0, len(list)):
                    visited.append(list[i])
                    if root in adjListMap[visited[len(visited)-1]]:
                        adjListMap[visited[len(visited)-1]].remove(root)
                    i = i + 1
        elif count2 == 1:
            alreadyVisited.append(visited.pop())
        if visited:
            root = visited[len(visited)-1]
    return adjListMap

'''
Grow a simple RRT 
'''
def growSimpleRRT(points):
    count = 1 # Keeps track of next empty spot in adjListMap
    newPoints = dict()
    adjListMap = dict()

    # Loop through the points in points dictionary and add to the tree
    for i in range(1, len(points) + 1):

        # Add point to the list of new points
        newPoints[count] = points[i]
        count = count + 1

        # If there is only one point in the list of new points, continue
        if(len(newPoints) == 1):
            i = i + 1
            continue

        # If there is only two points in the list of new points, connect those two points by adding them
        # to the adjListMap dict() and continue
        if(len(newPoints) == 2):
            adjx, adjy = newPoints[2]
            # Vertex 2 is the root's child
            adjListMap[1] = [2]
            adjx, adjy = newPoints[1]
            adjListMap[2] = [1]
            i = i + 1
            continue

        # If there are more than two points in the list of vertices (newPoints), compute the point's distance
        # to each of the other vertices and add to distances dict()
        distances = computeDistances(newPoints, count)

        # Find the smallest distance in the list
        dist, index = findSmallest(distances)

        # Find all adjacent vertices to newPoints[index]
        adjVertices = adjListMap[index]

        newDist = None
        point = None
        adjEdge = None
        for j in range(0, len(adjVertices)):
            # find slope between newPoints[index] and all of it's adjacent edges
            x1, y1 = newPoints[adjVertices[j]]
            x2, y2 = newPoints[index]
            xp, yp = newPoints[count-1]

            c = math.sqrt(((x1 - x2) ** 2) + ((y1 - y2) ** 2))

            a = math.sqrt(((xp - x1) ** 2) + ((yp - y1) ** 2))

            # Check angle
            angle = math.degrees(math.acos((a**2 - dist**2 - c**2)/(-2.0 * c * dist)))

            if(angle >= 90):
                j = j + 1
                continue

            m = (y2-y1)/(x2-x1)

            # Find point on line from (x1, y1) to (x2, y2) that is closest to the point (xp, yp)
            x = (((m*x1)-y1+yp+((1/m)*xp))*m)/(m ** 2 + 1)
            y = m*x - m*x1 + y1
            
            dist2 = math.sqrt(((x - xp) ** 2) + ((y - yp) ** 2))

            if(dist2 < dist):
                newDist = dist2
                point = x, y
                adjEdge = adjVertices[j]
            j = j + 1
        if(newDist == None):
            adjListMap[index].append(count-1)
            if(len(adjListMap) < count-1):
                adjListMap[count-1] = [index]
            else:
                adjListMap[count-1].append(index)
        else:
            newPoints[count] = point
            adjListMap[index].append(count)
            adjListMap[index].remove(adjEdge)
            adjListMap[adjEdge].remove(index)
            adjListMap[adjEdge].append(count)
            count = count + 1
            adjListMap[count-1] = [index, adjEdge, count-2]
            adjListMap[count-2] = [count-1]


        i = i + 1

    return newPoints, adjListMap

'''
Perform basic search 
'''
def basicSearch(tree, start, goal):
    alreadyVisited = []
    visited = []
    visited.append(1)
    tree = makeTree(tree, 1, visited, alreadyVisited)
    
    tree = {k: v for k, v in tree.iteritems() if v and v[0]}
    # queue of paths
    queue = [];
    # push the first path into the queue
    queue.append([start])
    while queue:
        # get the first path from the queue
        path = queue.pop(0);
        
        node = path[-1];
        # path found
        if node == goal:
            return path;
        for neighbor in tree.get(node, []):
            newPath = list(path);
            newPath.append(neighbor);
            queue.append(newPath);

def drawVertices(vertices , ax):
    x = []
    y = []
    for v in vertices:
        x.append(vertices[v][0]/10.0)
        y.append(vertices[v][1]/10.0)
    ax.plot(x , y , 'ko')

def drawRoadMap(points, tree, sign, ax):
    for k, v in tree.iteritems():

        p1 = list(points[k])
        p1 = list(map(lambda x: x / 10.0, p1))
        
        for elem in v:
            p2 = points[elem]
            p2 = list(map(lambda x: x / 10.0, p2))
            x = []
            y = []
            x.append(p1[0])
            x.append(p2[0])
            y.append(p1[1])
            y.append(p2[1])
            ax.plot(x, y, sign, lw = 2)

def drawPath(vertices, path, ax):
    x = []
    y = []
    for vert in path:
        vert_point = vertices[vert]
        vert_point = list(map(lambda x: x /10.0 , vert_point))
        x.append(vert_point[0])
        y.append(vert_point[1])
    ax.plot(x , y , 'orange', lw = 2)


'''
Display the RRT and Path
'''
def displayRRTandPath(points, tree, path=None, robotStart = None, robotGoal = None, polygons = None):
    fig, ax = setupPlot()
    drawVertices(points, ax)
    sign = "k"

    drawRoadMap(points, tree, sign, ax)

    drawPath(points, path, ax)

    plt.show()

def check_point_in_poly(p, poly):
    poly_cross_count = 0
    for p_1_idx in xrange(0,len(poly)):
        p_1 = poly[p_1_idx]
        p_2 = poly[(p_1_idx + 1) % len(poly)]
        if(p[1] >  min(p_1[1] , p_2[1]) and
           p[1] <= max(p_1[1] , p_2[1]) and
           p[0] <= max(p_1[0] , p_2[0]) and
           p_1[1] != p_2[1]):
            interval = (p[1] - p_1[1]) * (p_2[0] - p_1[0]) / (p_2[1] - p_1[1]) + p_1[0];
            if( p_1[0] == p_2[0] or p[0] <= interval):
                poly_cross_count += 1

    poly_cross_count = poly_cross_count % 2
    # if even crossings, then point not in poly
    if(poly_cross_count): # even hence outside poly
        return True
    else:
        return False

"""
    return True if point in polys
    """
def check_point_in_polys(p , polygons):
    for poly in polygons:
        if(check_point_in_poly(p ,poly)):
            return True

    return False
'''
    Collision checking
    Based off of : https://rosettacode.org/wiki/Ray-casting_algorithm
    return True is no collisions occur else false
    '''
def isCollisionFree(robot, point, obstacles):
    # Check if the point lies within any of the polygons
    # within the obstacles.
    if(check_point_in_polys(point , obstacles)):
        return False
    
    translated_robot = []
    for p in robot:
        translated_robot.append((p[0] + point[0] , p[1] + point[1]))

    for rp in translated_robot:
        if(check_point_in_polys(rp , obstacles)):
            return False
        
        if(rp[0] <= 0 or rp[0] >= 10 and rp[1] <= 0 or rp[1] >= 10):
            return False

    return True

'''
compute distance from new vertex to all others in the tree
'''
def computeDistances1(newPoints):
    # data structure to hold the distances from each point in list to the other vertices
    # already in the tree
    distances = dict()
    for j in range(1, len(newPoints)):
        x1, y1 = newPoints[j]

        x2, y2 = newPoints[len(newPoints)]

        distances[j] = math.sqrt(((x2 - x1) ** 2) + ((y2 - y1) ** 2))
    
    return distances

def collisonDetection(point1, point2, robot, obstacles):
    print "hi"
    inside = False
    x1 = point1[0]
    y1 = point1[1]
    x2 = point2[0]
    y2 = point2[1]
    if not isCollisionFree(robot, (x1, y1), obstacles):
        return False
    dist = math.sqrt(((x2 - x1) ** 2) + ((y2 - y1) ** 2))
    inc = dist/25
    while((x1, y1) != (x2,y2)):
        print "x2, y2"
        print x2, y2
        print x1, y1
        x1 = x1 - ((inc * (x1-x2))/dist)
        y1 = y1 - ((inc * (y1-y2))/dist)
        print "x1, y1"
        print x1, y1
        print
        if not isCollisionFree(robot, (x1, y1), obstacles):
            inside = True
            break
        if np.isclose(x1, x2):
            break
    if inside == True:
        return False
    return True

def generatePoint():
    point = (random.uniform(0.0, 10.0), random.uniform(0.0, 10.0))
    return point

def getEdge(points, tree, adjVertices, dist, index):
    newDist = None
    newPoint = None
    adjEdge = None
    for j in range(0, len(adjVertices)):
        x1, y1 = points[adjVertices[j]]
        x2, y2 = points[index] # closest vertex
        xp, yp = points[len(points)] # new vertex
        
        c = math.sqrt(((x1 - x2) ** 2) + ((y1 - y2) ** 2))
        
        
        a = math.sqrt(((xp - x1) ** 2) + ((yp - y1) ** 2))
        
        # Check angle
        angle = math.degrees(math.acos((a**2 - dist**2 - c**2)/(-2.0 * c * dist)))
        if(angle >= 90):
            j = j + 1
            continue
        
        m = (y2-y1)/(x2-x1)
        # Find point on line from (x1, y1) to (x2, y2) that is closest to the point (xp, yp)
        x = (((m*x1)-y1+yp+((1/m)*xp))*m)/(m ** 2 + 1)
        y = m*x - m*x1 + y1
        dist2 = math.sqrt(((x - xp) ** 2) + ((y - yp) ** 2))
        if(dist2 < dist):
            newDist = dist2
            newPoint = x, y
            adjEdge = adjVertices[j]
        j = j + 1
    if(newDist == None):
        #        if collisonDetection(points[len(points)], points[index], robot, obstacles) == False:
        #            points.pop(1)
        tree[index].append(len(points))
        if(len(tree) < len(points)):
            tree[len(points)] = [index]
        else:
            tree[len(points)].append(index)
    else:
    #        if collisonDetection(points[len(points)], points[index], robot, obstacles) == False:
    #            print "hello"
    #            print points
    #            points.pop(1)
    #            print
    #            print points
        points[len(points)+1] = newPoint
        tree[index].append(len(points))
        tree[index].remove(adjEdge)
        tree[adjEdge].remove(index)
        tree[adjEdge].append(len(points))
        tree[len(points)] = [index, adjEdge, len(points)-1]
        tree[len(points)-1] = [len(points)]

        return points, tree

'''
add new vertex to tree
'''
def addToTree(tree, points, point, robot, obstacles):

    if(len(points) == 1):
        #while not collisonDetection(point, points[1], robot, obstacles):
        point = generatePoint()
        points[2] = point
        tree[1] = [2]
        tree[2] = [1]
        return tree, points

'''
The full RRT algorithm
'''
def RRT(robot, obstacles, startPoint, goalPoint):
    points = dict()
    tree = dict()
    path = []
    points[1] = startPoint
    point = []
    goal = False
    for i in range(0, 1):
        while not isCollisionFree(robot, point, obstacles):
            point = generatePoint()
        tree, points = addToTree(tree, points, point, robot, obstacles)
        if tree == None:
            print "hello"
        i = i + 1
    return points, tree, path

if __name__ == "__main__":
    
    # Retrive file name for input data
    if(len(sys.argv) < 6):
        print "Five arguments required: python spr.py [env-file] [x1] [y1] [x2] [y2]"
        exit()
    
    filename = sys.argv[1]
    x1 = float(sys.argv[2])
    y1 = float(sys.argv[3])
    x2 = float(sys.argv[4])
    y2 = float(sys.argv[5])

    # Read data and parse polygons
    lines = [line.rstrip('\n') for line in open(filename)]
    robot = []
    obstacles = []
    for line in range(0, len(lines)):
        xys = lines[line].split(';')
        polygon = []
        for p in range(0, len(xys)):
            xy = xys[p].split(',')
            polygon.append((float(xy[0]), float(xy[1])))
        if line == 0 :
            robot = polygon
        else:
            obstacles.append(polygon)

    # Print out the data
    print "Robot:"
    print str(robot)
    print "Pologonal obstacles:"
    for p in range(0, len(obstacles)):
        print str(obstacles[p])
    print ""

    # Visualize
    robotStart = []
    robotGoal = []

    def start((x,y)):
        return (x+x1, y+y1)
    def goal((x,y)):
        return (x+x2, y+y2)
    robotStart = map(start, robot)
    robotGoal = map(goal, robot)
    drawProblem(robotStart, robotGoal, obstacles)

    # Example points for calling growSimpleRRT
    # You should expect many mroe points, e.g., 200-500
    points = dict()
    points[1] = (5, 5)
    points[2] = (7, 8.2)
    points[3] = (6.5, 5.2)
    points[4] = (0.3, 4)
    points[5] = (6, 3.7)
    points[6] = (9.7, 6.4)
    points[7] = (4.4, 2.8)
    points[8] = (9.1, 3.1)
    points[9] = (8.1, 6.5)
    points[10] = (0.7, 5.4)
    points[11] = (5.1, 3.9)
    points[12] = (2, 6)
    points[13] = (0.5, 6.7)
    points[14] = (8.3, 2.1)
    points[15] = (7.7, 6.3)
    points[16] = (7.9, 5)
    points[17] = (4.8, 6.1)
    points[18] = (3.2, 9.3)
    points[19] = (7.3, 5.8)
    points[20] = (9, 0.6)

    # Printing the points
    print "" 
    print "The input points are:"
    print str(points)
    print ""
    
    #points, adjListMap = growSimpleRRT(points)

    # Search for a solution
    #path = basicSearch(adjListMap, 1, 20)
    
    # Your visualization code 
    #displayRRTandPath(points, adjListMap, path)

    # Solve a real RRT problem
    points, tree, path = RRT(robot, obstacles, (x1, y1), (x2, y2))
    print points
    # Your visualization code
    #displayRRTandPath(points, adjListMap, robotStart, robotGoal, obstacles)



