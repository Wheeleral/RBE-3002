#!/usr/bin/env python
import heapq


class Node():
    def __init__(self, parent, pos, g, h):
        self.parent = parent  # for path
        self.pos = pos  # position of the node
        self.g = g  # how far from start
        self.h = h  # manhatten distance to goal
        self.f = g + h  # sum

    # find neighbors and determine if they are open or if they are obstacle
    def getNeighbors(self, gridCells, width, end, cost):
        x = self.pos[0]
        y = self.pos[1]

        neighbors = []
        neighbors.append(self._validate(x + 1, y, gridCells, width, end, cost))  # check for valid neighbors
        neighbors.append(self._validate(x - 1, y, gridCells, width, end, cost))
        neighbors.append(self._validate(x, y + 1, gridCells, width, end, cost))
        neighbors.append(self._validate(x, y - 1, gridCells, width, end, cost))
        return neighbors  # return all neighbors that aren't obstacles

    def _validate(self, x, y, gridCells, width, end, cost):  # check if wall
        print ("In Validate: ", x, y, "Width" ,width)

        if (gridCells[y * width + x] < 50):  # convert from x,y to grid cell number
            cellCost = 0
            #for item in cost:
            #    if (item.x == x and item.y == y):  # if the items are in the cost map, then assign the cost value
            #        cellCost = item.total
            #cellCost=cost[y*width+x]
            if (cellCost != 0):
                return Node(self, (x, y), self.g + cellCost, manhattan((x, y), end))  # creates new node using cost map
            else:
                return Node(self, (x, y), self.g + 1, manhattan((x, y), end))  # create new node using default
        else:
            return None


def manhattan(p1, p2):  # use manhattan distance to get dis to goal
    return abs(p1[0] - p2[0]) + abs(p2[1] - p2[1])


def AStar(start, end, gridCells, width, cost):  # calculate the distance THIS IS THE MAIN FUNCTION

    # setup frontier nodes using a min heap
    frontier = []
    startNode = Node(None, start, 0, manhattan(start, end))  # parent, position, g, h
    heapq.heappush(frontier, (startNode.f, startNode))  # puts items into min heap

    visited = {}  # create dictionary called "visited"

    while (len(frontier) > 0):  # if no solution, exit the while loop
        cur = heapq.heappop(frontier)[1]  # The current node is the shortest distance
        if (cur.pos in visited):  # don't pursue paths that have already been searched
            continue
        if (cur.pos == end):  # quit condition
            path = tracePath(start, cur, visited)  # probably trace path && return something useful
            waypt, dir = dirPath(path)
           #print ('Directions:', dir)
            return (path, waypt, dir)
        for neighbor in cur.getNeighbors(gridCells, width, end, cost):

            if (neighbor != None):
                heapq.heappush(frontier, (neighbor.f, neighbor))  # put the neighbor into the min heap
        visited[cur.pos] = cur  # puts current position into the dictionary of visited places

    print("YOU FAILED")
    return (None, None, None)


""" Takes last node taken in from A* (Final) and the dictionary of visited nodes, and returns the path from start to end """


def tracePath(start, final, visited):
    path = []
    cur = final
    path.append(cur.pos)
    while (cur.pos != start):
        parent = visited.get(cur.parent.pos)
        path.append(parent.pos)  # add to the empty list, path, the parent node, until reaching the start position/node
        cur = cur.parent
    path.reverse()  # reverses the list so that it goes start to finish
    #print "path"
    #print path
    return path


def dirPath(path):  # get the waypoints
    if not path:
        return
    currDir = findDir(path[0], path[1])
    nextDir = findDir(path[0], path[1])
    waypoint = []
    dir = []
    n = 0
    for item in path:  # check if the direction of each set of consecutive nodes is different
        # print len(path)
        if (len(path) > n + 1):
            nextDir = findDir(path[n], path[n + 1])
            if (currDir != nextDir):  # if the directions are different then it is a waypoint
                waypoint.append(item)
            n = n + 1
            dir.append(currDir)
            currDir = nextDir
    dir.append(currDir)
    return waypoint, dir


def findDir(currNode, nextNode):  # get the directions of a node and the next
    x = currNode[0]
    y = currNode[1]
    a = nextNode[0]
    b = nextNode[1]
    # in global directions, not local
    if (x + 1 == a and y == b):  # right
        return 1
    elif (x == a and y + 1 == b):  # up
        return 2
    elif (x - 1 == a and y == b):  # left
        return 3
    elif (x == a and y - 1 == b):  # down
        return 4
    else:
        return 0
