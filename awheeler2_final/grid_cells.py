#!/usr/bin/env python

import rospy
from realAStar import AStar
from robotMotion import Robot
from nav_msgs.msg import GridCells
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from kobuki_msgs.msg import BumperEvent
import tf
import numpy
import math
import rospy, tf, numpy, math
import heapq


##TODO Fix movement and starting rotation
##ToDO stop when there are no more frontiers            #Untested by works in theory
##TODO find a frontier cell that is more than one cell


class mapTransform:  # for map transforms
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.list = []
        self.total = 0


class frontier:
    def __init__(self, x, y):
        self.list = [(x, y)]
        self.length = 1

    def getKey(self):
        return self.length

    def __cmp__(self, other):
        if hasattr(other, 'getKey'):
            return self.getKey().__cmp__(other.getKey())


# reads in global map as
class Map:
    def __init__(self):
        self.gridGx = None
        self.gridGy = None
        self.goalX = None
        self.goalY = None
        self.gridSx = None
        self.gridSy = None
        self.startPosX = None
        self.startPosY = None
        self.eMap = []
        self.costMap = []
        self.frontierMap = []
        self.complete = False
        self.backup = False

    def mapCallBack(self, data):

        self.mapgrid = data
        self.resolution = data.info.resolution
        self.mapData = data.data
        self.width = data.info.width
        self.height = data.info.height
        self.offsetX = data.info.origin.position.x
        self.offsetY = data.info.origin.position.y
        #print data.info

    def CostMap(self, data):
        self.cresolution = data.info.resolution
        self.cmapData = data.data
        self.cwidth = data.info.width
        self.cheight = data.info.height
        self.coffsetX = data.info.origin.position.x
        self.coffsetY = data.info.origin.position.y

    # Get the Goal
    def readGoal(self, goal):

        self.gridGx = int(
            (goal.pose.position.x - self.offsetX - (.5 * self.resolution)) / self.resolution)  # grid coors
        self.gridGy = int((goal.pose.position.y - self.offsetY - (.5 * self.resolution)) / self.resolution)
        self.goalX = goal.pose.position.x  # map coords
        self.goalY = goal.pose.position.y
        print (self.gridGx, self.gridGy, "Goal")

    # get the start
    def readStart(self, startPos):

        self.startPosX = startPos.pose.pose.position.x  # map coords
        self.startPosY = startPos.pose.pose.position.y
        self.gridSx = int(
            (startPos.pose.pose.position.x - self.offsetX - (.5 * self.resolution)) / self.resolution)  # grid coords
        self.gridSy = int((startPos.pose.pose.position.y - self.offsetY - (.5 * self.resolution)) / self.resolution)
        print (self.gridSx, self.gridSy, startPos.pose.pose.position.x, startPos.pose.pose.position.y, "Start In")

        # publish the walls

    def publishCells(self, grid, pub):
        cells = GridCells()
        cells.header.frame_id = 'map'
        cells.cell_width = self.resolution
        cells.cell_height = self.resolution
        for i in range(len(grid)):  # height should be set to height of grid
            if (grid[i] < 50):
                continue
            col = i % self.width
            row = (i - col) / self.width
            point = Point()
            point.x = (col * self.resolution) + self.offsetX + (.5 * self.resolution)  # added secondary offset
            point.y = (row * self.resolution) + self.offsetY + (.5 * self.resolution)  # added secondary offset
            point.z = 0
            cells.cells.append(point)

        pub.publish(cells)  # publish!

    def pointDef(self, col, row):
        point = Point()
        point.x = (col * self.resolution) + self.offsetX + (.5 * self.resolution)  # added secondary offset
        point.y = (row * self.resolution) + self.offsetY + (.5 * self.resolution)  # added secondary offset
        point.z = 0
        return point

    def obstacleExpansion(self, grid, pub, howExpanded):
        for i in grid:
            self.eMap.append(int(i))

        cells = GridCells()
        cells.header.frame_id = 'map'
        cells.cell_width = self.resolution
        cells.cell_height = self.resolution
        checked = []
        for i in range(len(grid)):  # height should be set to height of grid

            # print (i, grid[i])
            if (grid[i] < 50):
                continue
            col = i % self.width
            row = (i - col) / self.width
            x = col
            y = row
            # Manhattan
            for k in range(x - howExpanded, x + howExpanded+1):  # it goes x+2 because for loops are exclusive
                for j in range(y - howExpanded, y + howExpanded+1):
                    if (k > 0 and k < self.width and j > 0 and j < self.width):
                        point = self.pointDef(k, j)
                        cells.cells.append(point)
                        self.eMap[j * self.width + k] = grid[i]
        # print checked
        pub.publish(cells)

    # takes the 80,80 cost map and rotates it to fit which square it belongs to.
    # averages the tingy squares to the big squares
    # gives heuristic to each square
    def costMapTransform(self, currx, curry, yaw):
        transforms = []
        for i in range(-7, 6):
            for j in range(-7, 6):
                transforms.append(mapTransform(i, j))

        for i in range(len(self.cmapData)):  # height should be set to height of grid
            y = i % self.cwidth  # was flipped
            x = (i - y) / self.cwidth
            xp = math.floor(
                ((x - self.cwidth / 2) * math.cos(yaw) - (y - self.cwidth / 2) * math.sin(yaw)) / 6)  # +currx
            yp = math.floor(
                ((x - self.cwidth / 2) * math.sin(yaw) + (y - self.cwidth / 2) * math.cos(yaw)) / 6)  # +curry
            for item in transforms:
                if (xp == item.x and yp == item.y):
                    item.list.append(self.cmapData[i])
                    # print ("cost:", xp, yp, i, self.cmapData[i], currx, curry)
        for item in transforms:
            item.total = self.avgList(item.list)
            item.x = item.x + currx  # Goes Global
            item.y = item.y + curry
            # print ("trans:", item.x,item.y , item.total, currx, curry)

        return transforms

    def createCostMap(self, grid):
        for i in grid:
            self.costMap.append(int(i))

    def updateCostMap(self, transforms):
        for item in transforms:
            if (self.costMap[item.y * self.width + item.x] < 95 or item.total < 95):
                self.costMap[item.y * self.width + item.x] = item.total

    # Helper Function
    def avgList(self, list):
        sum = 0
        total = 0
        # print len(list)
        for item in list:
            sum = sum + item
            total = total + 1
        if (total > 0):
            return sum / total
        else:
            return 1

    # create a grid of a given resolution
    def createGrid(self, res):
        newGrid = GridCells()
        newGrid.header.frame_id = 'map'
        newGrid.cell_width = res
        newGrid.cell_height = res
        return newGrid

    # adding points from specified lists to the specified path
    def appendToPath(self, list, path):
        point = Point()
        print (self.resolution)
        print (list[0])
        point.x = (list[0] * self.resolution) + self.offsetX + (.5 * self.resolution)
        point.y = (list[1] * self.resolution) + self.offsetY + (.5 * self.resolution)
        path.cells.append(point)

    def publishStart(self, start, pub):
        sPath = self.createGrid(self.resolution)
        self.appendToPath(start, sPath)
        pub.publish(sPath)

    # Get the Path from Astar
    def getPath(self, start, goal, pubP, pubW, pubS, pubG, mapgrid, width, cost):
        pointslist = ()  # path
        waypointsList = ()  # waypoints
        # (pointslist, waypointsList)=aStar((start[0], start[1]), (goal[0], goal[1]), mapgrid)
        pointslist, waypointsList, dir = AStar((start[0], start[1]), (goal[0], goal[1]), self.eMap, width, cost)
        if (dir == None):
            print "No Path Found"
        else:
            # cell messages
            nPath = self.createGrid(self.resolution)
            wPath = self.createGrid(self.resolution)
            sPath = self.createGrid(self.resolution)
            gPath = self.createGrid(self.resolution)
            n = len(pointslist)
            for item in pointslist:  # sort into path and waypoints
                if (item != pointslist[0] and item != pointslist[n - 1]):
                    if (item in waypointsList):
                        self.appendToPath(item, wPath)
                    else:
                        self.appendToPath(item, nPath)
            self.appendToPath(start, sPath)

            self.appendToPath(goal, gPath)

            pubP.publish(nPath)  #
            pubS.publish(sPath)
            pubG.publish(gPath)
            pubW.publish(wPath)
            # print 'printed'
        return dir
    '''
    def inTotalFrontier(self, totalFrontierCells, x, y):
        for item in totalFrontierCells:
            if (item[0] == x and item[1] == y):
                return False
        return True

    def frontierExpansion(self, currFrontier, currXY, grid,
                          totalFrontierCells):  # totalFrontierCells is all frontier cells that have been explored
        totalFrontierCells.append(currXY)
        currFrontier.append(currXY)
        for k in range(currXY[0] - 1, currXY[0] + 2):  # it goes x+2 because for loops are exclusive
            for j in range(currXY[1] - 1, currXY[1] + 2):
                if (k > 0 and k < self.width and j > 0 and j < self.width and self.validateFrontier(grid, k, j, self.width)
                    and self.inTotalFrontier(totalFrontierCells, k, j)):
                    newXY = (k, j)
                    (currFrontier, totalFrontierCells) = self.frontierExpansion(currFrontier, newXY, grid, totalFrontierCells)
        return (currFrontier,totalFrontierCells)

    '''
    def readBumper(self, msg):
        self.backup=True

    def idFrontier(self, grid):  # get first frontier block and expand from it to get frontier segment
        totalFrontierCells = []
        listOfFrontiers = []
        index=0
        for i in range(len(grid)):  # height should be set to height of grid
            if (grid[i] > 0):
                continue
            x = i % self.width
            y = (i - x) / self.width
            if (self.inTotalFrontier(totalFrontierCells, x, y) and self.validateFrontier(grid, x, y)):
                currFrontier=[]
                currXY=(x,y)
                (currFrontier, totalFrontierCells) = self.frontierExpansion(currFrontier, currXY, grid, totalFrontierCells)
                listOfFrontiers.append(currFrontier)

        return listOfFrontiers

    def validateFrontier(self, x, y, grid):
        flag = False
        for i in range(x - 1, x + 2):
            for j in range(y - 1, y + 2):
                if (grid[j * self.width + i] == 0):
                    flag = True
        if (grid[y * self.width + x] == -1 and flag == True):  # convert from x,y to grid cell number
            return True
        else:
            return False

    def findFrontierCell(self, grid):
        for i in range(len(grid)):  # height should be set to height of grid
            if (grid[i] > 0):
                continue
            x = i % self.width
            y = (i - x) / self.width
            if (self.validateFrontier(x, y, grid)):
                return x, y
        self.complete=True      #if self.validate is never true set map as complete
        return -10, -10

    '''
    def getNextFrontier(self, grid):
        Listfrontiers = self.idFrontier(grid)
        frontier = sorted(Listfrontiers)
        return self.frontierCentroid(frontier[0])
        #for distance, get centoids of each first, calc dist from robot then pop

    def publishFrontier(self, grid, pub):
        Listfrontiers = self.idFrontier(grid)
        cells = GridCells()
        cells.header.frame_id = 'map'
        cells.cell_width = self.resolution
        cells.cell_height = self.resolution
        gPath = self.createGrid(self.resolution)
        for item in Listfrontiers:  # sort into path and waypoints
            self.appendToPath(item, gPath)
        pub.publish(cells)  # publish!

    def frontierCentroid(self, frontier):
        x_c = y_c = count = 0
        for item in frontier.list:
            x_c = x_c + item.x
            y_c = y_c + item.y
            count = count + 1
        x_c = x_c / count
        y_c = y_c / count
        return (x_c, y_c)
    '''

    def aStarMovement(self, turtle, curr, turtleyaw, pubpath, pubway, start_pub, goal_pub): #consolidate run
        print("aStarMovement")
        dir = self.getPath((self.gridSx, self.gridSy), (self.gridGx, self.gridGy), pubpath, pubway, start_pub, goal_pub,
                          self.eMap, self.width, self.costMap)
        print ("dir", dir)
        #run = 1
        #initstartx = map.gridSx
        #initgoalx = map.gridGx
        turtle.aStarStart(dir, turtleyaw)
        print("Finished a star start")
        if(curr == None):
            curr =dir[0]
        turtle.aStarMove(curr, dir[1], turtleyaw)
        curr = dir[1]
        return curr

# Main handler of the project
def run():
    print("Starting run function")
    rospy.init_node('Team5')
    map = Map()
    turtle = Robot()
    sub = rospy.Subscriber("/map", OccupancyGrid, map.mapCallBack)
    pub = rospy.Publisher("/map_check", GridCells, queue_size=1)
    pubpath = rospy.Publisher("/path", GridCells, queue_size=1)  # you can use other types if desired
    pubway = rospy.Publisher("/waypoints", GridCells, queue_size=1)
    goal_sub = rospy.Subscriber('move_base_simple/goal2', PoseStamped, map.readGoal,
                                queue_size=1)  # change topic for best results
    start_sub = rospy.Subscriber('initialpose', PoseWithCovarianceStamped, map.readStart,
                                 queue_size=1)  # change topic for best results
    start_pub = rospy.Publisher("/start", GridCells, queue_size=1)
    goal_pub = rospy.Publisher("/goal", GridCells, queue_size=1)
    obPub = rospy.Publisher("/map_expand", GridCells, queue_size=1)
    costMapSub = rospy.Subscriber("/move_base/local_costmap/costmap", OccupancyGrid, map.CostMap)
    costMapPub = rospy.Publisher("/map_cost", GridCells, queue_size=1)
    frontierPub = rospy.Publisher("/map_frontier", GridCells, queue_size=1)
    exploredPub = rospy.Publisher("/map_explored", GridCells, queue_size=1)
    bumpersub=rospy.Subscriber('mobile_base/events/bumper', BumperEvent, map.readBumper, queue_size=1)  # handle bumper events

    rospy.sleep(2)  # wait a second for publisher, subscribers, and TF

    run = 0
    # turtle.driveStraight(.5, 1)
    (turtlex, turtley, turtleyaw) = turtle.initPose()
    print("turtle init:", turtlex, turtley, turtleyaw)
    map.startPosX = turtlex  # map coords
    map.startPosY = turtley
    map.gridSx = int((turtlex - map.offsetX - (.5 * map.resolution)) / map.resolution)  # grid coords
    map.gridSy = int((turtley - map.offsetY - (.5 * map.resolution)) / map.resolution)
    map.publishStart((map.gridSx, map.gridSy), start_pub)

    expand = int(.35/map.resolution)

    map.obstacleExpansion(map.mapData, obPub, expand)
    map.publishCells(map.mapData, pub)  # publishing map data every 2 seconds
    map.createCostMap(map.eMap)
    #map.publishFrontier(map.eMap, frontierPub)



    #print("Res", map.resolution, map.cresolution, "Width", map.width, map.cwidth, "height", map.height, map.cheight, "offx", map.offsetX, map.coffsetX)
    while(map.complete == False and not rospy.is_shutdown()):  # change to map not complete
        print ("width" , map.width)
        if (map.backup == True):
            print "Backing Up"
            turtle.backup()
            map.backup = False
        # spin once
        # map update
        # identify frontiers
        # get centroid of frontiers
        # move
        turtle.rotate(math.pi)
        turtle.rotate(math.pi)
        print "Rotation Complete"
        goal = map.findFrontierCell(map.eMap)
        if(goal[0]!=-10): #this will be false if no frontiers were identified
            map.gridGx =goal[0]
            map.gridGy=goal[1]
            run=0
            while map.gridSx !=map.gridGx and map.gridSy!=map.gridGy:
                if(map.startPosX is not None and map.gridGx is not None and run == 0):  # check if there is a start and goal set
                    print "Start"
                    curr = map.aStarMovement(turtle, None, turtleyaw, pubpath, pubway, start_pub,goal_pub)  # consolidate run
                    run = 1
                elif (run >= 1 and (map.gridSy != map.gridGy or map.gridSx != map.gridGx)):
                    # print "In Here"
                    curr = map.aStarMovement(turtle, curr, turtleyaw, pubpath, pubway, start_pub, goal_pub)

                (turtlex, turtley, turtleyaw) = turtle.initPose()
                print("Turtle:", turtlex, turtley, turtleyaw)
                map.startPosX = turtlex  # map coords
                map.startPosY = turtley
                map.gridSx = int((turtlex - map.offsetX - (.5 * map.resolution)) / map.resolution)  # grid coords
                map.gridSy = int((turtley - map.offsetY - (.5 * map.resolution)) / map.resolution)
                rospy.sleep(1)
        # print (map.startPosX, map.startPosY, map.gridSx, map.gridSy)
        #cost = map.costMapTransform(map.gridSx, map.gridSy, turtleyaw)
        #map.updateCostMap(cost)
        #map.publishCells(map.costMap, costMapPub)
        # for i in cost:
        # print ("Up Here")
        #print ("Grid", map.gridSx, map.gridSy, map.gridGx, map.gridGy)
        '''
        if (map.startPosX is not None and map.gridGx is not None and run == 0):  # check if there is a start and goal set
            print "Start"
            curr= map.aStarMovement(turtle, None, turtleyaw, pubpath, pubway, start_pub, goal_pub) #consolidate run
            run=1
        elif (run >= 1 and (map.gridSy != map.gridGy or map.gridSx != map.gridGx)):
            # print "In Here"
            curr=map.aStarMovement(turtle, curr, turtleyaw, pubpath, pubway, start_pub, goal_pub)
        '''



        rospy.sleep(1)
    while (1):
        print("DONE!!!!")
        turtle.rotate(math.pi/2)
        turtle.rotate(-math.pi/2)


if __name__ == '__main__':

    try:
        run()
    except rospy.ROSInterruptException:
        pass