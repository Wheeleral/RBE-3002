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


class mapTransform:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.list = []
        self.total = 0


# reads in global map
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

    def mapCallBack(self, data):

        self.mapgrid = data
        self.resolution = data.info.resolution
        self.mapData = data.data
        self.width = data.info.width
        self.height = data.info.height
        self.offsetX = data.info.origin.position.x
        self.offsetY = data.info.origin.position.y
        print data.info

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

    def obstacleExpansion(self, grid, pub):
        for i in grid:
            self.eMap.append(int(i))
            # print (i , grid[i], self.eMap[i])

        cells = GridCells()
        cells.header.frame_id = 'map'
        cells.cell_width = self.resolution
        cells.cell_height = self.resolution
        checked = []
        for i in range(len(grid)):  # height should be set to height of grid
            # self.eMap[i] = grid[i]

            # print (i, grid[i])
            if (grid[i] < 50):
                # self.eMap[i] = grid[i]
                '''
                if (i not in checked):
                    self.eMap[i] = grid[i]
                    checked.append(i)
                '''
                continue
            col = i % self.width
            row = (i - col) / self.width
            x = col
            y = row
            # points
            # print(self.width, x, y, y*self.width+x)
            # Manhattan

            if ((x - 1) > 0 and (x - 1) < self.width):
                point3 = self.pointDef(col, row - 1)
                cells.cells.append(point3)
                self.eMap[y * self.width + (x - 1)] = grid[i]

            if ((x + 1) > 0 and (x + 1) < self.width):
                point4 = self.pointDef(col, row + 1)
                cells.cells.append(point4)
                self.eMap[y * self.width + (x + 1)] = grid[i]

            if ((y - 1) > 0 and (y - 1) < self.width):
                point1 = self.pointDef(col - 1, row)
                cells.cells.append(point1)
                self.eMap[(y - 1) * self.width + x] = grid[i]

            if ((y + 1) > 0 and (y + 1) < self.width):
                point2 = self.pointDef(col + 1, row)
                cells.cells.append(point2)
                self.eMap[(y + 1) * self.width + x] = grid[i]

            # Corners
            if ((x - 1) > 0 and (x - 1) < self.width and (y - 1) > 0 and (y - 1) < self.width):
                point5 = self.pointDef(col - 1, row - 1)
                cells.cells.append(point5)
                self.eMap[(y - 1) * self.width + x - 1] = grid[i]

            if ((x - 1) > 0 and (x - 1) < self.width and (y + 1) > 0 and (y + 1) < self.width):
                point6 = self.pointDef(col + 1, row - 1)
                cells.cells.append(point6)
                self.eMap[(y + 1) * self.width + x - 1] = grid[i]

            if ((x + 1) > 0 and (x + 1) < self.width and (y - 1) > 0 and (y - 1) < self.width):
                point7 = self.pointDef(col - 1, row + 1)
                cells.cells.append(point7)
                self.eMap[(y - 1) * self.width + x + 1] = grid[i]

            if ((x + 1) > 0 and (x + 1) < self.width and (y + 1) > 0 and (y + 1) < self.width):
                point8 = self.pointDef(col + 1, row + 1)
                cells.cells.append(point8)
                self.eMap[(y + 1) * self.width + x + 1] = grid[i]
                # print "EMAP"
                # for i in range(len(self.eMap)) :
                # print (i , grid[i], self.eMap[i])
        # print checked
        pub.publish(cells)

    # takes the 80,80 cost map and rotates it to fit which square it belongs to.
    # averages the tingy squares to the big squares
    # gives heuristic to each square
    def costMapTransform(self, currx, curry, yaw):
        transforms = []
        for i in range(-6, 6):
            for j in range(-6, 6):
                transforms.append(mapTransform(i, j))

        for i in range(len(self.cmapData)):  # height should be set to height of grid
            x = i % self.cwidth
            y = (i - x) / self.cwidth
            xp = math.floor(
                ((x - self.cwidth / 2) * math.cos(yaw) - (y - self.cwidth / 2) * math.sin(yaw)) / 6)  # +currx
            yp = math.floor(
                ((x - self.cwidth / 2) * math.sin(yaw) + (y - self.cwidth / 2) * math.cos(yaw)) / 6)  # +curry
            for item in transforms:
                if (xp == item.x and yp == item.y):
                    item.list.append(self.cmapData[i])
        for item in transforms:
            item.total = self.avgList(item.list)
            item.x = item.x + currx  # Goes Global
            item.y = item.y + curry

        return transforms

    def createCostMap(self, grid):
        for i in grid:
            self.costMap.append(int(i))

    def updateCostMap(self, transforms):
        for item in transforms:
            if(self.costMap[item.y*self.width+item.x] < 95 or item.total<95):
                self.costMap[item.y*self.width+item.x]= item.total


    # Helper Function
    def avgList(self, list):
        sum = 0
        total = 0
        # print len(list)
        for item in list:
            sum = sum + item
            total = total + 1
        if(total>0):
            return sum / total
        else:
            return 1

    def publishStart(self, start, pub):
        sPath = GridCells()
        sPath.header.frame_id = 'map'  # start grid
        sPath.cell_width = self.resolution
        sPath.cell_height = self.resolution
        point = Point()
        point.x = (start[0] * self.resolution) + self.offsetX + (.5 * self.resolution)  # added secondary offset
        point.y = (start[1] * self.resolution) + self.offsetY + (.5 * self.resolution)
        # print  (start[0], start[1], point.x, point.y, 'Start Out')
        sPath.cells.append(point)
        pub.publish(sPath)

    # Get the Path from Astar
    def getPath(self, start, goal, pubP, pubW, pubS, pubG, mapgrid, width, cost):
        pointslist = ()  # path
        waypointsList = ()  # waypoints
        # (pointslist, waypointsList)=aStar((start[0], start[1]), (goal[0], goal[1]), mapgrid)
        pointslist, waypointsList, dir = AStar((start[0], start[1]), (goal[0], goal[1]), self.eMap, width, cost)
        if (pointslist == None):
            print "No Path Found"
        else:
            # cell messages
            nPath = GridCells()
            nPath.header.frame_id = 'map'  # path
            nPath.cell_width = self.resolution
            nPath.cell_height = self.resolution
            wPath = GridCells()
            wPath.header.frame_id = 'map'  # waypoints
            wPath.cell_width = self.resolution
            wPath.cell_height = self.resolution
            sPath = GridCells()
            sPath.header.frame_id = 'map'  # start grid
            sPath.cell_width = self.resolution
            sPath.cell_height = self.resolution
            gPath = GridCells()
            gPath.header.frame_id = 'map'  # goal grid
            gPath.cell_width = self.resolution
            gPath.cell_height = self.resolution
            n = len(pointslist)
            for item in pointslist:  # sort into path and waypoints
                if (item != pointslist[0] and item != pointslist[n - 1]):
                    if (item in waypointsList):
                        pointp = Point()
                        pointp.x = (item[0] * self.resolution) + self.offsetX + (
                            .5 * self.resolution)  # added secondary offset
                        pointp.y = (item[1] * self.resolution) + self.offsetY + (.5 * self.resolution)
                        wPath.cells.append(pointp)
                    else:
                        point = Point()
                        point.x = (item[0] * self.resolution) + self.offsetX + (
                            .5 * self.resolution)  # added secondary offset
                        point.y = (item[1] * self.resolution) + self.offsetY + (.5 * self.resolution)
                        nPath.cells.append(point)
            point = Point()
            point.x = (start[0] * self.resolution) + self.offsetX + (.5 * self.resolution)  # added secondary offset
            point.y = (start[1] * self.resolution) + self.offsetY + (.5 * self.resolution)
            # print  (start[0], start[1], point.x, point.y, 'Start Out')
            sPath.cells.append(point)

            pointa = Point()
            pointa.x = (goal[0] * self.resolution) + self.offsetX + (.5 * self.resolution)  # added secondary offset
            pointa.y = (goal[1] * self.resolution) + self.offsetY + (.5 * self.resolution)
            gPath.cells.append(pointa)

            pubP.publish(nPath)  #
            pubS.publish(sPath)
            pubG.publish(gPath)
            pubW.publish(wPath)
            # print 'printed'
        return dir


# Main handler of the project
def run():
    rospy.init_node('lab3')
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

    rospy.sleep(2)  # wait a second for publisher, subscribers, and TF

    run = 0
    # turtle.driveStraight(.5, 1)
    (turtlex, turtley, turtleyaw) = turtle.initPose()
    print(turtlex, turtley, turtleyaw)
    map.startPosX = turtlex  # map coords
    map.startPosY = turtley
    map.gridSx = int((turtlex - map.offsetX - (.5 * map.resolution)) / map.resolution)  # grid coords
    map.gridSy = int((turtley - map.offsetY - (.5 * map.resolution)) / map.resolution)
    map.publishStart((map.gridSx, map.gridSy), start_pub)
    map.obstacleExpansion(map.mapData, obPub)
    map.publishCells(map.mapData, pub)  # publishing map data every 2 seconds
    map.createCostMap(map.eMap)
    print("Res", map.resolution, map.cresolution, "Width", map.width, map.cwidth, "height",
          map.height, map.cheight, "offx", map.offsetX, map.coffsetX)
    while (1 and not rospy.is_shutdown()):

        # print (map.startPosX, map.startPosY, map.gridSx, map.gridSy)
        cost = map.costMapTransform(map.gridSx, map.gridSy, turtleyaw)
        map.updateCostMap(cost)
        # for i in cost:
        print ("Up Here")
        print ("Grid", map.gridSx, map.gridSy, map.gridGx, map.gridGy)

        if (map.startPosX is not None and map.gridGx is not None and run == 0):  # check if there is a start and goal set

            print "Start"
            dir = map.getPath((map.gridSx, map.gridSy), (map.gridGx, map.gridGy), pubpath, pubway, start_pub, goal_pub,
                              map.eMap, map.width, map.costMap)
            run = 1
            initstartx = map.gridSx
            initgoalx = map.gridGx
            turtle.aStarStart(dir, turtleyaw)
            turtle.aStarMove(dir[0], dir[1], turtleyaw)
            curr = dir[1]

        # currently works 3 tiems
        elif (run >= 1 and (map.gridSy != map.gridGy or map.gridSx != map.gridGx)):
            print "In Here"
            dir = map.getPath((map.gridSx, map.gridSy), (map.gridGx, map.gridGy), pubpath, pubway, start_pub, goal_pub,
                        map.eMap, map.width, map.costMap)
            run = run + 1
            print ("run", run, "curr", curr, "next", dir[1], dir[0])
            initstartx = map.gridSx
            initgoalx = map.gridGx
            #turtle.aStarStart(dir, turtleyaw)
            turtle.aStarMove(curr, dir[1], turtleyaw)
            curr = dir[1]
        (turtlex, turtley, turtleyaw) = turtle.initPose()
        print(turtlex, turtley, turtleyaw)
        map.startPosX = turtlex  # map coords
        map.startPosY = turtley
        map.gridSx = int((turtlex - map.offsetX - (.5 * map.resolution)) / map.resolution)  # grid coords
        map.gridSy = int((turtley - map.offsetY - (.5 * map.resolution)) / map.resolution)

        rospy.sleep(1)


if __name__ == '__main__':

    try:
        run()
    except rospy.ROSInterruptException:
        pass
