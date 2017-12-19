#!/usr/bin/env python

import rospy
from realAStar import AStar
from nav_msgs.msg import GridCells
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from kobuki_msgs.msg import BumperEvent
import tf
import numpy
import math 
import rospy, tf, numpy, math
from tf.transformations import euler_from_quaternion

class Robot:
	
    def __init__(self):

        """
            This constructor sets up class variables and pubs/subs
		cmd_vel_mux/input/teleop
        """
        self._current = Pose() # initlize correctly 
        self._odom_list = tf.TransformListener()
        rospy.Timer(rospy.Duration(.1), self.timerCallback)
        self._vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=10) #one or 10  
        #rospy.Subscriber('mobile_base/events/bumper', BumperEvent, self.readBumper, queue_size=1) 
		# handle bumper events
	self.rate = rospy.Rate(1)
	self.vel_msg=Twist()
	self.vel_msg.linear.x=0
	self.vel_msg.linear.y=0
	self.vel_msg.linear.z=0
	self.vel_msg.angular.x=0
	self.vel_msg.angular.y=0
	self.vel_msg.angular.z=0
    def driveStraight(self, speed, distance):
	print "drive straight"

        # origin = copy.deepcopy(self._current) ##WHAT IS THIS???
	time = speed/distance
        origin = copy.deepcopy(self._current) #hint:  use this ##WHAT IS THIS??? Where you begin
        move_msg = Twist()
    	move_msg.linear.x = speed
    	move_msg.angular.z = 0
    	stop_msg = Twist()
    	stop_msg.linear.x = 0
    	stop_msg.angular.z = 0

    	#publish move message for desired time
	driveStartTime = rospy.Time.now().secs
	#print "Here"

	print (self._current.position.x, (distance+origin.position.x)-.2 , ((distance+origin.position.x)-.2)/2)

	orig=math.sqrt(self._current.position.x**2 + self._current.position.y**2)
	curr=math.sqrt((self._current.position.x-origin.position.x)**2 + (self._current.position.y-origin.position.y)**2)
	while(curr<(distance-.2) and not rospy.is_shutdown()):

		print ('distance curr is', curr , 'distance' , distance, 'orig' , orig)
		self._vel_pub.publish(move_msg)
		curr=math.sqrt((self._current.position.x-origin.position.x)**2 + (self._current.position.y-origin.position.y)**2)

	self._vel_pub.publish(stop_msg)
    def rotate(self,angle):
        """
            This method should populate a ??? message type and publish it to ??? in order to spin the robot
        """

        print("start")
        diff=0
        origin = copy.deepcopy(self._current)

        q = [origin.orientation.x,
             origin.orientation.y,
             origin.orientation.z,
             origin.orientation.w] # quaternion nonsense

        (roll, pitch, yaw) = euler_from_quaternion(q)
        

	yawo = yaw
	endAngle= angle + yaw
	while(endAngle>math.pi):
	    endAngle=endAngle-2*math.pi
	while(endAngle<-math.pi):
	    endAngle=endAngle+2*math.pi
	self.vel_msg.linear.x = 0
	self.vel_msg.angular.z= endAngle-yaw
	while(np.fabs(endAngle-yaw)>0.1):
	     self.vel_msg.angular.z= endAngle-yaw
	     rospy.loginfo (self.vel_msg)
	     self._vel_pub.publish (self.vel_msg)
	     q = [self._current.orientation.x,
              self._current.orientation.y,
              self._current.orientation.z,
              self._current.orientation.w] # quaternion nonsense
             (roll, pitch, yaw) = euler_from_quaternion(q)
	self.vel_msg.angular.z=0

    def timerCallback(self,evprent):
        """
            This is a callback that runs every 0.1s.
            Updates this instance of Robot's internal position variable (self._current)
		tf_monitor list transofrms aviable
		tf_echo output data to terminal
        """
        self._odom_list.waitForTransform('map', 'base_footprint', rospy.Time(0), rospy.Duration(1.0))
        (position, orientation) = self._odom_list.lookupTransform('map','base_footprint', rospy.Time(0)) #finds the position and oriention of two objects relative to each other (hint: this returns arrays, while Pose uses lists)
        self._current.position.x = position[0]
        self._current.position.y = position[1]

        self._current.orientation.x = orientation[0]
        self._current.orientation.y = orientation[1]
        self._current.orientation.z = orientation[2]
        self._current.orientation.w = orientation[3]
        q = [self._current.orientation.x,
             self._current.orientation.y,
             self._current.orientation.z,
             self._current.orientation.w] # quaternion nonsense

        (roll, pitch, yaw) = euler_from_quaternion(q)    

# reads in global map
class Map:
    def __init__(self):
        self.gridGx = None
        self.gridGy = None
        self.goalX= None
        self.goalY= None
        self.gridSx = None
        self.gridSy = None
        self.startPosX= None
        self.startPosY= None

    def mapCallBack(self,data):
        
        self.mapgrid = data
        self.resolution = data.info.resolution
        self.mapData = data.data
        self.width = data.info.width
        self.height = data.info.height
        self.offsetX = data.info.origin.position.x
        self.offsetY = data.info.origin.position.y
        print data.info

    #Get the Goal
    def readGoal(self,goal):
    
        self.gridGx = int((goal.pose.position.x - self.offsetX - (.5 * self.resolution)) / self.resolution) #grid coors
        self.gridGy = int((goal.pose.position.y - self.offsetY - (.5 * self.resolution)) / self.resolution)
        self.goalX= goal.pose.position.x #map coords
        self.goalY= goal.pose.position.y
        print (self.gridGx, self.gridGy, "Goal")

    #get the start
    def readStart(self, startPos):
        
        self.startPosX = startPos.pose.pose.position.x #map coords
        self.startPosY = startPos.pose.pose.position.y
        self.gridSx = int((startPos.pose.pose.position.x - self.offsetX - (.5 * self.resolution)) / self.resolution) #grid coords
        self.gridSy = int((startPos.pose.pose.position.y - self.offsetY - (.5 * self.resolution)) / self.resolution)
        print (self.gridSx, self.gridSy, startPos.pose.pose.position.x, startPos.pose.pose.position.y, "Start In") 

    #publish the walls
    def publishCells(self, grid , pub):
        obstacleExpansion=4
        cells = GridCells()
        cells.header.frame_id = 'map'
        cells.cell_width = self.resolution 
        cells.cell_height = self.resolution
        for i in range(len(grid)): #height should be set to height of grid
            if (grid[i] < 50):
                continue
            col= i%self.width
            row=(i-col)/self.width
            point=Point()
            point.x=(col*self.resolution)+self.offsetX + (.5 * self.resolution) # added secondary offset 
            point.y=(row*self.resolution)+self.offsetY + (.5 * self.resolution) # added secondary offset 
            point.z=0
            cells.cells.append(point)
            for j in range(-obstacleExpansion, obstacleExpansion+1):
                for k in range(-obstacleExpansion, obstacleExpansion+1):
                    point2=Point()
                    point2.x = ((col+j)*self.resolution)+self.offsetX + (.5 * self.resolution)
                    point2.y = ((row+k)*self.resolution)+self.offsetY + (.5 * self.resolution) # added secondary offset 
                    cells.cells.append(point2)
        print("publishing")
        pub.publish(cells)    #publish!

    #Get the Path from Astar
    def getPath(self, start, goal, pubP,pubW, pubS, pubG, mapgrid, width):
        pointslist =() #path
        waypointsList =() #waypoints
        #(pointslist, waypointsList)=aStar((start[0], start[1]), (goal[0], goal[1]), mapgrid)
        pointslist, waypointsList = AStar((start[0], start[1]), (goal[0], goal[1]), mapgrid, width)
        if(pointslist == None):
            print "No Path Found"
        else:
            #cell messages
            nPath = GridCells()
            nPath.header.frame_id = 'map' #path
            nPath.cell_width = self.resolution
            nPath.cell_height = self.resolution
            wPath =GridCells()
            wPath.header.frame_id = 'map' #waypoints
            wPath.cell_width = self.resolution
            wPath.cell_height = self.resolution
            sPath =GridCells()
            sPath.header.frame_id = 'map' #start grid
            sPath.cell_width = self.resolution
            sPath.cell_height = self.resolution
            gPath =GridCells()
            gPath.header.frame_id = 'map' #goal grid
            gPath.cell_width = self.resolution
            gPath.cell_height = self.resolution
            n=len(pointslist)
            for item in pointslist: #sort into path and waypoints
                    if(item != pointslist[0] and item!=pointslist[n-1]):
                        if(item in waypointsList):
                            pointp=Point()
                            pointp.x=(item[0]*self.resolution)+self.offsetX + (.5 * self.resolution) # added secondary offset 
                            pointp.y=(item[1]*self.resolution)+self.offsetY + (.5 * self.resolution) 
                            wPath.cells.append(pointp)
                        else:
                            point=Point()
                            point.x=(item[0]*self.resolution)+self.offsetX + (.5 * self.resolution) # added secondary offset 
                            point.y=(item[1]*self.resolution)+self.offsetY + (.5 * self.resolution) 
                            nPath.cells.append(point)
                    '''
                    point = Point()
                    point.x = (item[0] * self.resolution) + self.offsetX + (.5 * self.resolution)  # added secondary offset
                    point.y = (item[1] * self.resolution) + self.offsetY + (.5 * self.resolution)
                    nPath.cells.append(point)
                    '''
            point=Point()
            point.x=(start[0]*self.resolution)+self.offsetX + (.5 * self.resolution) # added secondary offset
            point.y=(start[1]*self.resolution)+self.offsetY + (.5 * self.resolution)
            print  (start[0], start[1],point.x, point.y, 'Start Out')
            sPath.cells.append(point)

            pointa=Point()
            pointa.x=(goal[0]*self.resolution)+self.offsetX + (.5 * self.resolution) # added secondary offset
            pointa.y=(goal[1]*self.resolution)+self.offsetY + (.5 * self.resolution)
            gPath.cells.append(pointa)

            pubP.publish(nPath)  #
            pubS.publish(sPath)
            pubG.publish(gPath)
            pubW.publish(wPath)
            print 'printed'


#Main handler of the project
def run():
    rospy.init_node('lab3')
    #rospy.init_node('drive_base') #added here
    turtle = Robot()
    rospy.sleep(2) #to here
    map=Map()
    sub = rospy.Subscriber("/map", OccupancyGrid, map.mapCallBack)
    pub = rospy.Publisher("/map_check", GridCells, queue_size=1)  
    pubpath = rospy.Publisher("/path", GridCells, queue_size=1) # you can use other types if desired
    pubway = rospy.Publisher("/waypoints", GridCells, queue_size=1)
    goal_sub = rospy.Subscriber('move_base_simple/goal2', PoseStamped, map.readGoal, queue_size=1) #change topic for best results
    goal_sub = rospy.Subscriber('initialpose', PoseWithCovarianceStamped, map.readStart, queue_size=1) #change topic for best results
    start_pub =rospy.Publisher("/start", GridCells, queue_size=1) 
    goal_pub =rospy.Publisher("/goal", GridCells, queue_size=1) 

    rospy.sleep(2) # wait a second for publisher, subscribers, and TF

    run = 0
   

    while (1 and not rospy.is_shutdown()):

        print "Start"
        (turtlex, turtley, turtleyaw ) =turtle.initpose()
        print(turtlex, turtley)
        map.startPosX = turtlex #map coords
        map.startPosY = turtley
        map.gridSx = int((turtlex - map.offsetX - (.5 * map.resolution)) / map.resolution) #grid coords
        map.gridSy = int((turtley - map.offsetY - (.5 * map.resolution)) / map.resolution)
        print(map.startPosX, map.startPosY, map.gridSx, map.gridSy)
        map.publishCells(map.mapData, pub) #publishing map data every 2 seconds
        if(map.gridGx is not None and run ==0): #check if there is a start and goal set
            print "Start"
            turtle._current.position.x=map.gridSx #added these
            turtle._current.position.y=map.gridSy #two lines
            map.getPath((map.gridSx,map.gridSy), (map.gridGx,map.gridGy), pubpath,pubway, start_pub, goal_pub, map.mapData, map.width)
            run = 1
            initstartx = map.gridSx
            initgoalx =map.gridGx
        elif(run==1 and initstartx !=map.gridSx and initgoalx != map.gridGx):
            print "In Here"
            map.getPath((map.gridSx, map.gridSy), (map.gridGx, map.gridGy), pubpath, pubway, start_pub, goal_pub,
                        map.mapData, map.width)
            run = 1
            initstartx = map.gridSx
            initgoalx = map.gridGx
        rospy.sleep(2)

if __name__ == '__main__':

    try:
        run()
    except rospy.ROSInterruptException:
        pass


