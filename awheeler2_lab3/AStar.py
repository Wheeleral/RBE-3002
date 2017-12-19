#!/usr/bin/env python
import rospy
from nav_msgs.msg import GridCells, Path
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from kobuki_msgs.msg import BumperEvent
from tf.transformations import quaternion_from_euler
import tf
import numpy
import math
import rospy, tf, numpy, math

class Node:
	def __init__(self,value, coord, start, goal, grid): #init function for creating a node
		self.value = value
		self.coord = coord
		self.parent = []
		self.start = start
		self.goal = goal
		self.children = []
		self.grid = grid
		self.path = []
		if(len(self.path) == 0):
			self.path.append(start)
		
	def Children(self, path): #Get the Children of a node
		x = self.coord[0]
		y = self.coord[1]
		v = self.value
		s = self.start
		g = self.goal
		m = self.grid
		width = self.grid.info.width
		height = self.grid.info.height
		inPath =False
		
		for i in range(0, height-1): #height should be set to height of grid
			for j in range(0, width-1): #width should be set to width of grid
				if (self.grid.data[i*width+j] == 100): #its a wall
					#print 'WALLL'
					pass
				elif(x+1 == i and y == j): #theres a child
					inPath=False
					for item in path: #check if it's already in the path
						if(x+1 ==item[0] and y ==item[1]):
							inPath = True
							
					if(inPath==False):
						child1= Node(v+1, (x+1, y), s, g, m)
						self.children.append(child1)
				elif(x == i and y+1 == j):
					inPath=False
					for item in path:
						if(x ==item[0] and y+1 ==item[1]):
							inPath = True
						
					if(inPath==False):
						child2 = Node(v+1, (x, y+1), s, g, m)
						self.children.append(child2)
						
				elif(x-1 == i and y == j):
					inPath=False
					for item in path:
						if(x-1 ==item[0] and y ==item[1]):
							inPath = True
						
					if(inPath==False):
						child3 = Node(v+1, (x-1, y), s, g, m)
						self.children.append(child3)
	
				elif(x == i and y-1 == j):
					inPath=False
					for item in path:
						if(x ==item[0] and y-1 ==item[1]):
							inPath = True
						
					if(inPath==False):
						child4 = Node(v+1, (x, y-1), s, g, m)
						self.children.append(child4)
				else:
					pass
						

	def Manhattan(self, point, point2): #manhattan distance
		return abs(point[0] - point2[0]) + abs(point[1]-point2[1])
	
	def Greedy(self): #greedy distance
		x_side = abs(int(self.goal[0] - self.coord[0]))
		y_side = abs(int(self.goal[1] - self.coord[1]))
		hn = math.sqrt((x_side * x_side) + (y_side * y_side))
		return hn

	def CalculateF(self): #F value of the node
		g = self.value
		h = self.Greedy()
		return g + h
		

'''
def copyMap(data): #get the map data
	global mapGrid
	global mapData
	global width
	global height
	global mapgrid
	global resolution
	
	mapGrid = data
	resolution = data.info.resolution
	mapData = data.data
	width = data.info.width
	height = data.info.height
	
	initGrid(mapGrid)
'''

def Solve(start, goal, node):	# get the path
	shortest = 0
	current = node
	while(current.coord != goal): #iterate through the children
		#print start, goal
		#print node.path
		current.Children(node.path) #get the child
		for i in current.children: #get the shortest child
			if(shortest == 0):
				child = i
				shortest = i.CalculateF()
			elif(i.CalculateF() < shortest):
				child = i
				shortest = i.CalculateF()
		child.parent.append(current)
		node.path.append(current.coord)
		current = child
		shortest = 0
	node.path.append(current.coord) #add the the lsit of nodes
	return node.path

def dirPath(path): #get the wayp   oints
	if not path:
		return
	currDir=findDir(path[0], path[1])
	nextDir=findDir(path[0], path[1])
	waypoint=[]
	n=0
	for item in path: #check if the direction of each set of consecutive nodes is differnt
		#print len(path)
		if(len(path) > n+1):
			#print "in Else"	
			nextDir=findDir(path[n], path[n+1])	
			if (currDir != nextDir): #if the directions are differnt then it is a waypoint
				waypoint.append(item)
			n=n+1
			currDir=nextDir
	return waypoint

def findDir(currNode, nextNode):#get the directions of a node and the next
	x=currNode[0]
	y=currNode[1]
	a=nextNode[0]
	b=nextNode[1]
	if(x+1 == a and y == b):
		return 1
	elif(x == a and y+1 == b):
		return 2
	elif(x-1 == a and y == b):
		return 3
	elif(x == a and y-1 == b):
		return 4
	else:
		return 0
		#debug




def aStar(start, goal, grid): #overall function
	

	origin = Node(0, start, start, goal, grid)
	solution = Solve(start, goal, origin)
	waypoint = dirPath(solution)
	#print solution, waypoint
	return (solution, waypoint)