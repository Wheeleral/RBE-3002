#!/usr/bin/env python
import rospy, tf, copy, math
from kobuki_msgs.msg import BumperEvent
from geometry_msgs.msg import Twist, Pose, PoseStamped
from tf.transformations import euler_from_quaternion
import numpy as np
from std_msgs.msg import String
from nav_msgs.msg import Odometry


#def init():
	#rospy.Subscriber('move_base_simple/goal', PoseStamped, queue_size=1)
	#rospy.Subscriber('map', OccupancyGrid,  queue_size=1)
	#rospy.Subscriber('map_metadata', MapMetaData,  queue_size=1) #maybe not
	#rospy.Subscriber('map_updates', OccupancyGridUpdate,  queue_size=1) #maybe not only if map is dynamic
	#rospy.Subscriber('initialpose', PoseWithCovarianceStamped,  queue_size=5)
	
"""
A*(start, goal)
	
evaluatednodes- empty set
opennodes-empty evaluated at begiing
map

if not at goal
checkfrontier(the neighboors
	if not empty		
		remove current node from the opennodes and move to evaluated
		claulate the fscore of each neighboor not in evaulated
		get losest fscore and add to opennodes make this the current node
		get current path from start to current node



fscore=gscore +hscore
g-from graph
h-eculidian distance
"""
if __name__ == '__main__':

		gridpub= rospy.publisher('/grid/cells', GridCells)

		gridpubmsg = GridCells()
		gridpubmsg.cell_width =1 
		gridpubmsg.cell_height =1




