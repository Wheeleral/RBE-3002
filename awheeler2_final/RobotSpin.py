#!/usr/bin/env python

from nav_msgs.msg import Odometry, OccupancyGrid
from robotMotion import Robot
import rospy, tf, numpy, math
import subprocess
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose, PoseStamped

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

    def mapCallBack(self, data):
        self.mapgrid = data
        self.resolution = data.info.resolution
        self.mapData = data.data
        self.width = data.info.width
        self.height = data.info.height
        self.offsetX = data.info.origin.position.x
        self.offsetY = data.info.origin.position.y
        #print data.info

def spinFunc(turtlebot):

    # testPub = rospy.Publisher('Check', String, queue_size=10)
    # rostopic pub /cmd_vel_mux/input/teleop geometry_msgs/Twist -r 10000 '[10, 0, 0]' '[0, 0, .5]' #ROBOT MOVE CMD LINE
    #print "Rotate"
    turtlebot.rotate(-math.pi)
    #turtlebot.driveStraight(0.2, 0.5)
    print (turtlebot._current)


def spinFunc2(turtlebot):
    # testPub = rospy.Publisher('Check', String, queue_size=10)
    # rostopic pub /cmd_vel_mux/input/teleop geometry_msgs/Twist -r 10000 '[10, 0, 0]' '[0, 0, .5]' #ROBOT MOVE CMD LINE
    #print "Rotate"
    turtlebot.rotate(math.pi)
    #turtlebot.driveStraight(0.2, 0.5)
    print (turtlebot._current)

def driveFunc(turtlebot):
    # testPub = rospy.Publisher('Check', String, queue_size=10)
    # rostopic pub /cmd_vel_mux/input/teleop geometry_msgs/Twist -r 10000 '[10, 0, 0]' '[0, 0, .5]' #ROBOT MOVE CMD LINE
    #print "Drive"
    #turtlebot.rotate(math.pi)
    turtlebot.driveStraight(-0.2, 0.5)
    print (turtlebot._current)

def driveFunc2(turtlebot):
    # testPub = rospy.Publisher('Check', String, queue_size=10)
    # rostopic pub /cmd_vel_mux/input/teleop geometry_msgs/Twist -r 10000 '[10, 0, 0]' '[0, 0, .5]' #ROBOT MOVE CMD LINE
    #print "Drive"
    #turtlebot.rotate(math.pi)
    turtlebot.driveStraight(0.2, 0.5)
    print (turtlebot._current)

def nav(goal, turtlebot):
    turtlebot.navToPose(goal)

def run():
    rospy.init_node('TEST')
    map = Map()
    turtlebot=Robot()
    sub = rospy.Subscriber("/map", OccupancyGrid, map.mapCallBack)
    spinFunc(turtlebot)
    spinFunc2(turtlebot)
    """bashCommand = "roslaunch turtlebot_rviz_launchers view_navigation.launch"
    process = subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)
    output, error = process.communicate()"""
    goal = PoseStamped()
    goal.pose.position.x = 1.5
    goal.pose.position.y = 1.5
    while(1):
        #print("thing")
        """spinFunc(turtlebot)
        driveFunc(turtlebot)
        rospy.sleep(2)
        spinFunc2(turtlebot)
        driveFunc2(turtlebot)"""
        nav(goal, turtlebot)






if __name__ == '__main__':

    try:
        run()
    except rospy.ROSInterruptException:
        pass