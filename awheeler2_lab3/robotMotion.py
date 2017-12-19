#!/usr/bin/env python
import rospy, tf, copy, math
from kobuki_msgs.msg import BumperEvent
from geometry_msgs.msg import Twist, Pose, PoseStamped
from tf.transformations import euler_from_quaternion
import numpy as np
from std_msgs.msg import String
from nav_msgs.msg import Odometry


class Robot:
    def __init__(self):

        """
            This constructor sets up class variables and pubs/subs
		cmd_vel_mux/input/teleop
        """
        self._current = Pose()  # initlize correctly
        self._odom_list = tf.TransformListener()
        rospy.Timer(rospy.Duration(.1), self.timerCallback)
        self._vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=10)  # one or 10
        rospy.Subscriber('move_base_simple/goal1', PoseStamped, self.navToPose, queue_size=5)
df        rospy.Subscriber('mobile_base/events/bumper', BumperEvent, self.readBumper, queue_size=1)
        # handle bumper events
        self.rate = rospy.Rate(1)
        self.vel_msg = Twist()
        self.vel_msg.linear.x = 0
        self.vel_msg.linear.y = 0
        self.vel_msg.linear.z = 0
        self.vel_msg.angular.x = 0
        self.vel_msg.angular.y = 0
        self.vel_msg.angular.z = 0

    def navToPose(self, dsgoal):
        # self._odom_list.waitForTransform('map', 'base_footprint', rospy.Time(0), rospy.Duration(1.0))
        # transGoal = self._odom_list.transformPose('base_footprint', goal) # transform the nav goal from the global coordinate system to the robot's coordinate system
        origin = copy.deepcopy(self._current)

        q = [origin.orientation.x,
             origin.orientation.y,
             origin.orientation.z,
             origin.orientation.w]  # quaternion nonsense

        (roll, pitch, yaw) = euler_from_quaternion(q)
        qc = [self._current.orientation.x,
              self._current.orientation.y,
              self._current.orientation.z,
              self._current.orientation.w]
        (rollc, pitchc, yawc) = euler_from_quaternion(qc)
        x = goal.pose.position.x
        y = goal.pose.position.y
        cx = self._current.position.x
        cy = self._current.position.y

        print('current', cx, cy)
        print(x, y)
        theta = math.atan2(y, x)
        print ('angle is ', theta)
        if (y > cy):
            self.rotate(theta)
        else:
            self.rotate(-theta)
        distance = ((x - self._current.position.x) ** 2 + (y - self._current.position.y) ** 2) ** .5
        print ('distance is ', distance)
        self.driveStraight(1, distance)

    def initPose(self):
        origin = copy.deepcopy(self._current)

        q = [origin.orientation.x,
             origin.orientation.y,
             origin.orientation.z,
             origin.orientation.w]  # quaternion nonsense

        (roll, pitch, yaw) = euler_from_quaternion(q)
        return (self._current.position.x, self._current.position.y, yaw)

        # self._odom_list.waitForTransform('YOUR_STRING_HERE', 'YOUR_STRING_HERE', rospy.Time(0), rospy.Duration(1.0))

    # transGoal = self._odom_list.transformPose('YOUR_STRING_HERE', goal) # transform the nav goal from the global coordinate system to the robot's coordinate system

    def executeTrajectory(self):

        self.driveStraight(.5, .40)
        self.rotate(1.57)
        self.driveStraight(.5, .25)
        self.rotate(2.35)

    def driveStraight(self, speed, distance):
        #print "drive straight"

        time = speed / distance
        origin = copy.deepcopy(self._current)  # Where you begin
        move_msg = Twist()
        move_msg.linear.x = speed
        move_msg.angular.z = 0
        stop_msg = Twist()
        stop_msg.linear.x = 0
        stop_msg.angular.z = 0

        driveStartTime = rospy.Time.now().secs

        print (self._current.position.x, (distance + origin.position.x) - .2, ((distance + origin.position.x) - .2) / 2)

        orig = math.sqrt(self._current.position.x ** 2 + self._current.position.y ** 2)
        curr = math.sqrt(
            (self._current.position.x - origin.position.x) ** 2 + (self._current.position.y - origin.position.y) ** 2)
        while (curr < (distance - .15) and not rospy.is_shutdown()):
            
            #print ('distance curr is', curr, 'distance', distance, 'orig', orig, "pos", self._current.position.x, self._current.position.y)
            self._vel_pub.publish(move_msg)
            curr = math.sqrt((self._current.position.x - origin.position.x) ** 2 + (
                self._current.position.y - origin.position.y) ** 2)
            #print "Still Here"
        print ("curr", curr, "Distance ", distance)
        #print "HERE"
        self._vel_pub.publish(stop_msg)

    def spinWheels(self, v_left, v_right, time):
        """
           This method should use differential drive kinematics to compute V and omega (linear x = V, angular z = omega).
        """

        # paramaters
        diameter = 0.034  # based on wheel track from https://yujinrobot.github.io/kobuki/enAppendixKobukiParameters.html 35 mm
        wheel_base = .23  #

        r = diameter / 2
        b = wheel_base

        u = (r / 2) * (v_right + v_left)  # linear velocity
        w = (r / b) * (v_right - v_left)  # angluar velocity

        # create movement and stop messages
        move_msg = Twist()
        move_msg.linear.x = u
        move_msg.angular.z = w
        stop_msg = Twist()
        stop_msg.linear.x = 0
        stop_msg.angular.z = 0
        # publish move message for desired time
        driveStartTime = rospy.Time.now().secs
        while (rospy.Time().now().secs - driveStartTime < time and not rospy.is_shutdown()):
            # print rospy.Time().now().secs
            # print  self._current.position.x
            self._vel_pub.publish(move_msg)
        self._vel_pub.publish(stop_msg)

    def rotate(self, angle):

        #print("start")
        diff = 0
        origin = copy.deepcopy(self._current)

        q = [origin.orientation.x,
             origin.orientation.y,
             origin.orientation.z,
             origin.orientation.w]  # quaternion nonsense

        (roll, pitch, yaw) = euler_from_quaternion(q)

        endAngle = angle + yaw
        while (endAngle > math.pi):
            endAngle = endAngle - 2 * math.pi
        while (endAngle < -math.pi):
            endAngle = endAngle + 2 * math.pi
        self.vel_msg.linear.x = 0
        self.vel_msg.angular.z = (endAngle-yaw)/2
        while (endAngle > (yaw+.08) or endAngle < (yaw-.08)):
            #rospy.loginfo(self.vel_msg)
            self._vel_pub.publish(self.vel_msg)
            q = [self._current.orientation.x,
                self._current.orientation.y,
                self._current.orientation.z,
                self._current.orientation.w]  # quaternion nonsense
            (roll, pitch, yaw) = euler_from_quaternion(q)

        """else:
            self.vel_msg.angular.z = -.35
            while (endAngle < (yaw-.1)):
                #rospy.loginfo(self.vel_msg)
                self._vel_pub.publish(self.vel_msg)
                q = [self._current.orientation.x,
                     self._current.orientation.y,
                     self._current.orientation.z,
                     self._current.orientation.w]  # quaternion nonsense
                (roll, pitch, yaw) = euler_from_quaternion(q)"""


    def timerCallback(self, evprent):
        """
            This is a callback that runs every 0.1s.
            Updates this instance of Robot's internal position variable (self._current)
		tf_monitor list transofrms aviable
		tf_echo output data to terminal
        """
        self._odom_list.waitForTransform('map', 'base_footprint', rospy.Time(0), rospy.Duration(1.0))
        (position, orientation) = self._odom_list.lookupTransform('map', 'base_footprint', rospy.Time(
            0))  # finds the position and oriention of two objects relative to each other (hint: this returns arrays, while Pose uses lists)
        self._current.position.x = position[0]
        self._current.position.y = position[1]

        self._current.orientation.x = orientation[0]
        self._current.orientation.y = orientation[1]
        self._current.orientation.z = orientation[2]
        self._current.orientation.w = orientation[3]
        q = [self._current.orientation.x,
             self._current.orientation.y,
             self._current.orientation.z,
             self._current.orientation.w]  # quaternion nonsense

        (roll, pitch, yaw) = euler_from_quaternion(q)

    def readBumper(self, msg):
        """
        callback function that executes on a BumperEvent
        """
        # if(msg.bumper == BumperEvent.CENTER) :
        self.executeTrajectory()

    # helper functions
    def publishTwist(self, lin_Vel, ang_Vel):
        """Send a movement (twist) message."""

        msg = Twist()
        msg.linear.x = lin_Vel
        msg.angular.z = ang_Vel
        self._vel_pub.publish(msg)

    def planTraj(self, dis, speed):
        """
            Bonus Question:  compute the coefs for a cubic polynomial (hint:  you did this in 3001)
        """

    def arc(self, radius, speed, angle):
        w = speed / radius
        v1 = w * (radius + .5 * .352)
        v2 = w * (radius - .5 * .352)
        lin = (v1 + v2) / 2

        target = 0
        print('vel', v1, v2)
        cy = math.sin(angle) * radius
        cx2 = math.cos(angle) * radius
        cx = radius - cx2
        print('vel', v1, v2)
        origin = copy.deepcopy(self._current)
        print('vel', v1, v2)
        q = [origin.orientation.x,
             origin.orientation.y,
             origin.orientation.z,
             origin.orientation.w]  # quaternion nonsense

        (roll, pitch, yaw) = euler_from_quaternion(q)

        while (target == 0):

            if ((self._current.position.x - origin.position.x) < cx):

                self.publishTwist(lin, w)
            else:
                self.publishTwist(0, 0)
                target = 1

    ''' right return 1
    up return 2
    left return 3
    down return 4'''
    def beginingRotate(self, dir, yaw):
        curr =dir
        if (curr == 1):
            desiredAngle = 0 # rotate to 0
        elif (curr == 2):
            desiredAngle = (0.93 * (math.pi / 2)) # rotate to 90 deg left
        elif (curr == 3):
            desiredAngle = math.pi # rotate 180
        elif (curr == 4):
            desiredAngle = (-0.60 * (math.pi / 2)) # rotate -90 deg right
        if ((desiredAngle - yaw) > 0):
            self.rotate(desiredAngle - yaw)  # maybe?
        else:
            self.rotate(desiredAngle + yaw)


    def aStarStart(self, directions, yaw):
        curr = directions.pop(0)
        self.beginingRotate(curr, yaw)

    def aStarMove(self, curr, next, yaw):
        #print "Movement"
        distance=.35
        #desiredAngle=yaw
        #curr = directions.pop(0)
        #needs to rotate to correct pos in beg
        #self.beginingRotate(curr, yaw)


        print ("curr", curr)

        if (curr ==next): #Go Straight One Square
            print "straight"
            self.driveStraight(.5, distance)
        elif((curr+1)%4 ==next): #Turn Left; Go Straight One Square
            print "left"
            self.rotate(math.pi/2)
            self.driveStraight(.5, distance)
        elif((curr-1)%4 ==next or (curr ==1 and next ==4)): #Turn Right; Go Straight One Square
            print "right"
            self.rotate(  -math.pi / 2)
            self.driveStraight(.5, distance)
        else:
            self.driveStraight(.5, distance)

        """for item in directions:
                    print curr
                    next = item

                    if (curr ==next): #Go Straight One Square
                        print "straight"
                        self.driveStraight(.5, distance)
                    elif((curr+1)%4 ==next): #Turn Left; Go Straight One Square
                        print "left"
                        self.rotate(math.pi/2)
                        self.driveStraight(.5, distance)
                    elif((curr-1)%4 ==next or (curr ==1 and next ==4)): #Turn Right; Go Straight One Square
                        print "right"
                        self.rotate(3*math.pi / 2)
                        self.driveStraight(.5, distance)
                    else:
                        self.driveStraight(.5, distance)
                    curr=next"""


'''
if __name__ == '__main__':
    print "Starting Lab2"
    rospy.init_node('drive_base')
    turtle = Robot()
    rospy.sleep(2);
	# turtle.dance()
    while not rospy.is_shutdown():
        pass    
'''