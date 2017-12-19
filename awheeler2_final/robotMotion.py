#!/usr/bin/env python
import rospy, tf, copy, math
from kobuki_msgs.msg import BumperEvent
from geometry_msgs.msg import Twist, Pose, PoseStamped
from tf.transformations import euler_from_quaternion
import numpy as np
from std_msgs.msg import String
from nav_msgs.msg import Odometry


#TODO Obstacle Exapansion


class Robot:
    def __init__(self):

        """
            This constructor sets up class variables and pubs/subs
		cmd_vel_mux/input/teleop
        """
        self._current = Pose()  # initlize correctly
        self._odom_list = tf.TransformListener()
        rospy.Timer(rospy.Duration(.1), self.timerCallback)
        self._vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)  # one or 10
        rospy.Subscriber('move_base_simple/goal1', PoseStamped, self.navToPose, queue_size=5)
        # handle nav goal events
        rospy.Subscriber('mobile_base/events/bumper', BumperEvent, self.readBumper, queue_size=1)
        # handle bumper events
        self.rate = rospy.Rate(1)
        self.vel_msg = Twist()
        self.vel_msg.linear.x = 0
        self.vel_msg.linear.y = 0
        self.vel_msg.linear.z = 0
        self.vel_msg.angular.x = 0
        self.vel_msg.angular.y = 0
        self.vel_msg.angular.z = 0

    def navToPose(self, goal):
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
        cx = origin.position.x
        cy = self._current.position.y

        print('current', cx, cy)
        print(x, y)
        theta = math.atan2(y-cy, x-cx)
        print ('angle is ', theta)
        self.rotate(theta)
        distance = (((x - cx) ** 2) + ((y - cy) ** 2)) ** .5
        print ('distance is ', distance)
        self.driveStraight(0.5, distance)

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

        #print (self._current.position.x, (distance + origin.position.x) - .2, ((distance + origin.position.x) - .2) / 2)

        orig = math.sqrt(self._current.position.x ** 2 + self._current.position.y ** 2)
        curr = math.sqrt(
            (self._current.position.x - origin.position.x) ** 2 + (self._current.position.y - origin.position.y) ** 2)
        while (curr < (distance - .15) and not rospy.is_shutdown()):
            
            #print ('distance curr is', curr, 'distance', distance, 'orig', orig, "pos", self._current.position.x, self._current.position.y)
            self._vel_pub.publish(move_msg)
            curr = math.sqrt((self._current.position.x - origin.position.x) ** 2 + (
                self._current.position.y - origin.position.y) ** 2)
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
        stop_msg = Twist()
        stop_msg.linear.x = 0
        stop_msg.angular.z = 0
        print("rotate function")
        diff = 0
        origin = copy.deepcopy(self._current)

        q = [origin.orientation.x,
             origin.orientation.y,
             origin.orientation.z,
             origin.orientation.w]  # quaternion nonsense

        (roll, pitch, yaw) = euler_from_quaternion(q)

        error = angle- yaw
        self.vel_msg.angular.z =.7
        if(error < 0):
            self.vel_msg.angular.z = -1.0 * self.vel_msg.angular.z
        errorincrease =0
        lastError = error
        while ((abs(error)> 0.2)):
            self.vel_msg.angular.z = error
            if(abs(error) > abs(lastError)):
                errorincrease+=1
            else:
                errorincrease=0
            if(errorincrease>2):
                self.vel_msg.angular.z =-1.0 * self.vel_msg.angular.z
                errorincrease=0
            lastError=error
            self._vel_pub.publish(self.vel_msg)
            q = [self._current.orientation.x,
                 self._current.orientation.y,
                 self._current.orientation.z,
                 self._current.orientation.w]  # quaternion nonsense
            (roll, pitch, yaw) = euler_from_quaternion(q)
            error = angle - yaw
            rospy.sleep(.05)
            #print("angle:", angle, " current yaw:", yaw)

        self._vel_pub.publish(stop_msg)
        print("Rotation stopped")



    def timerCallback(self, evprent):

        self._odom_list.waitForTransform('map', 'base_footprint', rospy.Time(0), rospy.Duration(1.0))
        (position, orientation) = self._odom_list.lookupTransform('map', 'base_footprint', rospy.Time(
            0))  # finds the position and oriention of two objects relative to each other (hint: this returns arrays, while Pose uses lists)
        #print("input:", position[0], position[1])
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
        print("dir=",curr)
        if (curr == 1):
            desiredAngle = 0 # rotate to 0
        elif (curr == 2):
            desiredAngle = ((math.pi / 2)) # rotate to 90 deg left
        elif (curr == 3):
            desiredAngle = math.pi # rotate 180
        elif (curr == 4):
            desiredAngle = (-(math.pi / 2)) # rotate -90 deg right

        if ((desiredAngle - yaw) > 0):
            #self.rotate(desiredAngle-yaw )  # maybe?
            self.rotate(desiredAngle)
        else:
            self.rotate(desiredAngle)



    def aStarStart(self, directions, yaw):
        print("aStar Start")
        curr = directions.pop(0)
        self.beginingRotate(curr, yaw)

    def aStarMove(self, curr, next, yaw):
        #print "Movement"
        #distance=.35
        distance =.2
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
            #self.rotate(math.pi/2)
            #self.rotate(math.pi / 2)
            self.driveStraight(.5, distance)
        elif((curr-1)%4 ==next or (curr ==1 and next ==4)): #Turn Right; Go Straight One Square
            print "right"
            #self.rotate(  -math.pi / 2)
            #self.rotate(-math.pi / 2)
            self.driveStraight(.5, distance)
        else:
            self.driveStraight(.5, distance)

    def backup(self):
        distance = .35

        self.driveStraight(-.5, distance)

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