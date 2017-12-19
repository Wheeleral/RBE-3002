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
        self._current = Pose() # initlize correctly 
        self._odom_list = tf.TransformListener()
        rospy.Timer(rospy.Duration(.1), self.timerCallback)
        self._vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=10) #one or 10  
        rospy.Subscriber('move_base_simple/goal2', PoseStamped, self.navToPose, queue_size=5)
	 	# handle nav goal events
        rospy.Subscriber('mobile_base/events/bumper', BumperEvent, self.readBumper, queue_size=1) 
		# handle bumper events


    def navToPose(self,goal):
        """
            This is a callback function. It should exract data from goal, drive in a striaght line to reach the goal and 
            then spin to match the goal orientation.


	polar coords to get angle to turn from beg to end
	rotate(dist)
	go staight(distanc)
	rotate(other angle)	


        """
 	#self._odom_list.waitForTransform('map', 'base_footprint', rospy.Time(0), rospy.Duration(1.0))
        #transGoal = self._odom_list.transformPose('base_footprint', goal) # transform the nav goal from the global coordinate system to the robot's coordinate system
        origin = copy.deepcopy(self._current) 

        q = [origin.orientation.x,
             origin.orientation.y,
             origin.orientation.z,
             origin.orientation.w] # quaternion nonsense

        (roll, pitch, yaw) = euler_from_quaternion(q)
	qc = [self._current.orientation.x,
              self._current.orientation.y,
              self._current.orientation.z,
              self._current.orientation.w] 
	(rollc, pitchc, yawc) = euler_from_quaternion(qc)
	x=goal.pose.position.x
	y=goal.pose.position.y
	cx=self._current.position.x
	cy=self._current.position.y
	"""
	if(x > self._current.orientation.x) :
		self.driveStraight(1, abs(x-self._current.position.x))
	else :
		self.driveStraight(-1, abs((x-self._current.position.x)))

	if(y > 0) :
		self.rotate(1.57)
	else : 
		self.rotate(-1.57)
	self.driveStraight(1, abs((y-self._current.position.y)))
	"""
	print('current', cx, cy)
	print(x , y )
	theta=math.atan2(y,x)
	print ('angle is ', theta)
	if(y>cy):
		self.rotate(theta)
	else:
		self.rotate(-theta)	
	distance = ((x-self._current.position.x)**2+(y-self._current.position.y)**2)**.5
	print ('distance is ', distance)
	self.driveStraight(1, distance)
	
	

	
        #self._odom_list.waitForTransform('YOUR_STRING_HERE', 'YOUR_STRING_HERE', rospy.Time(0), rospy.Duration(1.0))
        #transGoal = self._odom_list.transformPose('YOUR_STRING_HERE', goal) # transform the nav goal from the global coordinate system to the robot's coordinate system

    def executeTrajectory(self):
	
	self.driveStraight(.5, .40)
	self.rotate(1.57)
	self.driveStraight(.5, .25)
	self.rotate(2.35)
        """
        See lab manual for the dance the robot has to excute
	drive foward 60cm
	turn right 90
	drive foward 45cm
	turn left 135

	self.driveStraight(speed, 60)
	self.rotate(90)
	self.driveStraight(speed, 45)
	self.rotate(135)
	
        """
	


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
	curr=math.sqrt(self._current.position.x**2 + self._current.position.y**2)
	while(curr<(distance+orig-.2) and not rospy.is_shutdown()):

		print ('distance curr is', curr , 'distance' , distance, 'orig' , orig)
		self._vel_pub.publish(move_msg)
		curr=math.sqrt(self._current.position.x**2 + self._current.position.y**2)

	self._vel_pub.publish(stop_msg)


    def driveStraight2(self, speed, distance):
	print "drive straight2"



        # origin = copy.deepcopy(self._current) ##WHAT IS THIS???
	time = speed/distance
        origin = copy.deepcopy(self._current) #hint:  use this ##WHAT IS THIS??? Where you begin
        move_msg = Twist()
    	move_msg.linear.x = speed
    	move_msg.angular.z = 0
    	stop_msg = Twist()
    	stop_msg.linear.x = 0
    	stop_msg.angular.z = 0
	acc=0
    	#publish move message for desired time
	driveStartTime = rospy.Time.now().secs
	#print "Here"
	"""
    	while(rospy.Time().now().secs - driveStartTime < time and not rospy.is_shutdown()):
		#print rospy.Time().now().secs
		#print  self._current.position.x
        	self._vel_pub.publish(move_msg)
    	self._vel_pub.publish(stop_msg)
	"""
	print (self._current.position.x, (distance+origin.position.x)-.2 , ((distance+origin.position.x)-.2)/2)

	orig=math.sqrt(self._current.position.x**2 + self._current.position.y**2)
	curr=math.sqrt(self._current.position.x**2 + self._current.position.y**2)
	while(curr<(distance+orig-.2)/2 and not rospy.is_shutdown()):
		acc+=.005
		if(acc>1):
			acc=1		
		move_msg.linear.x=speed*acc
		print ('distance curr is', curr , 'distance' , distance, 'orig' , orig)
		self._vel_pub.publish(move_msg)
		curr=math.sqrt(self._current.position.x**2 + self._current.position.y**2)
	while((distance+orig-.2)/2<=curr<(distance+orig-.2) and not rospy.is_shutdown()):
		acc-=.0005
		
		move_msg.linear.x=speed*acc
		print ('distance curr is', curr , 'distance' , distance, 'orig' , orig)
		self._vel_pub.publish(move_msg)
		curr=math.sqrt(self._current.position.x**2 + self._current.position.y**2)
		if(acc<0):
			curr=(distance+orig-.2)
	self._vel_pub.publish(stop_msg)

        
    def spinWheels(self, v_left, v_right, time): #how is this bing implemented 
        """
           This method should use differential drive kinematics to compute V and omega (linear x = V, angular z = omega).
           It should then create a ??? message type, and publish it to ??? in order to move the robot
        """

	"""
	compute wheel speeds (linear and angular)
	create a move_msg of type twist
		send the linear velocity
		send the angular velocity
	create a msg to stop the bot (0,0)
	publish the msgs 

	"""
	#paramaters
        diameter = 0.034 # based on wheel track from https://yujinrobot.github.io/kobuki/enAppendixKobukiParameters.html 35 mm
        wheel_base=.23 #

        r = diameter/2 
    	b = wheel_base

    	u = (r / 2) * (v_right + v_left) #linear velocity
    	w = (r / b) * (v_right - v_left) #angluar velocity

    	#create movement and stop messages
    	move_msg = Twist()
    	move_msg.linear.x = u
    	move_msg.angular.z = w
    	stop_msg = Twist()
    	stop_msg.linear.x = 0
    	stop_msg.angular.z = 0
    	#publish move message for desired time
	driveStartTime = rospy.Time.now().secs
    	while(rospy.Time().now().secs - driveStartTime < time and not rospy.is_shutdown()):
		#print rospy.Time().now().secs
		#print  self._current.position.x
        	self._vel_pub.publish(move_msg)
    	self._vel_pub.publish(stop_msg)

        

        
    def rotate(self,angle):
        """
            This method should populate a ??? message type and publish it to ??? in order to spin the robot
        """
        """
	calculate error/desired angle movement
	determine way to turn
	calculate the velocty
	
	create msg anglev type twist
	publish anglev 

	"""
        
        origin = copy.deepcopy(self._current) ##WHAT IS THIS???

        q = [origin.orientation.x,
             origin.orientation.y,
             origin.orientation.z,
             origin.orientation.w] # quaternion nonsense

        (roll, pitch, yaw) = euler_from_quaternion(q)
	qc = [self._current.orientation.x,
              self._current.orientation.y,
              self._current.orientation.z,
              self._current.orientation.w] 
	(rollc, pitchc, yawc) = euler_from_quaternion(qc)
	#paramaters
        diameter = 0.034 # based on wheel track from https://yujinrobot.github.io/kobuki/doxygen/enAppendixKobukiParameters.html 35 mm
        wheel_base=.23 #

        r = diameter/2 
    	b = wheel_base
	if(angle>0):
		v=.2 #.25m/s
	else:
		v=-.2
	
	#V=v/b #rad/sec
	#time=4*angle/V
	
	diff = (angle - yaw)
    	#create movement and stop messages
    	move_msg = Twist()
    	move_msg.linear.x = 0
    	move_msg.angular.z = v
    	stop_msg = Twist()
    	stop_msg.linear.x = 0
    	stop_msg.angular.z = 0
    	#publish move message for desired time
	driveStartTime = rospy.Time.now().secs
	threshold = .05
	#print diff
	if(angle>0):
    		while(diff>threshold and not rospy.is_shutdown()):
			qc = [self._current.orientation.x,
             	      	      self._current.orientation.y,
                              self._current.orientation.z,
                              self._current.orientation.w] 
	        	(rollc, pitchc, yawc) = euler_from_quaternion(qc)		
			#rospy.loginfo
			#print yawc
        		self._vel_pub.publish(move_msg)		
			diff = (angle - abs(yawc - yaw))
    		self._vel_pub.publish(stop_msg)        
	else:
    		while(diff<threshold and not rospy.is_shutdown()):
			qc = [self._current.orientation.x,
             	      	      self._current.orientation.y,
                              self._current.orientation.z,
                              self._current.orientation.w] 
	        	(rollc, pitchc, yawc) = euler_from_quaternion(qc)		
			#rospy.loginfo
			#print yawc
        		self._vel_pub.publish(move_msg)		
			diff = (angle + abs(yawc - yaw))
    		self._vel_pub.publish(stop_msg)  
	print('yawc ' , yawc, 'yaw' ,yaw)



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
        

    def readBumper(self, msg):
        """
        callback function that excutes on a BumperEvent
        """
        #if(msg.bumper == BumperEvent.CENTER) :
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
   	v1 = w * (radius + .5*.352)
    	v2 = w * (radius - .5*.352)
	lin=(v1+v2)/2

	target = 0
	print('vel', v1, v2)
	cy=math.sin(angle)*radius
	cx2=math.cos(angle)*radius
	cx=radius-cx2
	print('vel', v1, v2)
	origin = copy.deepcopy(self._current) 
	print('vel', v1, v2)
        q = [origin.orientation.x,
             origin.orientation.y,
             origin.orientation.z,
             origin.orientation.w] # quaternion nonsense

        (roll, pitch, yaw) = euler_from_quaternion(q)



	while(target == 0) :

		if((self._current.position.x-origin.position.x)<cx):

	        	self.publishTwist(lin, w)
		else:
			self.publishTwist(0,0)
			target = 1;
 
          
		

if __name__ == '__main__':
    print "Starting Lab2"
    rospy.init_node('drive_base')
    turtle = Robot()
    rospy.sleep(2);
    #test function calls here
    #turtle.spinWheels(-.1, .25, 10)
    #turtle.driveStraight(1, 4)
    #turtle.rotate(1.57)
    #turtle.executeTrajectory()
    #turtle.arc(1, 1, 1.57)
    while  not rospy.is_shutdown():
        pass    
