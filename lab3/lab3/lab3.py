#!/usr/bin/env python

## Brooks Kindle
## 11257408
## Kyle Vandelac
## 11193077
## Cpts 483
## Lab3
## lab3.py -- makes the actual turtlebot move in a square and then a spiral!

import rospy
from math import pi as PI

#use a message type "geometry_msgs/Twist"
from geometry_msgs.msg import Twist

x_speed = .50 #.50m/s

def driveStraight(speed, distance):
    """moves the turtlebot in a straight line, speed is in m/s and distance is
    in m"""
    p = rospy.Publisher("cmd_vel_mux/input/teleop", Twist)
    twist = Twist()

    #determine incremental speed to move at
    max_speed = 0.50 #0.50m/s
    if speed > max_speed: #adjust movement speed
        speed = max_speed
    total_time = float(distance) / speed #time in seconds to travel

    twist.linear.x = speed
    twist.angular.z = 0
    dist_traveled = 0
    while dist_traveled < distance:
        for i in range(10):
            p.publish(twist)
            rospy.sleep(0.1) #0.1 * 10 = 1 second
        dist_traveled += speed

    twist = Twist()
    p.publish(twist) #stop movement

def rotate(angle):
    """rotates the robot a given number of radians"""
    p = rospy.Publisher("cmd_vel_mux/input/teleop", Twist)
    twist = Twist()

    twist.angular.z = angle
    twist.linear.x = 0
    #rospy.sleep(1.0) #wait one second to turn PI/2 rads (which is 90 degrees)
    for k in range(15):
        p.publish(twist)
        rospy.sleep(0.1) # 10 * 0.1 = 1 second

    twist = Twist()
    p.publish(twist) #stop turning

def driveSquare(sideDistance):
    p = rospy.Publisher("cmd_vel_mux/input/teleop", Twist)
    twist = Twist()
    #move the robot in a square
    for i in range(4):
            #move forward
            rospy.loginfo("Movin' forward me hearties!")
            driveStraight(x_speed, x_speed) #only move forward x_speed
            rospy.loginfo("Turning #{}".format(i+1))
            #turn 90 degrees
            rotate(PI/2)

    #create a new empty message
    twist = Twist() #constructor defaults values to 0
    rospy.loginfo("Stopping!")

def driveSpiral():
	p = rospy.Publisher("cmd_vel_mux/input/teleop", Twist)
	twist = Twist()
	i = 0
	j = 0
	speed = .4
	turnSpeed = 1.6
	while(i < 300):
		if i == 15:
			j += 1
			i -= 1
			twist.linear.x = speed
			twist.angular.z = 0.1
			if j == 5:
				j = 0
				i = -1
				turnSpeed -= .1
				#speed += .05
			#end if
		#end if
		else:
			twist.linear.x = speed
			twist.angular.z = turnSpeed
		#end else
		p.publish(twist)
		rospy.sleep(.1)
		i += 1
	#end while
	twist = Twist()
	p.publish(twist)

def executeTrajectory():
	"""drives the robot in a square, and then in a spiral"""
	driveSquare(.5)
	driveSpiral()

def main():
    """the main function, chuu chuu!"""
    rospy.init_node("brooks_square") #init a node with a name
    p = rospy.Publisher("cmd_vel_mux/input/teleop", Twist) #public to cmd_vel

    #create a twist message to publish
    twist = Twist()
    twist.linear.x = x_speed

    #the turtlebot can only move forward, in order to move sideways, we
    #must rotate the turtlebot about the z axis. This is why the linear
    #y and z axis rotations are 0
    twist.linear.y = 0 #not used by turtlebot
    twist.linear.z = 0 #not used by turtlebot
    twist.angular.x = 0 #not used by turtlebot
    twist.angular.y = 0 #not used by turtlebot

    twist.angular.z = 0 #no rotation, just moving forward

    #move and publish the message
    rospy.loginfo("About to be moving forward!")
    executeTrajectory() #do it!

    #publish empty message so that we clear twist
    p.publish(twist)

if __name__ == "__main__":
    main()
