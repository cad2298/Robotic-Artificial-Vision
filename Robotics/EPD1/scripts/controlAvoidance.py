#!/usr/bin/env python



# A very basic TurtleBot script that moves TurtleBot forward unless there is an obstacle close enough. Press CTRL + C to stop.  To run:
# On TurtleBot:
# rosrun <package> controlAvoidance.py

import sys
import math
import rospy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class Turtlebot():
    def __init__(self):
       
        
	# Create a publisher which can "talk" to TurtleBot and tell it to move
        # Tip: You may need to change cmd_vel_mux/input/navi to /cmd_vel if you're not using TurtleBot2
        self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)

	self.min_distance_from_obs = 1.0
	self.laser_init = False
 
	rospy.Subscriber("/scan", LaserScan, self.callback)
     
	
        
    def go_forward(self):
	
	angular = 0.0
        linear = 0.2
	if self.laser_init:
		for dist in self.laser.ranges:
			if dist < self.min_distance_from_obs:
				#linear = 0.0
				#angular = 0.2
				#break
				if self.laser.ranges[0] < self.laser.ranges[len(self.laser.ranges)-1]:
					linear = 0.0
					angular = 0.2
					break
				else:
					linear = 0.0
					angular = -0.2
					break

	rospy.loginfo("Velocity %f", linear)
	
	self.publish(linear,angular)

    def publish(self,lin_vel, ang_vel):
	# Twist is a datatype for velocity
        move_cmd = Twist()
	# let's go forward at 0.2 m/s
        move_cmd.linear.x = lin_vel
	# let's turn at 0 radians/s
	move_cmd.angular.z = ang_vel

	self.cmd_vel.publish(move_cmd)


    def callback(self,data):
	self.laser = data
	self.laser_init = True
	rospy.loginfo("Laser received " + str(len(data.ranges)))
        
    def shutdown(self):
        # stop turtlebot
        rospy.loginfo("Stop TurtleBot")
	# a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
        self.cmd_vel.publish(Twist())
	# sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(1)
 
if __name__ == '__main__':
    try:
	 # initiliaze
        rospy.init_node('robotcontrolavoidance', anonymous=False)

	# tell user how to stop TurtleBot
	rospy.loginfo("To stop TurtleBot CTRL + C")

        robot=Turtlebot()
	 # What function to call when you ctrl + c    
        rospy.on_shutdown(robot.shutdown)

	#TurtleBot will stop if we don't keep telling it to move.  How often should we tell it to move? 10 HZ
        r = rospy.Rate(10);

	# as long as you haven't ctrl + c keeping doing...
        while not rospy.is_shutdown():
	    rospy.loginfo("Loop")
	    # publish the velocity
            robot.go_forward()

	    # wait for 0.1 seconds (10 HZ) and publish again
            r.sleep()

    except:
        rospy.loginfo("robotcontrolavoidance node terminated.")
