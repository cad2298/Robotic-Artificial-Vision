#! /usr/bin/python
# Copyright (c) 2013 Mak Nazecic-Andrlon 
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.


from __future__ import division

from pyorca import Agent, get_avoidance_velocity, orca, normalized, perp
from numpy import array, rint, linspace, pi, cos, sin, isnan, sqrt, atan2
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import tf
import rospy
import itertools
import random

class orca_ros():
	def __init__(self):
		self.delta_t = 0.1 #TODO ajustar valor
		self.v_orca = None
		self.radius = 0.4 
		self.max_speed = rospy.get_param('~max_speed', default=1)
		# Create a publisher which can "talk" to TurtleBot and tell it to move
		# Tip: You may need to change cmd_vel_mux/input/navi to /cmd_vel if you're not using TurtleBot2
		self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
		rospy.loginfo("Process finished 1")
		self.marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=10)
		rospy.loginfo("Process finished 2")

		self.laser_init = False

		self.cmd_vel_subscriber = rospy.Subscriber("/cmd_vel/tracker", Twist, self.callback_cmd_vel)
		self.laser_scan = rospy.Subscriber("/scan", LaserScan, self.callback_scan)
		rospy.loginfo("Process finished 3")

		self.listener = tf.TransformListener()

	def callback_cmd_vel(self, data):
		#Ej contemplar casos en los que la velocidad pueda ser negativa.
		a_vel = data.angular.z * self.delta_t
		self.v_orca = (data.linear.x * cos(a_vel), data.linear.x * sin(a_vel))
		self.cmd_vel = data
		
		

	def callback_scan(self, data):
		if self.v_orca == None :
			return
		if fabs(self.cmd_vel.linear.x) > 0.02 :
			self.laser_scan = data
			self.laser_init = True
			rospy.loginfo("Laser received " + str(len(data.ranges)))

			#TODO PARAMETRIZAR TODO
			agents = []

			agents.append(Agent((0., 0.), self.v_orca, self.radius, self.max_speed, self.v_orca)
			for i in range(1, len(self.laser_scan.ranges), 40):
				angle = self.laser_scan.angle_min + i * self.laser_scan.angle_increment
				d = self.laser_scan.ranges[i]
				if not isnan(d) :
					rospy.loginfo(str(angle))
					obstacle = Point(d * cos(angle), d * sin(angle), 0)
		    		marker = Marker(
			        	type=Marker.SPHERE,
			        	id=i,
			        	lifetime=rospy.Duration(1.5),
			        	pose=Pose(obstacle, Quaternion(0, 0, 0, 1)),
			        	scale=Vector3(0.5, 0.5, 0.5),
			        	header=Header(frame_id='base_link'),
			        	color=ColorRGBA(0.0, 1.0, 0.0, 0.8))
	    			self.marker_publisher.publish(marker)
	    			pos = (obstacle.x, obstacle.y)
	    			agents.append(Agent(pos, (0., 0.), 0.05, 0.1, (0., 0.)))
	    	v = orca_apply(agents)
	    	self.cmd_vel.linear.x = sqrt(v[0]**2 + v[1]**2)
	    	self.cmd_vel.angular.z = atan2(v[1], v[0]) / self.delta_t

	def orca_apply(self, agents):
		dt = self.delta_t
		new_vels = [None] * len(agents)
        for i, agent in enumerate(agents):
            candidates = agents[:i] + agents[i + 1:]
            # print(candidates)
            new_vels[i], all_lines[i] = orca(agent, candidates, 5, dt)
            # print(i, agent.velocity)
        return new_vels[0]

		


if __name__ == '__main__':
    try:
		# initiliaze
       	rospy.init_node('orca', anonymous=False)

		# tell user how to stop TurtleBot
		rospy.loginfo("To stop TurtleBot CTRL + C")

       	orca = orca_ros()

		#TurtleBot will stop if we don't keep telling it to move.  How often should we tell it to move? 10 HZ
       	r = rospy.Rate(10);
			
	 	# What function to call when you ctrl + c    
       	#rospy.on_shutdown(robot.shutdown)

		while not rospy.is_shutdown():
		    rospy.loginfo("Loop")
		    # wait for 0.1 seconds (10 HZ) and publish again
		    r.sleep()
	except:
		rospy.loginfo("robotcontrol node terminated.")

	


