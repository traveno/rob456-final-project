#!/usr/bin/env python3


import rospy
import sys
import numpy as np
from math import atan2, tanh, sqrt, pi

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion

import actionlib
import tf

from lab2.msg import NavTargetAction, NavTargetResult, NavTargetFeedback


class Driver:
	def __init__(self, position_source, threshold=0.1):
		# Goal will be set later. The action server will set the goal; you don't set it directly
		self.goal = None
		self.threshold = threshold

		self.transform_listener = tf.TransformListener()

		# Publisher before subscriber
		self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
		self.target_pub = rospy.Publisher('current_target', Marker, queue_size=1)

		# Subscriber after publisher
		self.sub = rospy.Subscriber('base_scan', LaserScan, self._callback, queue_size=1)

		# Action client
		self.action_server = actionlib.SimpleActionServer('nav_target', NavTargetAction, execute_cb=self._action_callback, auto_start=False)
		self.action_server.start()

	@classmethod
	def zero_twist(cls):
		"""This is a helper class method to create and zero-out a twist"""
		command = Twist()
		command.linear.x = 0.0
		command.linear.y = 0.0
		command.linear.z = 0.0
		command.angular.x = 0.0
		command.angular.y = 0.0
		command.angular.z = 0.0

		return command

	# Respond to the action request.
	def _action_callback(self, goal):
		""" This gets called when an action is received by the action server
		@goal - this is the new goal """
		rospy.loginfo(f'Got an action request for ({goal.goal.point.x:.2f}, {goal.goal.point.y:.2f})')

		# Set the goal.
		self.goal = goal.goal

		# Build a marker for the goal point
		#   - this prints out the green dot in RViz (the current goal)
		marker = Marker()
		marker.header.frame_id = goal.goal.header.frame_id
		marker.header.stamp = rospy.Time.now()
		marker.id = 0
		marker.type = Marker.SPHERE
		marker.action = Marker.ADD
		marker.pose.position = goal.goal.point
		marker.pose.orientation.x = 0.0
		marker.pose.orientation.y = 0.0
		marker.pose.orientation.z = 0.0		
		marker.pose.orientation.w = 1.0
		marker.scale.x = 0.3
		marker.scale.y = 0.3
		marker.scale.z = 0.3
		marker.color.r = 0.0
		marker.color.g = 1.0
		marker.color.b = 0.0
		marker.color.a = 1.0

		# Wait until we're at the goal.  Once we get there, the callback that drives the robot will set self.goal
		# to None.
		while self.goal:
			self.target_pub.publish(marker)
			rospy.sleep(0.1)

		rospy.loginfo('Action completed')

		# Build a result to send back
		result = NavTargetResult()
		result.success.data = True

		self.action_server.set_succeeded(result)

		# Get rid of the marker
		marker.action = Marker.DELETE
		self.target_pub.publish(marker)

	def _callback(self, lidar):
		# If we have a goal, then act on it, otherwise stay still
		if self.goal:
			# Update the timestamp on the goal and figure out where it it now in the base_link frame.
			self.goal.header.stamp = rospy.Time.now()
			target = self.transform_listener.transformPoint('base_link', self.goal)

			# rospy.loginfo(f'Target: ({target.point.x:.2f}, {target.point.y:.2f})')

			# Are we close enough?  If so, then remove the goal and stop
			distance = sqrt(target.point.x ** 2 + target.point.y ** 2)

			feedback = NavTargetFeedback()
			feedback.distance.data = distance
			self.action_server.publish_feedback(feedback)

			if distance < self.threshold:
				self.goal = None
				command = Driver.zero_twist()
			else:
				command = self.get_twist((target.point.x, target.point.y), lidar)
		else:
			command = Driver.zero_twist()

		self.cmd_pub.publish(command)


	# This function returns an bias of "go left" or "go right"
	def calc_turn_influence(self, lidar, dist_threshold, clamp):
		# First, limit lidar data that is below our distance threshold (which is 3)
		close_scans = np.hstack(np.where(np.array(lidar.ranges) < dist_threshold))

		# Keep track of "total" bias
		influence = 0
		for i in close_scans:
			# Left side
			if i < 90:
				# We know the angle of the scan by its index
				# We use this to make scans which are more forwards more important
				# Scans that are off to the side have less of an influence here
				
				# Also, objects that are closer have more influence, so
				# I subtract the scan's range from the threshold.

				# I divide by an arbitrary amount to scale the value down
				influence += (i / 90) * ((dist_threshold - lidar.ranges[i]) / 10)
			# Right side
			else:
				# Same process here, but objects on the right side subtract from the
				# overall influence, which causes the robot to turn left.
				influence -= (1 - ((i - 90) / 90)) * ((dist_threshold - lidar.ranges[i]) / 10)

		# Clamp the value if necessary
		return max(min(clamp, influence), -clamp)


	# This is the function that controls the robot.
	#
	# Inputs:
	# 	target:	a tuple with the (x, y) coordinates of the target point, in the robot's coordinate frame (base_link).
	# 			x-axis is forward, y-axis is to the left.
	# 	lidar:	a LaserScan message with the current data from the LiDAR.  Use this for obstacle avoidance.
	#           This is the same as your go and stop code
	def get_twist(self, target, lidar):
		command = Driver.zero_twist()

		# Calculate the angle between the robot and the goal
		theta = atan2(target[1], target[0])

		# Calculate the distance that the robot is from the goal
		distance = sqrt(target[0] ** 2 + target[1] ** 2) + 0.1

		# Define a distance we consider "close"
		close_distance = 1

		# If we're approaching a goal, limit our speed
		# Otherwise, we approach quickly
		if (distance < close_distance):
			command.linear.x = 0.2
		else:
			command.linear.x = min(0.5, 1 * distance)

		# How close should we start considering obstacles?
		# I determined this to be 3
		scan_distance = 3
		# Calculate the influence of the obstances in the robot's view
		obstacle_influence = self.calc_turn_influence(lidar, scan_distance, 5)

		# If any lidar scan comes in close, apply bias
		# from our surrouding obstances to navigate
		if (min(lidar.ranges) < close_distance):
			command.angular.z = 2 * (theta / pi) + (obstacle_influence / 2)
		else:
			command.angular.z = 4 * (theta / pi)
			
		# Debugging info
		rospy.loginfo(f'Target: ({target[0]:.2f}, {target[1]:.2f}), Theta: {theta}, Distance: {distance}, Turn: {command.angular.z}')
		return command

if __name__ == '__main__':
	rospy.init_node('driver', argv=sys.argv)

	driver = Driver('odom')

	rospy.spin()


