#!/usr/bin/env python3

import sys
import rospy
import numpy as np
from std_msgs.msg import Bool

from new_driver import Driver

from math import atan2, pi, sqrt

STUCK_THRESHOLD = 10

class StudentDriver(Driver):
	'''
	This class implements the logic to move the robot to a specific place in the world.  All of the
	interesting functionality is hidden in the parent class.
	'''
	def __init__(self, threshold=0.5):
		super().__init__('odom')
		# Set the threshold to a reasonable number
		self._threshold = threshold
		self._previous_target = None
		self._target_callback_count = 0

		# publisher message to indicate if the robot is stuck
		self._stuck_pub = rospy.Publisher('stuck', Bool, queue_size=10)

	def detect_corners(self, lidar_data):
		'''
		This function was created to detect a corner from the LiDAR data. It is supplemental to the close_enough function.

		The main idea for the function is to check if the dot product between two consecutive vectors is close to zero. 
		This would indicate that the vectors are perpendicular to each other, which would indicate a corner.

		parameters: lidar data = array of (x,y) coordinates of the lidar points
		'''

		corners = []
		for i in range(1, len(lidar_data) - 1):
			# Create vectors from consecutive lidar points
			vector1 = np.array(lidar_data[i]) - np.array(lidar_data[i - 1])
			vector2 = np.array(lidar_data[i + 1]) - np.array(lidar_data[i])

			# Normalize the vectors
			vector1_norm = vector1 / np.linalg.norm(vector1)
			vector2_norm = vector2 / np.linalg.norm(vector2)

			# Calculate dot product
			dot_product = np.dot(vector1_norm, vector2_norm)

			# Check if dot product is close to zero
			if np.isclose(dot_product, 0, atol=1e-2):
				corners.append(lidar_data[i])

		return (corners != [])

	def close_enough_to_waypoint(self, distance, target, lidar):
		'''
		This function is called periodically if there is a waypoint set. This function now includes
		LiDAR data analysis to make smarter decisions about stopping criteria, taking into account
		the proximity of obstacles in addition to the distance to the target waypoint.

		Parameters:
		- distance: Current distance to the target waypoint.
		- target: Target waypoint coordinates.
		- lidar: LiDAR data representing the surrounding environment.
		'''
		# Check if the robot is stuck
		if self._previous_target == target:
			self._target_callback_count += 1
		else:
			self._previous_target = target
			self._target_callback_count = 0

		# If the robot is stuck, then it is likely in a corner
		if self._target_callback_count > STUCK_THRESHOLD:
			self._stuck_pub.publish(True)
			rospy.loginfo(f'I am stuck in one spot for too long!, wiping waypoints!')
			return False

		safe_distance = 0.5 # 0.5 meters

		ranges = np.array(lidar.ranges)

		# create an array of (x,y) coordinates of the lidar points
		thetas = np.linspace(lidar.angle_min, lidar.angle_max, len(lidar.ranges))
		x_values = ranges * np.cos(thetas)
		y_values = ranges * np.sin(thetas)
		lidar_points = np.column_stack((x_values, y_values))

		# Check if within the simple distance threshold.
		if distance < self._threshold:
			# check to see if robot is in a corner
			if self.detect_corners(lidar_points):  # corner threshold = sensitivity
				self._stuck_pub.publish(True) # tell the controller that the robot is stuck
				rospy.loginfo(f'I am stuck in a corner, wiping waypoints!')

			# get the closest distance to the front of the robot
			front_indices = np.where(np.abs(y_values) < 0.22) # robot is 0.19 (half), increased to .22 to provide a moe for the robot
			front_ranges = ranges[front_indices]

			shortest = np.min(front_ranges)

			# Check if there are no obstacles within the safe distance.
			if shortest >= (safe_distance + 0.19): 
				self._stuck_pub.publish(False) # tell the controller that the robot is not stuck
				return True
		self._stuck_pub.publish(False) # tell the controller that the robot is not stuck
		return False

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
				influence -= (1 - ((i - 90) / 90)) * (
					(dist_threshold - lidar.ranges[i]) / 10
				)

		# Clamp the value if necessary
		return max(min(clamp, influence), -clamp)

	def get_twist(self, target, lidar):
		rospy.loginfo(f'I am calculating a new twist!')
		"""
		This function is called whenever there a current target is set and there is a lidar data
		available.  This is where you should put your code for moving the robot.  The target point
		is in the robot's coordinate frame.  The x-axis is positive-forwards, and the y-axis is
		positive to the left.

		The example sets constant velocities, which is clearly the wrong thing to do.  Replace this
		code with something that moves the robot more intelligently.

		Parameters:
				target:		The current target point, in the coordinate frame of the robot (base_link) as
										an (x, y) tuple.
				lidar:		A LaserScan containing the new lidar data.

		Returns:
				A Twist message, containing the commanded robot velocities.
		"""
		angle = atan2(target[1], target[0])
		distance = sqrt(target[0] ** 2 + target[1] ** 2)
		rospy.loginfo(f"Distance: {distance:.2f}, angle: {angle:.2f}")

		# This builds a Twist message with all elements set to zero.
		command = Driver.zero_twist()

		# Define a distance we consider "close"
		close_distance = 1

		# If we're approaching a goal, limit our speed
		# Otherwise, we approach quickly
		if distance < close_distance:
			command.linear.x = 0.2
		else:
			command.linear.x = min(0.35, 0.25 * distance)

		# How close should we start considering obstacles?
		# I determined this to be 3
		scan_distance = 3
		# Calculate the influence of the obstances in the robot's view
		obstacle_influence = self.calc_turn_influence(lidar, scan_distance, 5)

		# If any lidar scan comes in close, apply bias
		# from our surrouding obstances to navigate
		if distance < close_distance:
			command.angular.z = 6 * (angle / pi)
		else:
			command.angular.z = 4 * (angle / pi) + (obstacle_influence / 4)

		return command


if __name__ == "__main__":
    rospy.init_node("student_driver", argv=sys.argv)
    driver = StudentDriver()
    rospy.spin()