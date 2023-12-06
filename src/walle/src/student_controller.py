#!/usr/bin/env python3


import sys
import rospy
import signal
import numpy as np
from math import floor

from controller import RobotController
import path_planning
import exploring
import actionlib
import math

from walle.msg import SLAMAction, SLAMGoal, SLAM

from tf_repeated_data_suppress import suppress_TF_REPEATED_DATA

class StudentController(RobotController):
	'''
	This class allows you to set waypoints that the robot will follow.  These robots should be in the map
	coordinate frame, and will be automatially sent to the code that actually moves the robot, contained in
	StudentDriver.
	'''
	def __init__(self):
		super().__init__()
		self.slam_client = actionlib.SimpleActionClient('slam_processor', SLAMAction)
		self.slam_client.wait_for_server()
		self.slam_processing = False
		self.robot_position = None
		

	def distance_update(self, distance):
		'''
		This function is called every time the robot moves towards a goal.  If you want to make sure that
		the robot is making progress towards it's goal, then you might want to check that the distance to
		the goal is generally going down.  If you want to change where the robot is heading to, you can
		make a call to set_waypoints here.  This call will override the current set of waypoints, and the
		robot will start to drive towards the first waypoint in the new list.

		Parameters:
			distance:	The distance to the current goal.
		'''
		# rospy.loginfo(f'Distance: {distance}')

	def slam_update(self, state, result):
		points = result.best
		waypoints = np.array(points, dtype=tuple).reshape(-1, 2)
		
		if len(waypoints) > 0 and self.robot_position is not None:
			best = 1e30
			best_index = 0
			for i, point in enumerate(waypoints):
				distance = math.sqrt((self.robot_position[0] - point[0])**2 + (self.robot_position[1] - point[1])**2)
				if distance < best:
					best = distance
					best_index = i

			self.set_waypoints(waypoints[best_index:])
			
		self.slam_processing = False
		rospy.loginfo(f'Got SLAM update! {waypoints}')


	def map_update(self, point, map, map_data):
		'''
		This function is called every time a new map update is available from the SLAM system.  If you want
		to change where the robot is driving, you can do it in this function.  If you generate a path for
		the robot to follow, you can pass it to the driver code using set_waypoints().  Again, this will
		override any current set of waypoints that you might have previously sent.

		Parameters:
			point:		A PointStamped containing the position of the robot, in the map coordinate frame.
			map:		An OccupancyGrid containing the current version of the map.
			map_data:	A MapMetaData containing the current map meta data.
		'''
		rospy.loginfo('Got a map update.')

		if self.slam_processing:
			rospy.loginfo(f'SLAM is still being processed')
			return

		# It's possible that the position passed to this function is None.  This try-except block will deal
		# with that.  Trying to unpack the position will fail if it's None, and this will raise an exception.
		# We could also explicitly check to see if the point is None.
		try:
			# The (x, y) position of the robot can be retrieved like this.
			robot_position = (point.point.x, point.point.y)
			self.robot_position = robot_position

			slam_obj = SLAM()
			slam_obj.map = map
			slam_obj.pos = point

			slam_goal = SLAMGoal()
			slam_goal.update = slam_obj
			self.slam_client.send_goal(slam_goal, self.slam_update)
		except:
			rospy.loginfo('No odometry information')
			

if __name__ == '__main__':
	# Initialize the node.
	rospy.init_node('student_controller', argv=sys.argv)

	# Start the controller.
	controller = StudentController()

	# This will move the robot to a set of fixed waypoints.  You should not do this, since you don't know
	# if you can get to all of these points without building a map first.  This is just to demonstrate how
	# to call the function, and make the robot move as an example.
	# controller.set_waypoints(((-4, -3), (-4, -5)))

	# Once you call this function, control is given over to the controller, and the robot will start to
	# move.  This function will never return, so any code below it in the file will not be executed.
	controller.send_points()
