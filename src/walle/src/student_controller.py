#!/usr/bin/env python3
import sys
import rospy
import numpy as np

from controller import RobotController
import actionlib

from walle.msg import SLAMAction, SLAMGoal


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
		raw_width = result.pmap.width.data
		raw_thresh = result.pmap.thresh.data
		raw_unseen = result.pmap.unseen.data
		
		map_thresh = np.fromiter(raw_thresh, int).reshape(-1, raw_width)
		map_unseen = [(x, y) for x, y in np.fromiter(raw_unseen, tuple).reshape(-1, 2)]

		rospy.loginfo(f'Got SLAM update from callback')


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

		# It's possible that the position passed to this function is None.  This try-except block will deal
		# with that.  Trying to unpack the position will fail if it's None, and this will raise an exception.
		# We could also explicitly check to see if the point is None.
		try:
			# The (x, y) position of the robot can be retrieved like this.
			robot_position = (point.point.x, point.point.y)
		except:
			rospy.loginfo('No odometry information')

		# Send the SLAM map out for processing
		slam_goal = SLAMGoal()
		slam_goal.map = map
		self.slam_client.send_goal(slam_goal, self.slam_update)
			

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
