#!/usr/bin/env python3


import sys
import rospy
import signal

from controller import RobotController
import path_planning as path_planning
import exploring as exploring
import numpy as np

from tf_repeated_data_suppress import suppress_TF_REPEATED_DATA

class StudentController(RobotController):
	'''
	This class allows you to set waypoints that the robot will follow.  These robots should be in the map
	coordinate frame, and will be automatially sent to the code that actually moves the robot, contained in
	StudentDriver.
	'''
	def __init__(self):
		super().__init__()

		self.goal = None #store goal as member of class


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
		rospy.loginfo(f'Distance: {distance}')

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

			rospy.loginfo(f'Robot is at {robot_position} {point.header.frame_id}')
		except:
			rospy.loginfo('No odometry information')

	"""
	Little two-liner function that takes in the map data and spits out an image and image threshold
	
	Parameters:
			map:		An OccupancyGrid containing the current version of the map.
	"""
	def get_image_from_map(self, map, map_data):
		im = np.array(map.data).reshape(map.info.height, map.info.width)
		im_thresh = path_planning.convert_image(im, 0.7, 0.9) #borrowed from path_planning.open_image()

		return im, im_thresh
	
	def update_path(self, map, map_data, point): #actually call dijkstra's/explore here
		im, thresh = self.get_image_from_map(map) #get image and threshold from current map

		robot_start = (point.point.x, point.point.y) #i think this is in map coordinates, if not we need to change it

		if self.goal == None: #create a new path if we don't already have one
			#get all unseen points first
			unseen = exploring.find_all_possible_goals(thresh)
			#check if unseen is empty - if it is we're probably done exploring!
			if unseen is None:
				rospy.loginfo('Either all points are found, or weird error occured')
				return
			else:
				self.goal = exploring.find_best_point(thresh, unseen, robot_start)
		
		rospy.loginfo(f'Got a new goal: current path is from {robot_start} to {self.goal}')

		path = path_planning.dijkstra(thresh, robot_start, self.goal)
		map_waypoints = exploring.find_waypoints(thresh, path)

		for i in map_waypoints:
			#convert to x,y coordinates



if __name__ == '__main__':
	# Initialize the node.
	rospy.init_node('student_controller', argv=sys.argv)
	suppress_TF_REPEATED_DATA()

	# Start the controller.
	controller = StudentController()

	# This will move the robot to a set of fixed waypoints.  You should not do this, since you don't know
	# if you can get to all of these points without building a map first.  This is just to demonstrate how
	# to call the function, and make the robot move as an example.
	#controller.set_waypoints(((-4, -3), (-4, 0), (5, 0)))

	# Once you call this function, control is given over to the controller, and the robot will start to
	# move.  This function will never return, so any code below it in the file will not be executed.
	controller.send_points()
