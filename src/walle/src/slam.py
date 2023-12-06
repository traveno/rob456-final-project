#!/usr/bin/env python3

import rospy

from walle.srv import ProcessSLAM, ProcessSLAMResponse
import actionlib
import numpy as np
import path_planning, exploring
from math import floor, trunc
import tf
from walle.msg import SLAMAction, SLAMResult

def map_to_pixel(map_info, location):
	x = trunc((location[0] - map_info.origin.position.x) / map_info.resolution)
	y = trunc((location[1] - map_info.origin.position.y) / map_info.resolution)
	return (x, y)

def pixel_to_map(map_info, location):
	x = location[0] * map_info.resolution + map_info.origin.position.x
	y = location[1] * map_info.resolution + map_info.origin.position.y
	return [x, y]

class SLAMServer:
	def __init__(self):
		self.server = actionlib.SimpleActionServer('slam_processor', SLAMAction, self.execute, False)
		self.server.start()
		self.transform_listener = tf.TransformListener()

	def execute(self, message):
		map = message.update.map
		pos = message.update.pos

		robot_position = map_to_pixel(map.info, (pos.point.x, pos.point.y))
		map_thresh = self.process_map(map)

		# rospy.loginfo(f'Unique values in map_thresh: {np.unique(map_thresh)}')
		# rospy.loginfo(f'Robot position {robot_position}')
		# rospy.loginfo(f'Walls: {len(np.where(map_thresh == 0))} Free: {len(np.where(map_thresh == 255))} Unseen: {len(np.where(map_thresh == 128))}')

		map_unseen = exploring.find_all_possible_goals(map_thresh)
		map_waypoints, goal_point = self.make_waypoints(map_thresh, map_unseen, robot_position)

		if map_waypoints is None:
			rospy.loginfo('No waypoints created')
			response = SLAMResult()
			response.best = []
			self.server.set_aborted(response)
			return

		waypoints = []
		for mw in map_waypoints:
			waypoints.extend(pixel_to_map(map.info, (float(mw[0]), float(mw[1]))))

		# rospy.loginfo(f'New waypoints: {waypoints}')
		exploring.plot_with_explore_points(map_thresh, 0.1, robot_position, map_unseen, goal_point)

		response = SLAMResult()
		response.best = waypoints

		self.server.set_succeeded(response)

	def process_map(self, map):
		map_2d = np.zeros((map.info.height, map.info.width))

		for i in range(len(map.data)):
			map_2d[floor(i / map.info.width)][i % map.info.width] = map.data[i]

		map_ret = np.zeros((map.info.height, map.info.width)) + 128
		map_ret[map_2d == 100] = 0
		map_ret[map_2d == 0] = 255

		return map_ret

	def make_waypoints(self, map_thresh, map_unseen, robot_position):
		best_point = exploring.find_best_point(map_thresh, map_unseen, robot_position)
		if best_point is None: return None
		path = path_planning.dijkstra(map_thresh, robot_position, best_point)
		print(f'Path {path}')
		waypoints = exploring.find_waypoints(map_thresh, path)
		print(f'Waypoints {waypoints}')
		return waypoints, best_point

if __name__ == '__main__':
	rospy.init_node('slam_processor')
	server = SLAMServer()
	rospy.spin()