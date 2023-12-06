#!/usr/bin/env python3

import rospy
import actionlib
import numpy as np
from math import trunc
from walle.msg import SLAMAction, SLAMResult, ProcessedMap
from scipy import ndimage

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

	def execute(self, message):
		rospy.loginfo(f'Received SLAM map to process')
		map = message.map

		map_thresh = self.process_map(map)
		map_unseen = self.find_unseen(map_thresh)

		# exploring.plot_with_explore_points(map_thresh, 0.1, explore_points=map_unseen)

		pmap = ProcessedMap()
		pmap.thresh.data = tuple(map_thresh.flatten().astype(int))
		pmap.unseen.data = tuple(map_unseen.flatten().astype(int))
		pmap.width.data = map.info.width

		response = SLAMResult()
		response.pmap = pmap

		self.server.set_succeeded(response)
		rospy.loginfo(f'SLAM map processing complete')

	def process_map(self, map):
		map_2d = np.fromiter(map.data, int).reshape(-1, map.info.width)

		map_ret = np.zeros((map.info.height, map.info.width), dtype=int) + 128
		map_ret[map_2d == 100] = 0
		map_ret[map_2d == 0] = 255

		return map_ret

	def find_unseen(self, map):
		# Binary dilation, quite handy
		seen_mask = (map == 255)
		eroded_mask = ndimage.binary_dilation(seen_mask, np.ones((3, 3), dtype=bool))
		unseen = np.flip(np.argwhere((eroded_mask ^ seen_mask) & (map == 128)))
		return unseen

if __name__ == '__main__':
	rospy.init_node('slam_processor')
	server = SLAMServer()
	rospy.spin()