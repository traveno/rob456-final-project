#!/usr/bin/env python3

import rospy
import numpy as np
from math import trunc
from walle.msg import ProcessedMap
from scipy import ndimage
from sensor_msgs.msg import Image
import time
from nav_msgs.msg import OccupancyGrid

def map_to_pixel(map_info, location):
    x = trunc((location[0] - map_info.origin.position.x) / map_info.resolution) + 5
    y = trunc((location[1] - map_info.origin.position.y) / map_info.resolution) + 5
    return (x, y)


def pixel_to_map(map_info, location):
    x = location[0] * map_info.resolution + map_info.origin.position.x
    y = location[1] * map_info.resolution + map_info.origin.position.y
    return [x, y]


class SLAMNode:
    def __init__(self):
        # Image server
        self.image_pub = rospy.Publisher('supervision', Image, queue_size=1)

        # Subscribe to the map and the map metadata.
        self.map_sub = rospy.Subscriber('map', OccupancyGrid, self.execute, queue_size=10)

        # Processed map server
        self.pmap_pub = rospy.Publisher('map_processed', ProcessedMap, queue_size=1)

    def execute(self, map):
        rospy.loginfo(f"Received SLAM map to process")
        start = time.time()

        # Process raw monochrome data
        map_thresh, map_width, map_height, map_ij = self.process_map(map)

        # Find good places to see
        self.find_unseen(map_thresh)

        end = time.time()
        rospy.loginfo(f"SLAM processing took {round(end - start, 3)}s")

        # Send a colored image to our viewer
        self.annotate_image(map_thresh, map_width, map_height)

        start = time.time()
        pmap = ProcessedMap()
        pmap.map = tuple(map_thresh.flatten().astype(int))
        pmap.offset_i = map_ij[0]
        pmap.offset_j = map_ij[1]

        self.pmap_pub.publish(pmap)

        end = time.time()
        rospy.loginfo(f"Map msg processing took {round(end - start, 3)}s")

    def annotate_image(self, map, width, height):
        # Publish a new image to rqt_image_view
        start = time.time()

        # Turn into RGB spec
        map_rgb = np.repeat(map[:, :, np.newaxis], 3, axis=2)

        # 254 - areas around unseen that we can reach
        map_rgb = np.where(map_rgb == [254, 254, 254], [0, 0, 255], map_rgb)

        image = Image()
        image.data = tuple(np.flipud(map_rgb).flatten().astype(int))
        image.step = width * 3
        image.width = width
        image.height = height
        image.encoding = "rgb8"
        image.header.stamp = rospy.Time.now()

        # Send it
        self.image_pub.publish(image)
        end = time.time()
        rospy.loginfo(f"Image view msg processing took {round(end - start, 3)}s") 

    def process_map(self, map):
        map_2d = np.fromiter(map.data, int).reshape(-1, map.info.width)

        non_negative_indices = np.where(map_2d != -1)
        min_row, max_row = np.min(non_negative_indices[0]), np.max(non_negative_indices[0])
        min_col, max_col = np.min(non_negative_indices[1]), np.max(non_negative_indices[1])
        map_width, map_height = max_col - min_col + 1, max_row - min_row + 1
        map_trimmed = map_2d[min_row:max_row + 1, min_col:max_col + 1]

        map_ret = np.zeros((map_height, map_width), dtype=int) + 128
        map_ret[map_trimmed == 100] = 0
        map_ret[map_trimmed == 0] = 255

        # Thicken the wall areas using binary dilation
        wall_mask = map_ret == 0
        dilation_mask = ndimage.binary_dilation(wall_mask, np.ones((5, 5), dtype=bool))
        wide_walls = dilation_mask ^ wall_mask
        map_ret[wide_walls == 1] = 0

        # map_ret = np.repeat(map_ret, 3, axis=1)
        return map_ret, map_width, map_height, (min_row, min_col)

    def find_unseen(self, map):
        # Binary dilation, quite handy
        unseen_mask = (map == 128)
        dilation_mask = ndimage.binary_dilation(unseen_mask, np.ones((2, 2), dtype=bool))
        seen_border = (dilation_mask ^ unseen_mask) & (map == 255)

        # Set areas we can reach but haven't seen to 254
        map[seen_border == 1] = 254

        return seen_border


if __name__ == "__main__":
    rospy.init_node("slam_processor")
    server = SLAMNode()
    rospy.spin()
