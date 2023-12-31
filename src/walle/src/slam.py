#!/usr/bin/env python3

# https://github.com/traveno/rob456-final-project
# Running this project will need some additional files!
# This includes ProcessedMsg type for the SLAM node.
# The SLAM processing code is in slam.py. It receives
# the SLAM updates and sends the processed version
# to this controller.

import rospy
import numpy as np
from math import trunc
from walle.msg import ProcessedMap
from scipy import ndimage
from sensor_msgs.msg import Image
import time
from nav_msgs.msg import OccupancyGrid
from walle.msg import Path

def map_to_pixel(map_info, offset_ij, location_ij):
    j = trunc((location_ij[1] - map_info.origin.position.x) / map_info.resolution) - offset_ij[1]
    i = trunc((location_ij[0] - map_info.origin.position.y) / map_info.resolution) - offset_ij[0]
    return (i, j)

def pixel_to_map(map_info, offset_ij, location_ij):
    j = (location_ij[1] + offset_ij[1]) * map_info.resolution + map_info.origin.position.x
    i = (location_ij[0] + offset_ij[0]) * map_info.resolution + map_info.origin.position.y
    return (i, j)

class SLAMNode:
    def __init__(self):
        # Image server
        self.image_pub = rospy.Publisher('supervision', Image, queue_size=1)

        # Subscribe to the map and the map metadata.
        self.map_sub = rospy.Subscriber('map', OccupancyGrid, self.execute, queue_size=10)

        # Processed map server
        self.pmap_pub = rospy.Publisher('map_processed', ProcessedMap, queue_size=1, latch=True)

        # The robot will send messages that describe it's path, before its been turned
        # into waypoints. This allows us to compare the raw path vs our optimized waypoints.
        #   NOTE: Current path is a darker green than previous paths.
        self.path_sub = rospy.Subscriber('path', Path, self.path_update, queue_size=10)
        self.path_points = []
        self.old_path_points = []

        # Cache map data so if a new path comes in, we can render an image immediately
        self.viewer_cache = None

    def execute(self, map):
        rospy.loginfo(f'----BEGIN PROCESSING----')
        rospy.loginfo(f"Received SLAM map to process")
        start = time.time()

        # Process raw monochrome data
        map_thresh, map_width, map_height, offset_ij = self.process_map(map)

        # Find good places to check out ya know, explore the world
        # This function mutates the map, you have been warned!
        self.find_unseen(map_thresh)

        rospy.loginfo(f'Trimmed size: {map_width}px width, {map_height}px height')
        rospy.loginfo(f'Offset: {offset_ij[0]} y-axis, {offset_ij[1]} x-axis')

        end = time.time()
        rospy.loginfo(f"SLAM processing took {round(end - start, 3)}s")
        
        # Send a colored image to our viewer
        self.annotate_image(map_thresh, map_width, map_height, map.info, offset_ij)
        
        # Cache map data so we can render new paths without needing to wait for SLAM
        self.viewer_cache = (map_thresh, map_width, map_height, map.info, offset_ij)

        start = time.time()
        pmap = ProcessedMap()
        pmap.map = tuple(map_thresh.flatten().astype(int))
        pmap.width = map_width
        pmap.offset_i = offset_ij[0]
        pmap.offset_j = offset_ij[1]
        pmap.meta = map.info

        # We're done! Send the message
        self.pmap_pub.publish(pmap)

        end = time.time()
        rospy.loginfo(f"Map msg processing took {round(end - start, 3)}s")
        rospy.loginfo(f'-----END PROCESSING-----')

    def path_update(self, path_msg: Path):
        new_path_points = np.fromiter(path_msg.path, int).reshape(-1, 2)
        self.old_path_points.extend(self.path_points)
        self.path_points = new_path_points
        
        # If the viewer cache is defined, immediately render a new image
        if self.viewer_cache is not None:
            self.annotate_image(*self.viewer_cache)

    # This function colors and draws current path and previous path(s).
    # Result is submitted it to the image viewer for our viewing pleasure.
    # I think it will also help the team with debugging, and see exactly
    # what is happening with my SLAM code in real time. Otherwise,
    # this function has no effect on the robot's behavior.
    def annotate_image(self, map, width, height, meta, offset_ij):
        # Publish a new image to rqt_image_view
        start = time.time()

        # Turn into RGB spec
        map_rgb = np.repeat(map[:, :, np.newaxis], 3, axis=2)

        # 254 - areas around unseen that we can reach
        map_rgb = np.where(map_rgb == [254, 254, 254], [0, 0, 255], map_rgb)

        # 1 - areas where we don't allow movement (essentially a wall, but not actually)
        map_rgb = np.where(map_rgb == [1, 1, 1], [255, 230, 30], map_rgb)

        # Draw current path
        if len(self.path_points) > 0:
            for p in self.path_points:
                map_rgb[p[0], p[1]] = [64, 255, 64]

        # Make lines a bit thicker since 1px wide paths are hard to see sometimes.
        # This is done via binary dilation and is a recurring theme in my processing.
        line_mask = np.all(map_rgb == [64, 255, 64], axis=-1)
        line_dilation = ndimage.binary_dilation(line_mask, np.ones((2, 2), dtype=bool))
        line_mask = (line_mask ^ line_dilation)
        map_rgb[line_mask] = [128, 255, 128]

        # Draw old paths
        if len(self.old_path_points) > 0:
            for p in self.old_path_points:
                map_rgb[p[0], p[1]] = [225, 225, 225]

        # Make lines a bit thicker since 1px wide paths are hard to see sometimes
        line_mask = np.all(map_rgb == [225, 225, 225], axis=-1)
        line_dilation = ndimage.binary_dilation(line_mask, np.ones((2, 2), dtype=bool))
        line_mask = (line_mask ^ line_dilation)
        map_rgb[line_mask] = [225, 255, 225]
              
        # Package up the image for our image viewer
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
        # Reshape the 1D occupancy grid for processing
        map_2d = np.fromiter(map.data, int).reshape(-1, map.info.width)

        # Trim out the fat
        non_negative_indices = np.where(map_2d != -1)

        # Find the minimum dimensions while retaining all data
        min_row, max_row = np.min(non_negative_indices[0]), np.max(non_negative_indices[0])
        min_col, max_col = np.min(non_negative_indices[1]), np.max(non_negative_indices[1])

        # Calculate map width and height
        map_width, map_height = max_col - min_col + 1, max_row - min_row + 1

        # Create the trimmed map
        map_trimmed = map_2d[min_row:max_row + 1, min_col:max_col + 1]

        # Now we convert map_2d's range of [-1,100] to values we're
        # familiar with, which are [0,255]
        map_ret = np.zeros((map_height, map_width), dtype=int) + 128
        map_ret[map_trimmed == 100] = 0
        map_ret[map_trimmed == 0] = 255

        # Add some padding in case our pathfinding alg explores slightly
        # outside of map boundaries
        map_ret = np.pad(map_ret, 5, mode='constant', constant_values=0)

        # Thicken the wall areas using binary dilation
        # This appears as yellow on the map, think of it as a
        # danger zone (or no-go zone) for pathfinding.
        #   NOTE: This suffers from potentially cutting off narrow passages,
        #   I think it would be fun to explore further processing that checks
        #   for islands and draws thin bridges to ensure all areas of
        #   the map are still accessible.
        wall_mask = map_ret == 0
        dilation_mask = ndimage.binary_dilation(wall_mask, np.ones((15, 15), dtype=bool))
        wide_walls = (dilation_mask ^ wall_mask) & ((map_ret == 255) | (map_ret == 128))
        map_ret[wide_walls == 1] = 1

        # map_ret = np.repeat(map_ret, 3, axis=1)
        return map_ret, map_width + 10, map_height + 10, (min_row - 5, min_col - 5)

    def find_unseen(self, map):
        # Binary dilation, quite handy
        # We dilate unseen areas, XOR that with non-dilated unseen areas,
        # then AND it with areas that are accessible. This creates the blue
        # zones in the map.
        unseen_mask = (map == 128)
        dilation_mask = ndimage.binary_dilation(unseen_mask, np.ones((3, 3), dtype=bool))
        seen_border = (dilation_mask ^ unseen_mask) & (map == 255)

        # Set areas we can reach but haven't seen to 254
        # The value is arbitrary, it just needs to be unique
        map[seen_border == 1] = 254

        return seen_border

if __name__ == "__main__":
    rospy.init_node("slam_processor")
    server = SLAMNode()
    rospy.spin()

