#!/usr/bin/env python3

# This assignment lets you both define a strategy for picking the next point to explore and determine how you
#  want to chop up a full path into way points. You'll need path_planning.py as well (for calculating the paths)
#
# Note that there isn't a "right" answer for either of these. This is (mostly) a light-weight way to check
#  your code for obvious problems before trying it in ROS. It's set up to make it easy to download a map and
#  try some robot starting/ending points
#
# Given to you:
#   Image handling
#   plotting
#   Some structure for keeping/changing waypoints and converting to/from the map to the robot's coordinate space
#
# Slides

# The ever-present numpy
import numpy as np

# Your path planning code
import path_planning as path_planning
# Our priority queue
import heapq
from math import pi
from skimage.draw import line as skimage_line

# -------------- Showing start and end and path ---------------
def plot_with_explore_points(im_threshhold, zoom=1.0, robot_loc=None, explore_points=None, best_pt=None):
    """Show the map plus, optionally, the robot location and points marked as ones to explore/use as end-points
    @param im - the image of the SLAM map
    @param im_threshhold - the image of the SLAM map
    @param robot_loc - the location of the robot in pixel coordinates
    @param best_pt - The best explore point (tuple, i,j)
    @param explore_points - the proposed places to explore, as a list"""

    # Putting this in here to avoid messing up ROS
    import matplotlib.pyplot as plt

    fig, axs = plt.subplots(1, 2)
    axs[0].imshow(im_threshhold, origin='lower', cmap="gist_gray")
    axs[0].set_title("original image")
    axs[1].imshow(im_threshhold, origin='lower', cmap="gist_gray")
    axs[1].set_title("threshold image")
    """
    # Used to double check that the is_xxx routines work correctly
    for i in range(0, im_threshhold.shape[1]-1, 10):
        for j in range(0, im_threshhold.shape[0]-1, 2):
            if is_reachable(im_thresh, (i, j)):
                axs[1].plot(i, j, '.b')
    """

    # Show original and thresholded image
    if explore_points is not None:
        for p in explore_points:
            axs[1].plot(p[0], p[1], '.b', markersize=2)

    for i in range(0, 2):
        if robot_loc is not None:
            axs[i].plot(robot_loc[0], robot_loc[1], '+r', markersize=10)
        if best_pt is not None:
            axs[i].plot(best_pt[0], best_pt[1], '*y', markersize=10)
        axs[i].axis('equal')

    for i in range(0, 2):
        # Implements a zoom - set zoom to 1.0 if no zoom
        width = im_threshhold.shape[1]
        height = im_threshhold.shape[0]

        axs[i].set_xlim(width / 2 - zoom * width / 2, width / 2 + zoom * width / 2)
        axs[i].set_ylim(height / 2 - zoom * height / 2, height / 2 + zoom * height / 2)


# -------------- For converting to the map and back ---------------
def convert_pix_to_x_y(im_size, pix, size_pix):
    """Convert a pixel location [0..W-1, 0..H-1] to a map location (see slides)
    Note: Checks if pix is valid (in map)
    @param im_size - width, height of image
    @param pix - tuple with i, j in [0..W-1, 0..H-1]
    @param size_pix - size of pixel in meters
    @return x,y """
    if not (0 <= pix[0] <= im_size[1]) or not (0 <= pix[1] <= im_size[0]):
        raise ValueError(f"Pixel {pix} not in image, image size {im_size}")

    return [size_pix * pix[i] / im_size[1-i] for i in range(0, 2)]


def convert_x_y_to_pix(im_size, x_y, size_pix):
    """Convert a map location to a pixel location [0..W-1, 0..H-1] in the image/map
    Note: Checks if x_y is valid (in map)
    @param im_size - width, height of image
    @param x_y - tuple with x,y in meters
    @param size_pix - size of pixel in meters
    @return i, j (integers) """
    pix = [int(x_y[i] * im_size[1-i] / size_pix) for i in range(0, 2)]

    if not (0 <= pix[0] <= im_size[1]) or not (0 <= pix[1] <= im_size[0]):
        raise ValueError(f"Loc {x_y} not in image, image size {im_size}")
    return pix


def is_reachable(im, pix):
    """ Is the pixel reachable, i.e., has a neighbor that is free?
    Used for
    @param im - the image
    @param pix - the pixel i,j"""

    # Returns True (the pixel is adjacent to a pixel that is free)
    #  False otherwise
    # You can use four or eight connected - eight will return more points
    adj_nodes = path_planning.four_connected(pix)

    # Check if any neighbor is free
    for adj in adj_nodes:
        if path_planning.is_free(im, adj):
            return True

    return False


def find_all_possible_goals(im):
    """ Find all of the places where you have a pixel that is unseen next to a pixel that is free
    It is probably easier to do this, THEN cull it down to some reasonable places to try
    This is because of noise in the map - there may be some isolated pixels
    @param im - thresholded image
    @return dictionary or list or binary image of possible pixels"""
    possible_goals = []

    for i in range(0, im.shape[0]-1):
            for j in range(0, im.shape[1]-1):
                # Check if the current pixel is unseen
                if path_planning.is_unseen(im, (i, j)) and is_reachable(im, (i,j)):
                    possible_goals.append((i, j))

    return possible_goals

def dist(node1, node2):
            """Calculate Euclidean distance between two points"""
            return np.sqrt((node1[0] - node2[0])**2 + (node1[1] - node2[1])**2)

def find_best_point(im, possible_points, robot_loc):
    """ Pick one of the unseen points to go to
    @param im - thresholded image
    @param possible_points - possible points to chose from
    @param robot_loc - location of the robot (in case you want to factor that in)
    """

    best_point = None

    # Sort the list based on priorities
    # Sort based on distance first, then on the number of unseen neighbors
    priorities = sorted(possible_points, key=lambda point: (-dist(point, robot_loc), -sum(1 for adj in path_planning.eight_connected(point) if path_planning.is_unseen(im, adj))))

    # Select the point with the highest priority
    best_point = priorities[0]

    # Check if the best point is free; if not, find a free neighbor
    if not path_planning.is_free(im, best_point):
        adj_nodes = path_planning.four_connected(best_point)
        for adj in adj_nodes:
            if path_planning.is_free(im, adj):
                return adj

    return (best_point[0], best_point[1])


def find_waypoints(im, path, robot_ij):
    """ Place waypoints along the path
    @param im - the thresholded image
    @param path - the initial path
    @ return - a new path"""

    waypoints = []
    if len(path) <= 2:
        return path
    # waypoints.append(path[0]) 
    wall_side = list()
    wall_side.append('right')
    
    last_waypoint = path[0]

    for i in range(1, len(path)-1):
        adj_nodes = path_planning.four_connected(path[i])

        wall_detected = False
        for adj in adj_nodes:
            if path_planning.is_wall(im, adj):
                wall_detected = True
                if path[i][0] > adj[0]:
                    wall_side.append('left')
                elif path[i][0] < adj[0]:
                    wall_side.append('right')
                elif path[i][1] > adj[1]:
                    wall_side.append('below')
                elif path[i][1] < adj[1]:
                    wall_side.append('above')

        if not wall_detected:
            wall_side.append(wall_side[-1])
        
            
    for i in range(1, len(path)-1): # iterate through the path
        if dist(robot_ij, path[i]) < 25:
            continue
        
        inbetween = bresenham_line(last_waypoint, path[i])
        obstacles = im[inbetween[:, 0], inbetween[:, 1]] == 1
        
        x1, y1 = path[i-1]
        x2, y2 = path[i]
        x3, y3 = path[i+1]
        	
        if (x2 - x1) != 0:
            slope1 = (y2-y1)/(x2-x1) # previous slope
        else: # can't /0
            slope1 = float('inf') if (y2 - y1) > 0 else float('-inf')

        if (x3 - x2) != 0:
            slope2 = (y3-y2)/(x3-x2) # next slope
        else: # can't /0
            slope2 = float('inf') if (y3 - y2) > 0 else float('-inf')
            
        if slope1 is np.inf or slope1 is -np.inf or slope2 is np.inf or slope2 is -np.inf:
            continue

        if (not np.isclose(slope1, slope2, atol=pi/2)) or (True in obstacles) or (dist(last_waypoint, path[i]) > 25): 
            waypoints.append(adjust(path[i], wall_side[i])) # waypoint at slope change
            last_waypoint = waypoints[-1]
    waypoints.append(path[-1]) # final destination
    return waypoints

# Bresenham's line algorithm
def bresenham_line(start, end):
    rr, cc = skimage_line(start[0], start[1], end[0], end[1])
    return np.column_stack((rr, cc))

def adjust(point, wall_side):
    x, y = point
    adjusted_point = None
    
    if wall_side == 'left':
        adjusted_point = (x + 5, y)
    elif wall_side == 'right':
        adjusted_point = (x - 5, y)
    elif wall_side == 'below':
        adjusted_point = (x, y + 5)
    elif wall_side == 'above':
        adjusted_point = (x, y - 5)      

    return adjusted_point


if __name__ == '__main__':
    # Doing this here because it is a different yaml than JN
    import yaml_1 as yaml

    im, im_thresh = path_planning.open_image("map.pgm")

    robot_start_loc = (1940, 1953)

    all_unseen = find_all_possible_goals(im_thresh)
    best_unseen = find_best_point(im_thresh, all_unseen, robot_loc=robot_start_loc)

    plot_with_explore_points(im_thresh, zoom=0.1, robot_loc=robot_start_loc, explore_points=all_unseen, best_pt=best_unseen)

    path = path_planning.dijkstra(im_thresh, robot_start_loc, best_unseen)
    waypoints = find_waypoints(im_thresh, path)
    path_planning.plot_with_path(im, im_thresh, zoom=0.1, robot_loc=robot_start_loc, goal_loc=best_unseen, path=waypoints)

    # Depending on if your mac, windows, linux, and if interactive is true, you may need to call this to get the plt
    # windows to show
    # plt.show()

    print("Done")
