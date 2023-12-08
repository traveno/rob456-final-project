import numpy as np
import path_planning as path_planning
import heapq
import imageio
from math import atan2, sqrt, pi


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
# YOUR CODE HERE
    return False


def find_all_possible_goals(im):
    """ Find all of the places where you have a pixel that is unseen next to a pixel that is free
    It is probably easier to do this, THEN cull it down to some reasonable places to try
    This is because of noise in the map - there may be some isolated pixels
    @param im - thresholded image
    @return dictionary or list or binary image of possible pixels"""

    goals = set()
    
    def process_point(i, j):
      related = path_planning.eight_connected((i, j))
      has_unseen_neighbor = False
      seen_neighbors = []
      
      for related_i, related_j in related:
        if 0 <= related_i < im.shape[0] and 0 <= related_j < im.shape[1]:
          if im[related_i, related_j] == 128:
            has_unseen_neighbor = True
          if im[related_i, related_j] == 255:
            seen_neighbors.append((related_j, related_i))
          if im[related_i, related_j] == 0:
            return

      if has_unseen_neighbor and len(seen_neighbors) >= 2:
        for point in seen_neighbors: goals.add((point[0], point[1], 0))
    
    free = np.where(im == 255)
    for i, j in zip(free[0], free[1]): process_point(i, j)
    return np.fromiter(list(goals), dtype='float64')


def find_best_point(im, possible_points, robot_loc):
    """ Pick one of the unseen points to go to
    @param im - thresholded image
    @param possible_points - possible points to chose from
    @param robot_loc - location of the robot (in case you want to factor that in)
    """

    if len(possible_points) == 0: return None
    return max(possible_points, key=lambda p: sqrt((p[0] - robot_loc[0])**2 + (p[1] - robot_loc[1])**2))

def find_waypoints(im, path):
    """ Place waypoints along the path
    @param im - the thresholded image
    @param path - the initial path
    @ return - a new path"""

    waypoints = [] 
    
    prev_theta = None
    prev_point = None
    
    for point in reversed(path):
      if prev_point is None:
        prev_point = point
        continue
    
      theta = atan2(point[0] - prev_point[0], point[1] - prev_point[1])
    
      if prev_theta is None:
        prev_theta = theta
        continue
      
      if not np.isclose(theta, prev_theta, atol=0.05):
        waypoints.append(prev_point)
        
      prev_point = point
      prev_theta = theta

    if len(waypoints) == 0:
       waypoints.append(path[-1])
        
    return waypoints