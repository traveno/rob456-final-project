import numpy as np
import path_planning as path_planning
from scipy.signal import correlate2d
from math import atan2, sqrt, pi
from skimage.draw import line as skimage_line

def find_best_point(pmap, possible_points, robot_ij):
    """Pick one of the unseen points to go to
    @param im - thresholded image
    @param possible_points - possible points to chose from
    @param robot_loc - location of the robot (in case you want to factor that in)
    """

    if len(possible_points) == 0:
        return None

    filtered_map = (pmap == 254)
    filtered_map = np.where(filtered_map == 1, 254, 0)

    mask = np.full((25, 25), 254)

    # Find best cross correlation
    correlation = correlate2d(filtered_map, mask, mode='valid')

    # Find the indices of the maximum correlation
    max_corr_index = np.unravel_index(np.argmax(correlation), correlation.shape)
    
    furthest = min(
        possible_points,
        # Desire points closest to the max correlated point, also desire points that are further away from the robot
        key=lambda p: sqrt((p[0] - max_corr_index[0]) ** 2 + (p[1] - max_corr_index[1]) ** 2) +
                      np.log(1 + sqrt((p[0] - robot_ij[0]) ** 2 + (p[1] - robot_ij[1]) ** 2)) / 4,
    )

    return (furthest[0], furthest[1])


def find_waypoints(pmap, path, robot_ij):
    """Place waypoints along the path
    @param im - the thresholded image
    @param path - the initial path
    @ return - a new path"""

    waypoints = []

    prev_point = robot_ij

    for i in range(0, len(path)):
        inbetween = bresenham_line(prev_point, path[i])
        obstacles = pmap[inbetween[:, 0], inbetween[:, 1]] == 1

        if True in obstacles or dist(path[i], prev_point) > 25:
            waypoints.append(prev_point)
            prev_point = path[i - 1]

    if path[-1] not in waypoints:
        waypoints.append(path[-1])

    return waypoints

def dist(start, end):
    return sqrt((start[0] - end[0]) ** 2 + (start[1] - end[1]) ** 2)

# Bresenham's line algorithm
def bresenham_line(start, end):
    rr, cc = skimage_line(start[0], start[1], end[0], end[1])
    return np.column_stack((rr, cc))
