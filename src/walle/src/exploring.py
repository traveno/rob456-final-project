import numpy as np
import path_planning as path_planning
<<<<<<< HEAD
# Our priority queue
import heapq

# Using imageio to read in the image
import imageio


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
    
    
    # Used to double check that the is_xxx routines work correctly
    for i in range(0, im_threshhold.shape[1]-1, 10):
        for j in range(0, im_threshhold.shape[0]-1, 2):
            if is_reachable(im_threshhold, (i, j)):
                axs[1].plot(i, j, '.b')
    
    

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

    for i in path_planning.eight_connected(pix): #loop through all neighbors
        if path_planning.is_free(im, i): #check if pixel in neighbors is free
            return True
        else:
            return False
    

def find_all_possible_goals(im):
    """ Find all of the places where you have a pixel that is unseen next to a pixel that is free
    It is probably easier to do this, THEN cull it down to some reasonable places to try
    This is because of noise in the map - there may be some isolated pixels
    @param im - thresholded image
    @return dictionary or list or binary image of possible pixels"""

    #first find all empty pixels
    empty = np.where(im == 255)

    #now the goal is to find all unseen pixels connected to the empty pixels
    possible = [] #array of (i,j) tuples
    for i in empty: #loop through empty spaces
        for pix in path_planning.eight_connected(i): #check neighbors of empty pixels
            if np.any(im[pix[0]] == 128) and np.any(im[pix[1]] == 128): #check for unseen pixels
                #check if reachable (sanity check):
                if is_reachable(im, pix):
                    possible.append(pix) #append unseen pixel location to list

    return possible


=======
from scipy.signal import correlate2d
from math import atan2, sqrt, pi
from skimage.draw import line as skimage_line
>>>>>>> main

def find_best_point(pmap, possible_points, robot_ij):
    """Pick one of the unseen points to go to
    @param im - thresholded image
    @param possible_points - possible points to chose from
    @param robot_loc - location of the robot (in case you want to factor that in)
    """
<<<<<<< HEAD
    #starting with the quick and dirtiest way of doing this- let's call the "best" point the point closest to the robot
    distances = []
    for i in possible_points:
        #calculate euclidean distance for each point and save to array
        dist = np.sqrt((i[0] - robot_loc[0]) ** 2 + (i[1] - robot_loc[1]) ** 2)
        distances.append(dist)

    #find location of closest distance
    closest_idx = np.argmin(distances)
    best_ij = possible_points[closest_idx]
    return best_ij
=======

    if len(possible_points) == 0:
        return None
>>>>>>> main

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

<<<<<<< HEAD
    #naive sample approach (just to see output)
    #path starts at endpoint and loops back to robot's location
    cp_path = path #copy path to new variable
    cp_path = list(reversed(path)) #reverse list for sampling
    
    count = 0
    goal_point = cp_path[-1]
    #print(goal_point)
    new_path = [cp_path[0]]
    for i in cp_path:
        count += 1
        if count % 20 == 0:
            new_path.append(i)

    if goal_point not in new_path:
        new_path.append(goal_point)

    #print(new_path)
    #reverse the new path again for plotting
    new_path = list(reversed(new_path))
    return new_path

        

if __name__ == '__main__':
    # Doing this here because it is a different yaml than JN
    import yaml_1 as yaml
=======
    waypoints = []
>>>>>>> main

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
