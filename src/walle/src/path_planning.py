import numpy as np
import heapq
import rospy
from math import sqrt

# -------------- Thresholded image True/False ---------------
def is_wall(im, pix):
    """Is the pixel a wall pixel?
    @param im - the image
    @param pix - the pixel i,j"""
    if im[pix[0], pix[1]] in [0, 1]:
        return True
    return False


def is_unseen(im, pix):
    """Is the pixel one we've seen?
    @param im - the image
    @param pix - the pixel i,j"""
    if im[pix[0], pix[1]] == 128:
        return True
    return False


def is_free(im, pix):
    """Is the pixel empty?
    @param im - the image
    @param pix - the pixel i,j"""
    if im[pix[0], pix[1]] == 255:
        return True
    return False


# -------------- Getting 4 or 8 neighbors ---------------
def four_connected(pix):
    """Generator function for 4 neighbors
    @param im - the image
    @param pix - the i, j location to iterate around"""
    for i in [-1, 1]:
        ret = pix[0] + i, pix[1]
        yield ret
    for i in [-1, 1]:
        ret = pix[0], pix[1] + i
        yield ret


def eight_connected(pix):
    """Generator function for 8 neighbors
    @param im - the image
    @param pix - the i, j location to iterate around"""
    for i in range(-1, 2):
        for j in range(-1, 2):
            if i == 0 and j == 0:
                pass
            ret = pix[0] + i, pix[1] + j
            yield ret


def dijkstra(im, robot_ij, goal_ij):
    """Occupancy grid image, with robot and goal loc as pixels
    @param im - the thresholded image - use is_free(i, j) to determine if in reachable node
    @param robot_loc - where the robot is (tuple, i,j)
    @param goal_loc - where to go to (tuple, i,j)
    @returns a list of tuples"""



    # Sanity check
    # if not is_free(im, robot_loc):
    #     raise ValueError(
    #         f"Start location {robot_loc} is not in the free space of the map"
    #     )

    # if not is_free(im, goal_loc):
    #     raise ValueError(
    #         f"Goal location {goal_loc} is not in the free space of the map"
    #     )

    # The priority queue itself is just a list, with elements of the form (weight, (i,j))
    #    - i.e., a tuple with the first element the weight/score, the second element a tuple with the pixel location
    priority_queue = []
    # Push the start node onto the queue
    #   push takes the queue itself, then a tuple with the first element the priority value and the second
    #   being whatever data you want to keep - in this case, the robot location, which is a tuple
    heapq.heappush(priority_queue, (0, robot_ij))

    # The power of dictionaries - we're going to use a dictionary to store every node we've visited, along
    #   with the node we came from and the current distance
    # This is easier than trying to get the distance from the heap
    visited = {}
    # Use the (i,j) tuple to index the dictionary
    #   Store the best distance, the parent, and if closed y/n
    visited[robot_ij] = (
        0,
        None,
        False,
    )  # For every other node this will be the current_node, distance

    # While the list is not empty - use a break for if the node is the end node
    while priority_queue:
        # Get the current best node off of the list
        current_node = heapq.heappop(priority_queue)
        # Pop returns the value and the i, j
        node_score = current_node[0]
        node_ij = current_node[1]

        # Showing how to get this data back out of visited
        visited_triplet = visited[node_ij]
        visited_distance = visited_triplet[0]
        visited_parent = visited_triplet[1]
        visited_closed_yn = visited_triplet[2]

        #  Step 1: Break out of the loop if node_ij is the goal node
        if node_ij == goal_ij:
            break
        #  Step 2: If this node is closed, skip it
        if visited_closed_yn:
            continue
        #  Step 3: Set the node to closed
        visited[node_ij] = (visited_distance, visited_parent, True)
        # rospy.loginfo(f'{node_ij} {robot_loc}')
        dist_to_robot = sqrt((node_ij[0] - robot_ij[0])**2 + (node_ij[1] - robot_ij[1])**2)

        for neighbor in eight_connected(node_ij):
            if not is_free(im, neighbor):  # skip over if pixel is full
                continue

            # edge weight is just the euclidean distance
            # temp_distance is just current node score + euclidean distance unless there is a wall in the way
            # set all distances to 1
            # if we were doing something more complicated edge_weight would actually be the dist between current i,j and neighbor i,j
            # gonna actually use a special flavor of dijkstra's here instead of A*
            # Uniform cost search! Basically if we've visited a node decrease it's key- else add it to the queue
            # psuedocode here: https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm#Practical_optimizations_and_infinite_graphs

            # first set edge weight
            if node_ij[1] == neighbor[1]:  # if these are the same location
                edge_weight = 1
            else:
                edge_weight = np.sqrt(2)

            # calculate cost to get to next node (current node score + edge_weight)
            cost = node_score + edge_weight

            if neighbor not in visited:
                # add to priority queue and visited dictionary
                visited[neighbor] = (cost, node_ij, False)
                heapq.heappush(priority_queue, (cost, neighbor))
            elif visited[neighbor][0] > cost:  # if already visited and higher cost
                # replace existing neighbor with new tuple with cost = to lower value
                visited[neighbor] = (cost, node_ij, False)

    # Now check that we actually found the goal node
    try_2 = None
    if not goal_ij in visited:
        # TODO: Deal with not being able to get to the goal loc
        # we basically just want to go to the closest node to the goal that we visted
        distance = []
        for i, j in visited:
            dist = np.sqrt((goal_ij[0] - i) ** 2 + (goal_ij[1] - j) ** 2)
            distance.append(dist)

        # find closest distance
        closest_idx = np.argmin(distance)
        k, v = list(visited.items())[
            closest_idx
        ]  # get closest i,j pair out of dictionary
        print("Key:", k)
        print("Val:", v)

        try_2 = k
        if try_2 is not None:
            return dijkstra(im, robot_ij, try_2)
        else:
            raise ValueError(f"Goal {goal_ij} not reached")

    path = []
    path.append(goal_ij)
    # TODO: Build the path by starting at the goal node and working backwards
    current_node = goal_ij
    # loop until we get back to robot location
    while current_node != robot_ij:
        current_node = visited[current_node][
            1
        ]  # get i,j location of current in visited array
        path.append(current_node)

    path.reverse()
    return path
