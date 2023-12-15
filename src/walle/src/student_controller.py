#!/usr/bin/env python3

# https://github.com/traveno/rob456-final-project
# Running this project will need some additional files!
# This includes ProcessedMsg type for the SLAM node.
# The SLAM processing code is in slam.py. It receives
# the SLAM updates and sends the processed version
# to this controller.

import sys
import rospy
import numpy as np
from std_msgs.msg import Bool

from controller import RobotController
import exploring
import path_planning
from slam import map_to_pixel, pixel_to_map

from walle.msg import ProcessedMap, Path


class StudentController(RobotController):
    """
    This class allows you to set waypoints that the robot will follow.  These robots should be in the map
    coordinate frame, and will be automatially sent to the code that actually moves the robot, contained in
    StudentDriver.
    """

    def __init__(self):
        super().__init__()
        self.pmap_sub = rospy.Subscriber(
            "map_processed", ProcessedMap, self.slam_update
        )
        self.path_pub = rospy.Publisher("path", Path, queue_size=1)
        self.pmap = None
        self.map_info = None
        self.robot_map_yx = None
        self._stuck_subscriber = rospy.Subscriber("robot_stuck", Bool, self.stuck_callback)

    # Helper function for close_enough_to_waypoint (in StudentDriver) if the robot is stuck then force it to calculate a new goal
    def stuck_callback(self, msg):
        if msg.data:
            rospy.loginfo("Robot is stuck!")
            self.set_waypoints([])
            self.find_new_goal()

    def distance_update(self, distance):
        """
        This function is called every time the robot moves towards a goal.  If you want to make sure that
        the robot is making progress towards it's goal, then you might want to check that the distance to
        the goal is generally going down.  If you want to change where the robot is heading to, you can
        make a call to set_waypoints here.  This call will override the current set of waypoints, and the
        robot will start to drive towards the first waypoint in the new list.

        Parameters:
                distance:	The distance to the current goal.
        """
        # rospy.loginfo(f'Distance: {distance}')

    def slam_update(self, pmap_msg: ProcessedMap):
        rospy.loginfo(f"Got SLAM update from callback")

        pmap_data = np.fromiter(pmap_msg.map, int).reshape(-1, pmap_msg.width)
        pmap_ij = (pmap_msg.offset_i, pmap_msg.offset_j)

        with self.mutex:
            self.pmap = pmap_data
            self.pmap_ij = pmap_ij
            self.pmap_meta = pmap_msg.meta

    def find_new_goal(self):
        if self.pmap is None or self.pmap_meta is None:
            return
        
        # Get the robot's map location
        robot_ij = map_to_pixel(self.pmap_meta, self.pmap_ij, self.robot_map_yx)
        rospy.loginfo(f"Robot location {self.robot_map_yx} (map {robot_ij})")

        # Find the "best point"
        goal_ij = exploring.find_best_point(self.pmap, np.argwhere(self.pmap == 254), robot_ij)
        rospy.loginfo(f"Best point {goal_ij}")

        # Make a path to the best point
        path = path_planning.dijkstra(self.pmap, robot_ij, goal_ij)
        rospy.loginfo(f"Path length {len(path)}")

        # Send the path over to the SLAM map annotator (for our image view, doesn't affect robot)
        path_sv = Path()
        path_sv.path = list(sum(path, ()))
        self.path_pub.publish(path_sv)

        # Convert waypoints from pixel coords to map coords
        waypoints = [
            pixel_to_map(self.pmap_meta, self.pmap_ij, (i, j))
            for i, j in exploring.find_waypoints(self.pmap, path, robot_ij)
        ]
        
        rospy.loginfo(f"Waypoints {waypoints}")
        self.set_waypoints((x, y) for y, x in waypoints)

    def pose_update(self, point):
        if point is not None:
            self.robot_map_yx = (point.point.y, point.point.x)
            # Only find a new goal if we have no waypoints
            if self._waypoints is None or len(self._waypoints) == 0:
                self.find_new_goal()


if __name__ == "__main__":
    # Initialize the node.
    rospy.init_node("student_controller", argv=sys.argv)

    # Start the controller.
    controller = StudentController()

    # This will move the robot to a set of fixed waypoints.  You should not do this, since you don't know
    # if you can get to all of these points without building a map first.  This is just to demonstrate how
    # to call the function, and make the robot move as an example.
    # controller.set_waypoints(((-4.9499999, -4.049998), (-4, -5)))

    # Once you call this function, control is given over to the controller, and the robot will start to
    # move.  This function will never return, so any code below it in the file will not be executed.
    controller.send_points()
