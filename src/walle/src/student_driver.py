#!/usr/bin/env python3

import sys
import rospy
import numpy as np

from new_driver import Driver

from math import atan2, pi, sqrt


class StudentDriver(Driver):
    """
    This class implements the logic to move the robot to a specific place in the world.  All of the
    interesting functionality is hidden in the parent class.
    """

    def __init__(self, threshold=0.1):
        super().__init__("odom")
        # Set the threshold to a reasonable number
        self._threshold = threshold

    def close_enough_to_waypoint(self, distance, target, lidar):
        """
        This function is called perioidically if there is a waypoint set.  This is where you should put any code that
        has a smarter stopping criteria then just checking the distance. See get_twist for the parameters; distance
        is the current distance to the target.
        """
        # Default behavior.
        if distance < self._threshold:
            return True
        return False

        # This function returns an bias of "go left" or "go right"

    def calc_turn_influence(self, lidar, dist_threshold, clamp):
        # First, limit lidar data that is below our distance threshold (which is 3)
        close_scans = np.hstack(np.where(np.array(lidar.ranges) < dist_threshold))

        # Keep track of "total" bias
        influence = 0
        for i in close_scans:
            # Left side
            if i < 90:
                # We know the angle of the scan by its index
                # We use this to make scans which are more forwards more important
                # Scans that are off to the side have less of an influence here

                # Also, objects that are closer have more influence, so
                # I subtract the scan's range from the threshold.

                # I divide by an arbitrary amount to scale the value down
                influence += (i / 90) * ((dist_threshold - lidar.ranges[i]) / 10)
            # Right side
            else:
                # Same process here, but objects on the right side subtract from the
                # overall influence, which causes the robot to turn left.
                influence -= (1 - ((i - 90) / 90)) * (
                    (dist_threshold - lidar.ranges[i]) / 10
                )

        # Clamp the value if necessary
        return max(min(clamp, influence), -clamp)

    def get_twist(self, target, lidar):
        """
        This function is called whenever there a current target is set and there is a lidar data
        available.  This is where you should put your code for moving the robot.  The target point
        is in the robot's coordinate frame.  The x-axis is positive-forwards, and the y-axis is
        positive to the left.

        The example sets constant velocities, which is clearly the wrong thing to do.  Replace this
        code with something that moves the robot more intelligently.

        Parameters:
                target:		The current target point, in the coordinate frame of the robot (base_link) as
                                        an (x, y) tuple.
                lidar:		A LaserScan containing the new lidar data.

        Returns:
                A Twist message, containing the commanded robot velocities.
        """
        angle = atan2(target[1], target[0])
        distance = sqrt(target[0] ** 2 + target[1] ** 2)
        rospy.loginfo(f"Distance: {distance:.2f}, angle: {angle:.2f}")

        # This builds a Twist message with all elements set to zero.
        command = Driver.zero_twist()

        # Define a distance we consider "close"
        close_distance = 1

        # If we're approaching a goal, limit our speed
        # Otherwise, we approach quickly
        if distance < close_distance:
            command.linear.x = 0.2
        else:
            command.linear.x = min(0.15, 0.05 * distance)

        # How close should we start considering obstacles?
        # I determined this to be 3
        scan_distance = 3
        # Calculate the influence of the obstances in the robot's view
        obstacle_influence = self.calc_turn_influence(lidar, scan_distance, 5)

        # If any lidar scan comes in close, apply bias
        # from our surrouding obstances to navigate
        if distance < close_distance:
            command.angular.z = 6 * (angle / pi)
        else:
            command.angular.z = 4 * (angle / pi) + (obstacle_influence / 4)

        return command


if __name__ == "__main__":
    rospy.init_node("student_driver", argv=sys.argv)
    driver = StudentDriver()
    rospy.spin()
