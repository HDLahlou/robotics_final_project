#!/usr/bin/env python3
"""
Navigate through hallways while avoiding light and follwing a path.
"""

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header, String
import constants as C


class HallwayNav():

    def __init__(self):
        # once everything is setup initialized will be set to true
        self.initialized = False

        rospy.init_node("HallwayNav")

        # CMD_VEL
        self.velocity_pub = rospy.Publisher(
            C.CMD_VEL_TOPIC, Twist, queue_size=C.QUEUE_SIZE)

        self.min_scan_regions = {}
        self.max_scan_regions = {}

        self.prev_min_scan_regions = None
        self.prev_max_scan_regions = None

        self.current_diff = {}
        self.prev_diff = {}

        # This will be replaced by a subscriber to A* path decision making
        self.next_turn_direction = C.RIGHT

        # State Flags
        self.flags = {}

        # Turning vs Centering Flag
        self.flags[C.CENTER_MOVEMENT] = False
        self.flags[C.JUNCTION_MOVEMENT] = True

        # NOTE: When this is set, all other flags should be false
        self.flags[C.TURN_AROUND] = False

        # subscribe to the lidar scan from the robot
        rospy.Subscriber(C.SCAN_TOPIC, LaserScan, self.robot_scan_received)
        # rospy.Subscriber(C.LIGHT_TOPIC, self.light_received)

        self.initialized = True

    def robot_scan_received(self, msg):
        if self.initialized:

            # Determines the ranges for scan, finds each "side"
            min_regions = {
                C.RIGHT:  min(min(msg.ranges[45:135]), 10),
                C.FRONT:  min(min(msg.ranges[0:45]), min(msg.ranges[315:]), 10),
                C.LEFT:   min(min(msg.ranges[225:315]), 10),
                C.BACK:   min(min(msg.ranges[135:225]), 10),
            }
            max_regions = {
                C.RIGHT:  min(max(msg.ranges[45:135]), 10),
                C.FRONT:  min(max(msg.ranges[0:45]), max(msg.ranges[315:]), 10),
                C.LEFT:   min(max(msg.ranges[225:315]), 10),
                C.BACK:   min(max(msg.ranges[135:225]), 10),
            }
            # Sets current scan region values
            self.max_scan_regions = max_regions
            self.min_scan_regions = min_regions

            # Avoids on init edge cases by giving prev_scan_regions a value
            if self.prev_max_scan_regions is None:
                self.prev_max_scan_regions = self.max_scan_regions
            if self.prev_min_scan_regions is None:
                self.prev_min_scan_regions = self.min_scan_regions

            # Calculate difference between min/max scan values to detect junctions
            self.current_diff = {}
            for key in min_regions:
                self.current_diff[key] = self.max_scan_regions[key] - \
                    self.min_scan_regions[key]

            if self.next_turn_direction == C.RIGHT and self.current_diff[C.RIGHT] > C.TURN_DIFF_THRESHOLD:
                pass

            # End of callback, sets previous scan data to current
            self.prev_max_scan_regions = self.max_scan_regions
            self.prev_min_scan_regions = self.min_scan_regions
            self.prev_diff = self.current_diff

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('HallwayNav')
    hNav = HallwayNav()
    hNav.run()
