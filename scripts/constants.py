#!/usr/bin/env python3
"""
Constants used throughout our modules.
"""

QUEUE_SIZE = 10

MAP_TOPIC = "map"
SCAN_TOPIC = "scan"
ODOM_TOPIC = "odom"
CMD_VEL_TOPIC = "cmd_vel"

CENTER_MOVEMENT = "center_movement"
JUNCTION_MOVEMENT = "junction_movement"
TURN_AROUND = "turn_around"

RIGHT = "right"
LEFT = "left"
FRONT = "front"
BACK = "back"

# Minimum diff value for bot to detect a junction
TURN_DIFF_THRESHOLD = 1
