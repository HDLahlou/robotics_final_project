#!/usr/bin/env python3

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Quaternion, Point, Pose, PoseArray, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Header, String
import math
import numpy as np


class Cell:

    # Every square of the maze is a cell
    def __init__(self, i, j, x, y, parent_i, parent_j, f, g, h):
        self.i = i
        self.j = j
        self.mapx = x
        self.mapy = y
        self.parent_i = parent_i
        self.parent_j = parent_j
        self.f = f
        self.g = g
        self.h = h


class AStar:
    def __init__(self):

        # set starting position cell and goal cell
        self.starting_position = Cell(10, 10, -1, -1, -1, -1, 0, 0, -1)
        self.goal = Cell(
            4, 2, -1, -1, -1, -1, -1, -1, -1
        )  # the player robot. If successor = goal stop search

        # map setup
        self.col_num = 13
        self.row_num = 13
        self.map_topic = "map"
        self.map = OccupancyGrid()

        print("Initiaizing...")
        self.map_flag = False
        self.nodes_flag = False

        # subscribe to the map server
        rospy.Subscriber(self.map_topic, OccupancyGrid, self.get_map)

        while not self.map_flag:
            pass

        # lists for A* search algorithm
        self.open_list = []
        self.closed_list = []

        self.init_map_nodes()

        while not self.nodes_flag:
            pass

        # subscribe to odom pose
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.update_odometry)

        # find the shortest path from starting cell to goal cell
        self.find_path()

    def find_path(self):

        print("START")
        print(self.starting_position.i)
        print(self.starting_position.j)
        print("GOAL")
        print(self.goal.i)
        print(self.goal.j)
        dest_found = False
        self.open_list.append(
            self.cell_details[
                self.find_index(self.starting_position.i, self.starting_position.j)
            ]
        )

        # while open list is not empty
        while len(self.open_list) > 0:

            # q = node with smallest f
            q_temp = min(self.open_list, key=lambda x: x.f)
            q = self.cell_details[self.find_index(q_temp.i, q_temp.j)]

            # pop q
            self.open_list.remove(q_temp)

            # generate q's 8 successors and set their parent to q
            successors = [-1, -1, -1, -1]

            # for each direction, check if succesor is valid
            # if valid, find the values for the successor

            if self.check_north(q):
                successors[0] = Cell(-1, -1, -1, -1, -1, -1, -1, -1, -1)
                successors[0].i = q.i - 1
                successors[0].j = q.j
                successors[0].parent_i = q.i
                successors[0].parent_j = q.j
                successors[0].h = self.manhattan_distance(successors[0], self.goal)
                index = self.find_index(q.i, q.j)
                successors[0].g = self.cell_details[index].g + 1
                successors[0].f = successors[0].g + successors[0].h

            if self.check_east(q):
                successors[1] = Cell(-1, -1, -1, -1, -1, -1, -1, -1, -1)
                successors[1].i = q.i
                successors[1].j = q.j + 1
                successors[1].parent_i = q.i
                successors[1].parent_j = q.j
                successors[1].h = self.manhattan_distance(successors[1], self.goal)
                index = self.find_index(q.i, q.j)
                successors[1].g = self.cell_details[index].g + 1
                successors[1].f = successors[1].g + successors[1].h

            if self.check_south(q):
                successors[2] = Cell(-1, -1, -1, -1, -1, -1, -1, -1, -1)
                successors[2].i = q.i + 1
                successors[2].j = q.j
                successors[2].parent_i = q.i
                successors[2].parent_j = q.j
                successors[2].h = self.manhattan_distance(successors[2], self.goal)
                index = self.find_index(q.i, q.j)
                successors[2].g = self.cell_details[index].g + 1
                successors[2].f = successors[2].g + successors[2].h

            if self.check_west(q):
                successors[3] = Cell(-1, -1, -1, -1, -1, -1, -1, -1, -1)
                successors[3].i = q.i
                successors[3].j = q.j - 1
                successors[3].parent_i = q.i
                successors[3].parent_j = q.j
                successors[3].h = self.manhattan_distance(successors[3], self.goal)
                index = self.find_index(q.i, q.j)
                successors[3].g = self.cell_details[index].g + 1
                successors[3].f = successors[3].g + successors[3].h

            # now, check each successor
            for s in successors:
                if not (s == -1):
                    s_index = self.find_index(s.i, s.j)
                    # print( "Node:" + str(s.i) + "," + str(s.j) + " f: " + str(s.f))
                    # print(self.cell_details[s_index].f)

                    if not self.in_closed_list(s):

                        # if successor is the goal stop, search
                        # print( "S:" + str(s.i) + "," + str(s.j) + " f: " + str(s.f)
                        if s.i == self.goal.i and s.j == self.goal.j:
                            dest_found = True
                            print("Found Destination")
                            g_index = self.find_index(self.goal.i, self.goal.j)
                            self.cell_details[g_index].parent_i = q.i
                            self.cell_details[g_index].parent_j = q.j
                            break

                        # if successor has lower f than the current f of that node's cell details, add to open list
                        # and update cell details
                        if (
                            self.cell_details[s_index].f == -1
                            or self.cell_details[s_index].f > s.f
                        ):
                            self.open_list.append(s)
                            self.cell_details[s_index].f = s.f
                            self.cell_details[s_index].g = s.g
                            self.cell_details[s_index].h = s.h
                            self.cell_details[s_index].parent_i = s.parent_i
                            self.cell_details[s_index].parent_j = s.parent_j

            # push q to the closed list
            self.closed_list.append(q)
            if dest_found:
                break

        print("End of search")

        # trace the path by starting at the goal cell
        # and tracing back the parent nodes until you reach the starting cell
        self.trace_path()

    # check if a cell is in closed list
    def in_closed_list(self, cell):
        for c in self.closed_list:
            if c.i == cell.i and c.j == cell.j:
                return True
        return False

    # get 1D array index from 2D array indexes
    def find_index(self, i, j):
        return (i * self.col_num) + j

    # get index of occupancy grid cell 2d aray indexes
    def node_to_occ(self, i, j):
        x = 38 + 0.5 * self.gridsize + self.gridsize * i
        y = 38 + 0.5 * self.gridsize + self.gridsize * j
        occdex = self.xrange * y + x
        return int(occdex)

    # check which directions you can move in
    def check_west(self, parent):

        location = self.node_to_occ(parent.i, parent.j)
        # check a grid space away and see if you run into a wall
        for v in range(location, location - self.xrange * self.gridsize, -self.xrange):
            if self.truemap[v] > 0:
                return False  # false, you cannot move west
        return True

    def check_south(self, parent):

        location = self.node_to_occ(parent.i, parent.j)
        # # check a grid space away and see if you run into a wall
        for v in range(location, location + self.gridsize):
            if self.truemap[v] > 0:
                return False  # false, you cannot move south
        return True

    def check_east(self, parent):

        location = self.node_to_occ(parent.i, parent.j)
        # check a grid space away and see if you run into a wall
        for v in range(location, location + self.xrange * self.gridsize, self.xrange):

            if self.truemap[v] > 0:
                return False  # false, you cannot move east
        return True

    def check_north(self, parent):

        location = self.node_to_occ(parent.i, parent.j)
        # check a grid space away and see if you run into a wall
        for v in range(location - self.gridsize, location):
            if self.truemap[v] > 0:
                return False  # false, you cannot move north
        return True

    def manhattan_distance(self, current_cell, goal):
        return abs(current_cell.i - goal.i) + abs(current_cell.j - goal.j)

    def init_map_nodes(self):
        self.cell_details = []
        xrange = (self.map.info.width - 60) / 13
        yrange = (self.map.info.height - 60) / 13

        for row in range(13):
            for col in range(13):
                cx = (
                    (row) * xrange
                    - ((13 / 2) * xrange)
                    + self.map.info.origin.position.x
                ) * self.map.info.resolution + 0.5
                cy = (
                    (col) * yrange
                    - ((13 / 2) * yrange)
                    + self.map.info.origin.position.y
                ) * self.map.info.resolution + 0.5

                f = -1
                g = -1
                if row == self.starting_position.j and col == self.starting_position.i:
                    f = 0
                    g = 0
                c = Cell(row, col, cx, cy, -1, -1, f, g, -1)

                self.cell_details.append(c)
        self.nodes_flag = True
        print("FINISHED INIT NODES")

    # trace the path once the destination has been found
    def trace_path(self):
        print("The path is ")
        self.path = []
        row = self.goal.i
        col = self.goal.j

        path = []

        while not (
            (row == self.starting_position.i) and (col == self.starting_position.j)
        ):
            index = self.find_index(row, col)
            cell = self.cell_details[index]
            path.append(cell)
            row = cell.parent_i
            col = cell.parent_j

        start_index = self.find_index(row, col)
        start_cell = self.cell_details[start_index]
        path.append(start_cell)

        path = path[::-1]
        for c in path:
            print(str(c.i) + ", " + str(c.j))

        self.path = path

    def get_map(self, data):
        self.map = data
        self.xrange = self.map.info.width
        self.yrange = self.map.info.height
        self.gridsize = 70

        # rotate occupancy grid to match map
        h = self.map.info.height
        w = self.map.info.width

        temp = np.reshape(np.array(self.map.data), (h, w))

        new = np.rot90(temp)
        self.truemap = new.flatten()

        self.map_flag = True

    def update_odometry(self, data):
        self.odom_pose = data.pose.pose

    def run(self):
        rospy.spin()


if __name__ == "__main__":

    rospy.init_node("laser_a_star")
    astar = AStar()
    astar.run()
