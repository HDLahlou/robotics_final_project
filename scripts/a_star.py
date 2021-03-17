#!/usr/bin/env python3
 
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Quaternion, Point, Pose, PoseArray, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Header, String
import math


class Cell:

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
        self.robot_estimate = Pose()

        # MAP AND NODES __________

        self.map_topic = "map"
        self.map = OccupancyGrid()
        print("Init")

        # subscribe to the map server
        rospy.Subscriber(self.map_topic, OccupancyGrid, self.get_map)
        
        self.starting_position = {i: 8, j: 5}
        self.col_num = 13
        self.row_num = 13


        self.open_list = []
        self.closed_list = []

        self.init_map_nodes()

        self.goal = Cell() # the player robot. If successor = goal stop search 

        # ODOMETRY __________

        # subscribe to odom pose 
        
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.update_odometry)

        self.find_path() 

    def find_path(self):
        
        # while open lit is not empty 
        while len(self.open_list) > 0:
            # q = smallest f
            # https://stackoverflow.com/questions/5320871/in-list-of-dicts-find-min-value-of-a-common-dict-field
            
            q = min(self.open_list, key=lambda x:x.f)
            
            # pop q
            
            self.open_list.pop(q)
            
            # generate q's 8 successors and set their parent to q
            successors = []

            # check if succesor is valid
            # if valid, find the values for the successor 
            # order: north south east and west 

            if (check_north(q)):
                successors[0] = Cell()
                successors[0].i = q.i - 1 
                successors[0].j = q.j 
                successors[0].parent_i = q.i
                successors[0].parent_j = q.j 
                successors[0].h = mannhattan_distance(, self.goal)
                successors[0].g = 
                successors[0].f = successors[s].g + successors[s].h
            else: 
                succssors[0] = nan #or something lol 

            if (check_east(q)):
                successors[1] = Cell()
                successors[1].i = q.i 
                successors[1].j = q.j + 1 
                successors[1].parent_i = q.i
                successors[1].parent_j = q.j 
                successors[1].h = mannhattan_distance(, self.goal)
                successors[1].g = 
                successors[1].f = successors[s].g + successors[s].h
            else: 
                succssors[1] = nan #or something lol

            if (check_south(q)):
                successors[2] = Cell()
                successors[2].i = q.i + 1
                successors[2].j = q.i 
                successors[2].parent_i = q.i
                successors[2].parent_j = q.j 
                successors[2].h = mannhattan_distance(, self.goal)
                successors[2].g = 
                successors[2].f = successors[s].g + successors[s].h
            else: 
                succssors[2] = nan #or something lol
                           
            if (check_west(q)):
                successors[3] = Cell()
                successors[3].i = q.i  
                successors[3].j = q.j - 1 
                successors[3].parent_i = q.i
                successors[3].parent_j = q.j 
                successors[3].h = mannhattan_distance(, self.goal)
                successors[3].g = 
                successors[3].f = successors[s].g + successors[s].h
            else: 
                succssors[3] = nan #or something lol
            

            closed_list.append(q)

        # if successor is the goal, stop search
        if successor 

    def find_index(i,j):
        return (i * self.col_num) + j

    def pos_to_map_data_index(y):
        return int((((y-.5)/self.map.info.resolution) - self.map.info.origin.position.y + self.yrange/2) * self.yrange)

    
    # check which directions you can move in 
    def check_north(parent):
            if (parent.i == 0):
                return 0 
            location = pos_to_map_data_index(parent.mapy)

            # check a grid space away and see if you run into a wall 
            for v in range (location, location - xrange * gridsize, -self.xrange):
                if self.map.data[v] = 100 
                return 0 #false, you cannot move north            
            return 1 #true, you can move north 

    def check_south():
            if (parent.i == 12):
                return 0
            
            location = pos_to_map_data_index(parent.mapy)

            # check a grid space away and see if you run into a wall 
            for v in range  (location, location - xrange * gridsize, -self.xrange):
                if self.map.data[v] = 100
                return 0 #false, you cannot move north           
            return 1 #true, you can move north 

    def check_east():
            if (parent.j == 12):
                return 0
            
            location = pos_to_map_data_index(parent.mapy)

            # check a grid space away and see if you run into a wall 
            for v in range  (location, location + gridsize):
                if self.map.data[v] = 100
                return 0 #false, you cannot move north            
            return 1 #true, you can move north 
    
    def check_west(): 
            if (parent.j == 0):
                return 0

            location = pos_to_map_data_index(parent.mapy)

            # check a grid space away and see if you run into a wall 
            for v in range  (location, location - gridsize):
                if self.map.data[v] = 100
                return 0 #false, you cannot move north             
            return 1 #true, you can move north 

    def manhattan_distance(current_cell , goal):
        return math.abs(current_cell.x - goal.x) + math.abs(current_cell.y - goal.y)
    
    def init_map_nodes(self):
        self.open_list = []
        xrange = (self.map.info.width - 60) / 13
        yrange = (self.map.info.height - 60) / 13
        
        for x in range(13):
            for y in range(13):
                cx = ((x)*xrange - ((13/2)*xrange) + self.map.info.origin.position.x)*self.map.info.resolution +.5
                cy = ((y)*yrange - ((13/2)*yrange) + self.map.info.origin.position.y)*self.map.info.resolution + .5
                
                c = Cell(x, y, cx, cy, -1, -1, -1, -1, -1)

                self.open_list.append(c)




    def get_map(self, data):
        self.map = data
        self.xrange = self.map.info.width
        self.yrange = self.map.info.height
        self.gridsize = (self.xrange - 60)/13 
        print("Call back")
        #print(data)

    def update_odometry(self, data):
        self.odom_pose = data.pose.pose

    def run(self):
        rospy.spin()

if __name__ == '__main__':

        rospy.init_node('a_star')
        astar = AStar()
        astar.run()
