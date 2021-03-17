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
        #self.robot_estimate = Pose()

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

        #init the values for goal 
        self.goal.i = 0 
        self.goal.j = 0 

        # ODOMETRY __________

        # subscribe to odom pose 
        
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.update_odometry)

        self.init_map_nodes()

        self.find_path() 

    def find_path(self):
        dest_found = False
        init_node = Cell(0,0, -1, -1, -1, -1, 0, 0,-1)
        self.open_list.append
        # while open lit is not empty 
        while len(self.open_list) > 0:
            # q = node with smallest f
            # https://stackoverflow.com/questions/5320871/in-list-of-dicts-find-min-value-of-a-common-dict-field
            
            q = min(self.open_list, key=lambda x:x.f)
            
            # pop q
            
            self.open_list.pop(q)
            
            # generate q's 8 successors and set their parent to q
            successors = []

            # check if succesor is valid
            # if valid, find the values for the successor 
            # order: north south east and west 

            if (self.check_north(q)):
                successors[0] = Cell()
                successors[0].i = q.i - 1 
                successors[0].j = q.j 
                successors[0].parent_i = q.i
                successors[0].parent_j = q.j 
                successors[0].h = self.mannhattan_distance(successor[0], self.goal)
                index = self.find_index(successors[0].i, successors[0].j)
                successors[0].g = self.cell_details[index] + 1
                successors[0].f = successors[0].g + successors[0].h
            else: 
                succssors[0] = -1  

            if (self.check_east(q)):
                successors[1] = Cell()
                successors[1].i = q.i 
                successors[1].j = q.j + 1 
                successors[1].parent_i = q.i
                successors[1].parent_j = q.j 
                successors[1].h = self.mannhattan_distance(successor[1], self.goal)
                index = self.find_index(successors[1].i, successors[1].j)
                successors[1].g = self.cell_details[index] + 1
                successors[1].f = successors[1].g + successors[1].h
            else: 
                succssors[1] = -1 

            if (self.check_south(q)):
                successors[2] = Cell()
                successors[2].i = q.i + 1
                successors[2].j = q.i 
                successors[2].parent_i = q.i
                successors[2].parent_j = q.j 
                successors[2].h = self.mannhattan_distance(successor[2], self.goal)
                index = self.find_index(successors[2].i, successors[2].j)
                successors[2].g = self.cell_details[index] + 1
                successors[2].f = successors[2].g + successors[2].h
            else: 
                succssors[2] = -1 
                           
            if (self.check_west(q)):
                successors[3] = Cell()
                successors[3].i = q.i  
                successors[3].j = q.j - 1 
                successors[3].parent_i = q.i
                successors[3].parent_j = q.j 
                successors[3].h = self.mannhattan_distance(successor[4], self.goal)
                index = self.find_index(successors[3].i, successors[3].j)
                successors[3].g = self.cell_details[index] + 1
                successors[3].f = successors[3].g + successors[3].h
            else: 
                succssors[3] = -1 
            
            # now, check each successor 
            for s in successors: 
                
                if (not (s == -1)) and self.in_closed_list(s):
                    # if successor is goal stop search 
                    if (s.i == self.goal.i and s.j == self.goal.j):
                        dest_found = True
                        print("Found Destination")
                        g_index = self.find_index(self.goal.i, self.goal.j)
                        self.cell_details[g_index].parent_i = q.i
                        self.cell_details[g_index].parent_j = q.j
                        break; 

                    s_index =  self.find_index(s.i, s.j)
                    if self.cell_details[s_index].f == -1 or self.cell_details[s_index].f > s.f: 
                        self.open_list.append(s)
                        self.cell_details[s_index].f = s.f
                        self.cell_details[s_index].g = s.g
                        self.cell_details[s_index].h = s.h
                        self.cell_details[s_index].parent_i = s.parent_i
                        self.cell_details[s_index].parent_j = s.parent_j

            # push q to the closed list 
            closed_list.append(q)
            if dest_found:
                break
        print("End of search")
        self.trace_path()


    def in_closed_list(self,cell):
        for c in self.closed_list:
            if (c.i == cell.i and c.j == cell.j):
                    return True
        return False 
            

    def find_index(self,i,j):
        return (i * self.col_num) + j

    def pos_to_map_data_index(self,y):
        return int((((y-.5)/self.map.info.resolution) - self.map.info.origin.position.y + self.yrange/2) * self.yrange)

    
    # check which directions you can move in 
    def check_north(self,parent):
            if (parent.i == 0):
                return False 
            location = pos_to_map_data_index(parent.mapy)

            # check a grid space away and see if you run into a wall 
            for v in range (location, location - self.xrange * self.gridsize, -self.xrange):
                if self.map.data[v] > 0:
                    return False #false, you cannot move north            
            return True #true, you can move north 

    def check_south(self,parent):
            if (parent.i == 12):
                return False
            
            location = pos_to_map_data_index(parent.mapy)

            # check a grid space away and see if you run into a wall 
            for v in range  (location, location + self.xrange * self.gridsize, self.xrange):
                if self.map.data[v] > 0:
                    return False #false, you cannot move north           
            return True #true, you can move north 

    def check_east(self,parent):
            if (parent.j == 12):
                return False
            
            location = pos_to_map_data_index(parent.mapy)

            # check a grid space away and see if you run into a wall 
            for v in range  (location, location + self.gridsize):
                if self.map.data[v] > 0:
                    return False #false, you cannot move north            
            return True #true, you can move north 
    
    def check_west(self,parent): 
            if (parent.j == 0):
                return False

            location = pos_to_map_data_index(parent.mapy)

            # check a grid space away and see if you run into a wall 
            for v in range  (location, location - self.gridsize):
                if self.map.data[v] > 0:
                    return False #false, you cannot move north             
            return True #true, you can move north 

    def manhattan_distance(self,current_cell , goal):
        return math.abs(current_cell.i - goal.i) + math.abs(current_cell.j - goal.j)
    
    def init_map_nodes(self):
        self.cell_details = []
        xrange = (self.map.info.width - 60) / 13
        yrange = (self.map.info.height - 60) / 13
        
        for x in range(13):
            for y in range(13):
                cx = ((x)*xrange - ((13/2)*xrange) + self.map.info.origin.position.x)*self.map.info.resolution +.5
                cy = ((y)*yrange - ((13/2)*yrange) + self.map.info.origin.position.y)*self.map.info.resolution + .5
                f = -1
                g = -1
                if (x == self.starting_position.j and y == self.starting_position.i):
                    f = 0
                    g = 0
                c = Cell(x, y, cx, cy, -1, -1, f, g, -1)
                # self.open_list.append(c)
                self.cell_details.append(c)

    # trace the path once the destination has been found 
    def trace_path(self):
        print("The path is ")
        self.path = []
        row = self.goal.i
        col = self.goal.j 

        path = [] 
        
        while (not ((row == self.starting_position.i) and (col = self.starting_position.j))):
            index = self.find_index(row, col)
            cell = self.cell_details[index]
            path.append(cell)
            row = cell.parent_i
            col = cell.parent_j

        start_index = self.find_index(row, col)
        start_cell = self.cell_details[start_index]
        path.append(start_cell)
        
        for c in path:
            print(c.i + ", " + c.j)

        self.path = path

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
