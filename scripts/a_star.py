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

        self.starting_position = Cell(0, 0, -1, -1, -1, -1, 0, 0, -1)

    

        self.col_num = 13
        self.row_num = 13
        self.map_topic = "map"
        self.map = OccupancyGrid()
        
        print("Init")
        self.map_flag = False
        self.init_map_nodes()
        
        # subscribe to the map server
        rospy.Subscriber(self.map_topic, OccupancyGrid, self.get_map)
        
        while (not self.map_flag):
            pass

      


        self.open_list = []
        self.closed_list = []

        self.init_map_nodes()

        self.goal = Cell(8, 5, -1, -1, -1, -1, -1, -1, -1) # the player robot. If successor = goal stop search 

        # #init the values for goal 
        # self.goal.i = 0 
        # self.goal.j = 0 

        # ODOMETRY __________

        # subscribe to odom pose 
        
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.update_odometry)

        

        self.find_path() 

    def find_path(self):
        dest_found = False
        self.open_list.append(self.starting_position)
        # while open lit is not empty 
        while len(self.open_list) > 0:
            # q = node with smallest f
            # https://stackoverflow.com/questions/5320871/in-list-of-dicts-find-min-value-of-a-common-dict-field
            
            q = min(self.open_list, key=lambda x:x.f)
            print( "Node:" + str(q.i) + "," + str(q.j) + " f: " + str(q.f))
            # pop q
            
            self.open_list.remove(q)
            
            # generate q's 8 successors and set their parent to q
            successors = [-1,-1,-1,-1]

            # check if succesor is valid
            # if valid, find the values for the successor 
            # order: north south east and west 

            if (self.check_north(q)):
                successors[0] = Cell(-1, -1, -1, -1, -1, -1, -1, -1, -1)
                successors[0].i = q.i - 1 
                successors[0].j = q.j 
                successors[0].parent_i = q.i
                successors[0].parent_j = q.j 
                successors[0].h = self.manhattan_distance(successors[0], self.goal)
                index = self.find_index(successors[0].i, successors[0].j)
                successors[0].g = self.cell_details[index].g + 1
                successors[0].f = successors[0].g + successors[0].h

            if (self.check_east(q)):
                successors[1] = Cell(-1, -1, -1, -1, -1, -1, -1, -1, -1)
                successors[1].i = q.i 
                successors[1].j = q.j + 1 
                successors[1].parent_i = q.i
                successors[1].parent_j = q.j 
                successors[1].h = self.manhattan_distance(successors[1], self.goal)
                index = self.find_index(successors[1].i, successors[1].j)
                successors[1].g = self.cell_details[index].g + 1
                successors[1].f = successors[1].g + successors[1].h

            if (self.check_south(q)):
                successors[2] = Cell(-1, -1, -1, -1, -1, -1, -1, -1, -1)
                successors[2].i = q.i + 1
                successors[2].j = q.i 
                successors[2].parent_i = q.i
                successors[2].parent_j = q.j 
                successors[2].h = self.manhattan_distance(successors[2], self.goal)
                index = self.find_index(successors[2].i, successors[2].j)
                successors[2].g = self.cell_details[index].g + 1
                successors[2].f = successors[2].g + successors[2].h
                           
            if (self.check_west(q)):
                successors[3] = Cell(-1, -1, -1, -1, -1, -1, -1, -1, -1)
                successors[3].i = q.i  
                successors[3].j = q.j - 1 
                successors[3].parent_i = q.i
                successors[3].parent_j = q.j 
                successors[3].h = self.manhattan_distance(successors[3], self.goal)
                index = self.find_index(successors[3].i, successors[3].j)
                successors[3].g = self.cell_details[index].g + 1
                successors[3].f = successors[3].g + successors[3].h
            
            # now, check each successor 
            for s in successors: 
                print(s)
                if (not (s == -1)):
                    if (not self.in_closed_list(s)):
                        # if successor is goal stop search
                        print( "S:" + str(s.i) + "," + str(s.j) + " f: " + str(s.f))

                        if (s.i == self.goal.i and s.j == self.goal.j):
                            dest_found = True
                            print("Found Destination")
                            g_index = self.find_index(self.goal.i, self.goal.j)
                            self.cell_details[g_index].parent_i = q.i
                            self.cell_details[g_index].parent_j = q.j
                            break; 

                        s_index =  self.find_index(s.i, s.j)
                        print(s_index)
                        if self.cell_details[s_index].f == -1 or self.cell_details[s_index].f > s.f: 
                            print("Adding to open list")

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
            location = self.pos_to_map_data_index(parent.mapy)
            # print(location)
            # print(self.gridsize)
            # for index in range(self.gridsize):
            #     val = location - index*self.xrange
            #     if val >= 0:
            #         print(self.map.data[val])
            #         if self.map.data[val] > 0:
            #             return False #false, you cannot move north            
            # return True #true, you can move north 
            # check a grid space away and see if you run into a wall 
            for v in range (location, location - self.xrange * self.gridsize, -self.xrange):
                print(v)
                if self.map.data[v] > 0:
                    return False #false, you cannot move north            
            return True #true, you can move north 

    def check_south(self,parent):
            if (parent.i == 12):
                return False
            
            location = self.pos_to_map_data_index(parent.mapy)

            # for index in range(10):
            #     val = location + index*self.xrange
            #     if val >= 0:
            #         if self.map.data[val] > 0:
            #             return False #false, you cannot move north            
            # return True #true, you can move north
            # # check a grid space away and see if you run into a wall 
            for v in range  (location, location + self.xrange * self.gridsize, self.xrange):
                print(v)
                if self.map.data[v] > 0:
                    return False #false, you cannot move north           
            return True #true, you can move north 

    def check_east(self,parent):
            if (parent.j == 12):
                return False
            
            location = self.pos_to_map_data_index(parent.mapy)

            # for index in range(self.gridsize):
            #     val = location + index
            #     if val >= 0:
            #         if self.map.data[val] > 0:
            #             return False #false, you cannot move north            
            # return True #true, you can move north
            # # check a grid space away and see if you run into a wall 
            for v in range  (location, location + self.gridsize):
                print(v)

                if self.map.data[v] > 0:
                    return False #false, you cannot move north            
            return True #true, you can move north 
    
    def check_west(self,parent):

            if (parent.j == 0):
                return False

            location = self.pos_to_map_data_index(parent.mapy)
            
            # for index in range(self.gridsize):
            #     val = location - index
            #     if val >= 0:
            #         if self.map.data[val] > 0:
            #             return False #false, you cannot move north            
            # return True 
            
            #true, you can move north 
            # # check a grid space away and see if you run into a wall 
            for v in range  (location - self.gridsize, location):
                print(v)

                if self.map.data[v] > 0:
                    return False #false, you cannot move north             
            return True #true, you can move north 

    def manhattan_distance(self,current_cell , goal):
        return abs(current_cell.i - goal.i) + abs(current_cell.j - goal.j)
    
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
                # print( "Created:" + str(c.i) + "," + str(c.j) + " f: " + str(c.f))

                # self.open_list.append(c)
                self.cell_details.append(c)

    # trace the path once the destination has been found 
    def trace_path(self):
        print("The path is ")
        self.path = []
        row = self.goal.i
        col = self.goal.j 

        path = [] 
        
        while (not ((row == self.starting_position.i) and (col == self.starting_position.j))):
            index = self.find_index(row, col)
            cell = self.cell_details[index]
            path.append(cell)
            row = cell.parent_i
            col = cell.parent_j

        start_index = self.find_index(row, col)
        start_cell = self.cell_details[start_index]
        path.append(start_cell)
        
        for c in path:
            print(str(c.i) + ", " + str(c.j))

        self.path = path

    def get_map(self, data):
        self.map = data
        self.xrange = self.map.info.width
        self.yrange = self.map.info.height
        self.gridsize = int((self.xrange - 60)/13)
        self.map_flag = True
        print("Call back")
        print(self.xrange)
        print(self.yrange)
        #print(data)

    def update_odometry(self, data):
        self.odom_pose = data.pose.pose

    def run(self):
        rospy.spin()

if __name__ == '__main__':

        rospy.init_node('a_star')
        astar = AStar()
        astar.run()
