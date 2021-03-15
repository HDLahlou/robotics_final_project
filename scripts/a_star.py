#!/usr/bin/env python3

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid

@dataclass
class Cell:
    parent_i: int = 0
    parent_j: int = 0
    f: float = 0
    g: float = 0
    h: float = 0

class AStar:

    def __init__(self):
        self.map_topic = "map"
        self.map = OccupancyGrid()
        print("Init")

        # subscribe to the map server
        rospy.Subscriber(self.map_topic, OccupancyGrid, self.get_map)

        self.open_list = []
        self.closed_list = []

    def find_path(self):
        while len(self.open_list) > 0:
            # q = smallest f
            # https://stackoverflow.com/questions/5320871/in-list-of-dicts-find-min-value-of-a-common-dict-field
            q = min(self.open_list, key=lambda x:x.f)
            # pop q
            self.open_list.pop(q)
            # generate q's 8 successors and set their parent to q
            successors = []
            for i in range(8):
                successors[i] = {
                    parent_i: 0,
                    parent_j: 0,
                    f: 0,
                    g:0,
                    h:0
                }

    def get_map(self, data):
        self.map = data
        print("Call back")
        print(data)


    def run(self):
        rospy.spin()

if __name__ == '__main__':

        rospy.init_node('a_star')
        astar = AStar()
        astar.run()
