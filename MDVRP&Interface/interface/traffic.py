#!/usr/bin/env python3
from typing import Callable, List, Tuple
from cbs_mapf.planner import Planner
import cv2 
import numpy as np
import matplotlib.pyplot as plt
from agent import Agent
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16

GRID_SIZE = 20
ROBOT_RADIUS = 5 
COLOR = [(255,0,0),(0,0,255),(0,255,0)]
RESULT = []

MAP_PATH  ='/home/natta/interface_ws/src/full_interface/config/map_demo.pgm'

SAMPLE =    [[[129, 164], [233, 159], [245, 69], [323, 101], [245, 69], [233, 159], [129, 164]], 
             [[226, 427], [245, 384], [350, 421], [377, 314], [299, 242], [323, 320], [262, 316], [222, 294], [299, 242], [381, 163], [299, 242], [262, 316], [245, 384], [226, 427]]]

# SAMPLE =    [
#             [[198, 207], [273, 169] ],
#             [[389, 499], [445, 466], [383, 392], [433, 287]]
#             ]

PIXEL = [[[18.06, 23.233333333333334], [32.62, 22.525], [34.3, 9.775], [45.22, 14.308333333333334], [34.3, 9.775], [32.62, 22.525], [18.06, 23.233333333333334]],
        [[31.64, 60.49166666666667], [34.3, 54.4], [49.0, 59.641666666666666], [52.78, 44.483333333333334], [41.86, 34.28333333333333], [45.22, 45.333333333333336], [36.68, 44.766666666666666], [31.08, 41.65], [41.86, 34.28333333333333], [53.34, 23.091666666666665], [41.86, 34.28333333333333], [36.68, 44.766666666666666], [34.3, 54.4], [31.64, 60.49166666666667]]]
map_demo_size = [112,85]


class Traffic_Management():
    def __init__(self):
        # super().__init__('traffic_manager')
        self.grid_size = GRID_SIZE
        self.start = []
        self.goal = []
        self.obs_list = []
        self.obs_ind = []

    def optimal_plan(self,start_list,goal_list,obstacle):
        # self.obs_ind = self.get_obstacle_ind(map_path)
        self.obs_ind  =obstacle
        # all_start_list,all_goal_list = self.prepare_data(fleet_result)
        planner = Planner(grid_size=GRID_SIZE,robot_radius=ROBOT_RADIUS,static_obstacles=self.obs_ind)
        path = planner.plan(starts= start_list,goals=goal_list,debug=False,assign=direct_assigner)
        return path
        

        

    def get_obstacle_ind(self,name):
        self.map_img = name 
        originalImage = cv2.imread(name)
        imS = cv2.resize(originalImage, (500, 500))
        grayImage = cv2.cvtColor(imS, cv2.COLOR_BGR2GRAY)      
        (thresh, blackAndWhiteImage) = cv2.threshold(grayImage, 127, 255, cv2.THRESH_BINARY)
        print('Converted to Gray Scale')
        self.img_copy = imS.copy()
        for w in range (1,blackAndWhiteImage.shape[0]):
            for h in range (1,blackAndWhiteImage.shape[1]):
                pixel = blackAndWhiteImage[w,h]
                if pixel>=10:
                    self.img_copy[w][h]=(255,255,255)                 
                else:
                    self.obs_ind.append((h,w))
                    self.img_copy[w][h]=(0,0,0)
        return self.obs_ind

    def full_plan(self,name,fleet):
        self.map_img = name 
        originalImage = cv2.imread(name)
        imS = cv2.resize(originalImage, (500, 500))
        grayImage = cv2.cvtColor(imS, cv2.COLOR_BGR2GRAY)      
        (thresh, blackAndWhiteImage) = cv2.threshold(grayImage, 127, 255, cv2.THRESH_BINARY)
        print('Converted to Gray Scale')
        self.img_copy = imS.copy()
        for w in range (1,blackAndWhiteImage.shape[0]):
            for h in range (1,blackAndWhiteImage.shape[1]):
                pixel = blackAndWhiteImage[w,h]
                if pixel>=10:
                    self.img_copy[w][h]=(255,255,255)                 
                else:
                    self.obs_ind.append((h,w))
                    self.img_copy[w][h]=(0,0,0)
        all_start_list,all_goal_list = self.prepare_data(fleet)
        planner = Planner(grid_size=GRID_SIZE,robot_radius=ROBOT_RADIUS,static_obstacles=self.obs_ind)
        
        for i in range(len(all_start_list)):
            path = planner.plan(starts= all_start_list[i],goals=all_goal_list[i],debug=True,assign=direct_assigner)
            for k in path:
                RESULT.append(path)

            for z in range(len(path)):
                for j in range(len(path[z])):         
                    x_pos = path[z][j][1]
                    y_pos = path[z][j][0]
                    self.img_copy[x_pos][y_pos] = COLOR[z%3]

        
        cv2.imshow("Image Copy", self.img_copy)       
        cv2.waitKey(0)
        cv2.destroyAllWindows()
 
    
    def prepare_data(self,fleet_result):
        start_list = []
        goal_list = []
        all_start_list = []
        all_goal_list = []
        max_len = 0
        diff = 0
        dif_len = lambda x,y : x-y
        for route in fleet_result:
            start_list.append(route[0:len(route)-1])
            goal_list.append(route[1:len(route)])    
        for i in start_list:
            if len(i)>max_len:
                max_len = len(i)
        for l in range(len(start_list)):
            if dif_len(len(start_list[l]),max_len)<0:
                what_append = start_list[l][-1]
                what_goal_append = goal_list[l][-1]
                diff = abs(dif_len(len(start_list[l]),max_len))
                for j in range(diff):
                    start_list[l].append(what_append)
                    goal_list[l].append(what_goal_append)


        for i in range(0,len(start_list[0])):
            all_start_list.append([])
            all_goal_list.append([])
        for i in range(0,len(start_list)):
            for j in range(0,len(start_list[0])):
                all_start_list[j].append(start_list[i][j])
        for i in range(0,len(goal_list)):
            for j in range(0,len(goal_list[0])):
                all_goal_list[j].append(goal_list[i][j])

        return all_start_list,all_goal_list

    def equal_len(self,fleet_res):
        max_len = 0
        for i in range(len(fleet_res)):
            if len(fleet_res[i])>max_len:
                max_len = len(fleet_res[i])

        for i in range(len(fleet_res)):
            if len(fleet_res[i])<max_len:
                # fleet_res[i] = fleet_res[i].append(fleet_res[i][-1])
                this_len = len(fleet_res[i])
                for j in range(max_len-this_len):
                    fleet_res[i].append(fleet_res[i][-1]) 
        return fleet_res

def direct_assigner(starts: List[Tuple[int, int]], goals: List[Tuple[int, int]]):
    assert(len(starts) == len(goals))
    agents = []
    for i, start in enumerate(starts):
        agents.append(Agent(start, goals[i]))
    return agents




# r = Traffic_Management()
# print(r.print_img('/home/natta/interface_ws/src/full_interface/config/tormap2.png'))
# print(r.print_img('/home/natta/interface_ws/src/full_interface/config/map_demo.pgm'))
# check = r.prepare_data(SAMPLE)
# m = r.equal_len(SAMPLE)
# print(m)



# def main(args=None):
#     rclpy.init(args=args)
#     traffic_manager = Traffic_Management()
#     print(traffic_manager.full_plan(name=MAP_PATH))
#     rclpy.spin(traffic_manager)


# if __name__ == '__main__':
#     main()
