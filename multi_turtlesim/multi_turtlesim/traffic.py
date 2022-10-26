#!/usr/bin/env python3
from time import time
from typing import  List, Tuple
from cbs_mapf.planner import Planner
import cv2 
import numpy as np
import matplotlib.pyplot as plt
from cbs_mapf.agent import Agent
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16

GRID_SIZE = 20
ROBOT_RADIUS = 8
COLOR = [(255,0,0),(0,0,255),(0,255,0)]
RESULT = []

# MAP_PATH  ='/home/natta/interface_ws/src/full_interface/config/map_example0.png'

# SAMPLE =    [[[129, 164], [233, 159], [245, 69], [323, 101], [245, 69], [233, 159], [129, 164]], 
#              [[226, 427], [245, 384], [350, 421], [377, 314], [299, 242], [323, 320], [262, 316], [222, 294], [299, 242], [381, 163], [299, 242], [262, 316], [245, 384], [226, 427]]]



class Traffic_Management():
    def __init__(self):
        # super().__init__('traffic_manager')
        self.grid_size = GRID_SIZE
        self.start = []
        self.goal = []
        self.current_goal =[]
        self.obs_list = []
        self.obs_ind = []
        self.index = 0
        self.current_index =[]
        
    def get_plan_data_list(self,x,id):
        start = []
        goal = []
        for i in range(len(x)):
            start.append(x[i][id])
            goal.append(x[i][id+1])
        return start,goal
        
    def initial(self,map_path,fleet:List[Tuple[int, int]]):
        def int_path(x):
            n=0
            new_path = []
            for l in range(len(x)):
                new_path.append([])
            for i in x:
                for j in i:
                    new_path[n].append([int(j[0]),int(j[1])])
                n+=1
            return new_path    
        self.obs_ind  = self.get_obstacle_ind(map_path)
        self.planner = Planner(grid_size=GRID_SIZE,robot_radius=ROBOT_RADIUS,static_obstacles=self.obs_ind)
        self.fleet_pixel_equal = self.equal_len(int_path(fleet))
        for i in range(len(self.fleet_pixel_equal)):
            self.current_index.append(0)
        self.max_index = len(self.fleet_pixel_equal[0])
        #First path for Inintial
        start,goal = self.get_plan_data_list(self.fleet_pixel_equal,self.index)
        self.current_goal = goal.copy()
        path = self.planner.plan(starts= start,goals=goal,debug=False,assign=direct_assigner)
        return path
    
    def optimal_plan(self,Trigger=None,arrive_id=None,current_all_pos=None):
        to_int = lambda x : [[int(round(x[0][0])),int(round(x[0][1]))],[int(round(x[1][0])),int(round(x[1][1]))]]
        if Trigger: 
            print(f'prev : {self.current_goal}')
            self.current_index[arrive_id] +=1
            q,new_goal = self.get_plan_data_list(self.fleet_pixel_equal,self.current_index[arrive_id]) 
            self.current_goal[arrive_id] = to_int(new_goal)[arrive_id]
            print(f'after : {self.current_goal}')
            start = to_int(current_all_pos)
            path = self.planner.plan(starts= start,goals=self.current_goal,debug=False,assign=direct_assigner)
            
        return path
    
    
    def get_obstacle_ind(self,name):
        self.map_img = name 
        originalImage = cv2.imread(name)
        imS = cv2.resize(originalImage, (800, 800))
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




# def main(args=None):
#     rclpy.init(args=args)
#     traffic_manager = Traffic_Management()
#     print(traffic_manager.full_plan(name=MAP_PATH))
#     rclpy.spin(traffic_manager)

# def main(args=None):
#     state = 0
#     RUN = True
#     First_path = traffic.initial(map_path=MAP_PATH,fleet=PATH)
#     a = traffic.optimal_plan(Trigger=True,arrive_id=1,current_all_pos=[[143.0, 211.0],[440, 700]])
#     print(a)
    # while RUN:
    #     a = traffic.optimal_plan(Trigger=True)
    #     print(a)
    #     break
        


# if __name__=='__main__':
#     PATH = [[[59.0, 216.0], [143.0, 211.0], [135.0, 94.0], [251.0, 92.0], [135.0, 94.0], [143.0, 211.0], [59.0, 216.0]], 
#     [[440.0, 700.0], [436.0, 593.0], [451.0, 290.0], [325.0, 287.0], [327.0, 208.0], [325.0, 287.0], [451.0, 290.0], [436.0, 593.0], [549.0, 599.0], [703.0, 708.0], [559.0, 478.0], [549.0, 599.0], [436.0, 593.0], [451.0, 290.0], [325.0, 287.0], [249.0, 290.0], [325.0, 287.0], [451.0, 290.0], [436.0, 593.0], [440.0, 700.0]]]
#     essential_pos = [[[59, 216], [440, 700]], [[251, 92], [327, 208], [249, 290], [559, 478], [703, 708]]]
#     MAP_PATH = '/home/natta/interface_ws/src/full_interface/config/map_example0.png'
        
#     traffic = Traffic_Management()

#     main()
