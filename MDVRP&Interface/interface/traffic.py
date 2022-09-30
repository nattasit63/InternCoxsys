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


GRID_SIZE = 5
STAT_OBS = [([10,10],[20,20])]
# START = [[584, 83],[575,167]]
# GOAL  = [[584, 335],[579,331]]
START = [[198, 207], [389, 499]]
GOAL = [[273, 169], [445, 466]]
ROBOT_RADIUS = 3 
COLOR = [(255,0,0),(0,0,255),(0,255,0)]
RESULT = []


SAMPLE =    [
            [[198, 207], [273, 169], [267, 71], [273, 169], [118, 167], [273, 169], [267, 71], [402, 86], [523, 114], [402, 86], [267, 71], [273, 169], [198, 207]],
            [[389, 499], [445, 466], [383, 392], [433, 287], [581, 297], [581, 199], [581, 297], [433, 287], [498, 395], [383, 392], [445, 466], [556, 508], [622, 431], 
             [632, 379], [586, 380], [581, 297], [433, 287], [383, 392], [445, 466], [389, 499]]
            ]





class Traffic_Management(Node):
    def __init__(self):
        super().__init__('traffic_manager')
 
        self.grid_size = GRID_SIZE
        self.start = START
        self.goal = GOAL
        self.obs_list = STAT_OBS
        self.obs_ind = []
    #     self.publisher_ = self.create_publisher(Int16, 'traffic_data', 10)
    #     timer_period = 0.5  # seconds
    #     self.timer = self.create_timer(timer_period, self.timer_callback)
    
    # def timer_callback(self):
    #     msg = Int16()
    #     msg.data = 3
    #     self.publisher_.publish(msg)


    def get_obstacle_ind(self,name):
        self.map_img = name 
        originalImage = cv2.imread(name)
        imS = cv2.resize(originalImage, (800, 600))
        grayImage = cv2.cvtColor(imS, cv2.COLOR_BGR2GRAY)      
        (thresh, blackAndWhiteImage) = cv2.threshold(grayImage, 127, 255, cv2.THRESH_BINARY)
        print('Converted to Gray Scale')

        self.img_copy = imS.copy()
        # print(imS.shape)
        for w in range (1,blackAndWhiteImage.shape[0]):
            for h in range (1,blackAndWhiteImage.shape[1]):
                pixel = blackAndWhiteImage[w,h]
                # print(pixel)
                if pixel>=10:
                    self.img_copy[w][h]=(255,255,255)                 
                else:
                    # print(x,y)
                    self.obs_ind.append((h,w))
                    self.img_copy[w][h]=(0,0,0)
        return self.obs_ind

    def full_plan(self,name):
        self.map_img = name 
        originalImage = cv2.imread(name)
        imS = cv2.resize(originalImage, (800, 600))
        grayImage = cv2.cvtColor(imS, cv2.COLOR_BGR2GRAY)      
        (thresh, blackAndWhiteImage) = cv2.threshold(grayImage, 127, 255, cv2.THRESH_BINARY)
        print('Converted to Gray Scale')

        self.img_copy = imS.copy()
        # print(imS.shape)
        for w in range (1,blackAndWhiteImage.shape[0]):
            for h in range (1,blackAndWhiteImage.shape[1]):
                pixel = blackAndWhiteImage[w,h]
                # print(pixel)
                if pixel>=10:
                    self.img_copy[w][h]=(255,255,255)                 
                else:
                    # print(x,y)
                    self.obs_ind.append((h,w))
                    self.img_copy[w][h]=(0,0,0)
        # print(self.obs_ind)
        all_start_list,all_goal_list = self.prepare_data(SAMPLE)
        planner = Planner(grid_size=GRID_SIZE,robot_radius=ROBOT_RADIUS,static_obstacles=self.obs_ind)
        
        for i in range(len(all_start_list)):
            path = planner.plan(starts= all_start_list[i],goals=all_goal_list[i],debug=True,assign=minion)
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
        # return RESULT
    
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


    


# r = Traffic_Management()
# print(r.print_img('/home/natta/interface_ws/src/full_interface/config/tormap2.png'))
# print(r.print_img('/home/natta/interface_ws/src/full_interface/config/map_demo.pgm'))
# print(r.prepare_data(SAMPLE))

def minion(starts: List[Tuple[int, int]], goals: List[Tuple[int, int]]):
    assert(len(starts) == len(goals))
    agents = []
    for i, start in enumerate(starts):
        agents.append(Agent(start, goals[i]))
    return agents


def main(args=None):
    rclpy.init(args=args)
    traffic_manager = Traffic_Management()
    rclpy.spin(traffic_manager)


if __name__ == '__main__':
    main()