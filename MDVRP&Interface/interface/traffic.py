#!/usr/bin/env python3

from typing import Callable, List, Tuple
from cbs_mapf.planner import Planner,min_cost
import cv2
import numpy as np
import matplotlib.pyplot as plt
from agent import Agent
from scipy.optimize import linear_sum_assignment

GRID_SIZE = 5
STAT_OBS = [([10,10],[20,20])]
START = [[584, 83],[575,167]]
GOAL  = [[584, 335],[579,331]]
# START = [[100,100]]
# GOAL = [[200,200]]
ROBOT_RADIUS = 3 
COLOR = [(255,0,0),(0,0,255),(0,255,0)]
RESULT = []

eul = lambda x,y : (x[0]-y[0])**2 + (x[1]-y[1])**2

# print('EUL  :',eul(START[0],GOAL[1]))

SAMPLE =    [
            [[198, 207], [273, 169], [267, 71], [273, 169], [118, 167], [273, 169], [267, 71], [402, 86], [523, 114], [402, 86], [267, 71], [273, 169], [198, 207]],
            [[389, 499], [445, 466], [383, 392], [433, 287], [581, 297], [581, 199], [581, 297], [433, 287], [498, 395], [383, 392], [445, 466], [556, 508], [622, 431], 
             [632, 379], [586, 380], [581, 297], [433, 287], [383, 392], [445, 466], [389, 499]]
            ]

# SAMPLE =    [
#             [[198, 207], [273, 169], [267, 71],[273, 169], [118, 167], [273, 169], [267, 71], [402, 86], [523, 114]]
           
#             ]


def assigner_from_fleet(starts,goals  ):                           
    print('-'*20)
    n= len(starts)
    agent = []
    for i in range(0,n):
        agent.append(starts[i][0])
    return agent


def minion(starts: List[Tuple[int, int]], goals: List[Tuple[int, int]]):
    assert(len(starts) == len(goals))
    agents = []
    for i, start in enumerate(starts):
        # print('satert :',start)
        # print('Goal :',goals[i])
        # agents.append(Agent(start, goals[col_ind[i]]))
        agents.append(Agent(start, goals[i]))
    return agents



class Traffic_Management:
    def __init__(self):
        self.grid_size = GRID_SIZE
        self.start = START
        self.goal = GOAL
        self.obs_list = STAT_OBS
        self.obs_ind = []


    def print_img(self,name):
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
                    pass
        
        # print(self.obs_ind)
        start,goal = self.prepare_data(SAMPLE)
        print('starter :',start[0][0])
        # print('goaler  :',goal[0])
        planner = Planner(grid_size=GRID_SIZE,robot_radius=ROBOT_RADIUS,static_obstacles=self.obs_ind)
        # path = planner.plan(starts=START,goals=GOAL,debug=False,assign=minion)
        
        for i in range(len(start)):
            # path = planner.plan(starts=start[i],goals=goal[i],debug=True,assign=minion)
            # path = planner.plan(starts= [[198, 207]],goals=[[273, 169]],debug=True,assign=minion)
            for l in range(len(start[i])):
                path = planner.plan(starts= [start[i][l]],goals=[goal[i][l]],debug=True,assign=minion)
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
        return RESULT
    
    def prepare_data(self,fleet_result):
        start_list = []
        goal_list = []
        for route in fleet_result:
            start_list.append(route[0:len(route)-1])
            goal_list.append(route[1:len(route)])
        
        return start_list,goal_list
        # try:
        #     assert(len(start_list)==len(goal_list))
        #     return assigner_from_fleet(start_list,goal_list)
        # except AssertionError:
        #     print('Not equal')

    


r = Traffic_Management()
# print(r.print_img('/home/natta/interface_ws/src/full_interface/config/tormap2.png'))
print(r.print_img('/home/natta/interface_ws/src/full_interface/config/map_demo.pgm'))
# print(r.run(SAMPLE))