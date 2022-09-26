#!/usr/bin/env python3

from typing import List, Tuple
from cbs_mapf.planner import Planner
import cv2
import numpy as np
import matplotlib.pyplot as plt


GRID_SIZE = 5
STAT_OBS = [([10,10],[20,20])]
START = [[584, 83],[575,167]]
GOAL  = [[584, 335],[579,331]]
ROBOT_RADIUS = 1
COLOR = [(255,0,0),(0,0,255),(0,255,0)]
RESULT = []


SAMPLE = [
    [[198, 207], [273, 169], [267, 71], [273, 169], [118, 167], [273, 169], [267, 71], [402, 86], [523, 114], [402, 86], [267, 71], [273, 169], [198, 207]],
    [[389, 499], [445, 466], [383, 392], [433, 287], [581, 297], [581, 199], [581, 297], [433, 287], [498, 395], [383, 392], [445, 466], [556, 508], [622, 431], [632, 379], [586, 380], [581, 297], [433, 287], [383, 392], [445, 466], [389, 499]
    ]]



def assigner_from_fleet(starts: List[List[Tuple[int,int]]],goals: List[List[Tuple[int,int]]]):
    
    return '5'



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
        for x in range (1,blackAndWhiteImage.shape[0]):
            for y in range (1,blackAndWhiteImage.shape[1]):
                pixel = blackAndWhiteImage[x,y]
                # print(pixel)
                if pixel>=10:
                    self.img_copy[x][y]=(255,255,255)
                    self.obs_ind.append((x,y))
                else:
                    self.img_copy[x][y]=(0,0,0)
                    pass

        planner = Planner(grid_size=GRID_SIZE,robot_radius=ROBOT_RADIUS,static_obstacles=self.obs_ind)
        path = planner.plan(starts=START,goals=GOAL,debug=False)
        for i in path:
            RESULT.append(path)


        for i in range(len(path)):
            for j in range(len(path[i])):         
                x_pos = path[i][j][0]
                y_pos = path[i][j][1]
                self.img_copy[y_pos][x_pos] = COLOR[i]
        
        cv2.imshow("Image Copy", self.img_copy)       
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        return RESULT
    
    def run(self,fleet_result):

        start_list = []
        goal_list = []
        obs_list = self.obs_ind
        for route in fleet_result:
            start_list.append(route[0:len(route)-1])
            goal_list.append(route[1:len(route)])
        try:
            assert(len(start_list)==len(goal_list))
            return assigner_from_fleet(start_list,goal_list)
        except AssertionError:
            print('Not equal')

    


r = Traffic_Management()
print(r.print_img('/home/natta/interface_ws/src/full_interface/config/map_demo.pgm'))
# print(r.run(SAMPLE))