#!/usr/bin/env python3
'''
Author: Haoran Peng
Email: gavinsweden@gmail.com
'''


import time
import sys
from copy import deepcopy
import cv2
import numpy as np
import yaml

from cbs_mapf.planner import Planner, Agent

from typing import List, Tuple, Dict, Callable, Set
def direct_assign(starts: List[Tuple[int, int]], goals: List[Tuple[int, int]]):
    # The number of start positions must be equal to the number of goal positions
    assert(len(starts) == len(goals))
    agents = []
    for i in range(len(starts)):
        agents.append(Agent(starts[i], goals[i]))
    
    return agents

class Simulator:

    def __init__(self):
        # Set up a white 1080p canvas
        self.canvas = np.ones((550,550,3), np.uint8)*255 
        # Draw the rectangluar obstacles on canvas
        
        GRID_SIZE = 50
        ROBOT_RADIUS = 25
        
        obstacle = []
        for x in range(0,550+GRID_SIZE,GRID_SIZE):
            obstacle.append((x,0))
            obstacle.append((x,(550//GRID_SIZE+1)*(GRID_SIZE)))
        for y in range(0,550+GRID_SIZE,GRID_SIZE):
            obstacle.append((0,y))
            obstacle.append(((550//GRID_SIZE+1)*(GRID_SIZE),y))
        
        # Call cbs-mapf to plan
        obstacle.extend([(75, 125), (125, 125), (225, 125), (275, 125), (325, 125), (375, 125), (425, 125), (475, 125), (75, 175), (475, 175), (75, 225), (175, 225), (225, 225), (325, 225), (375, 225), (475, 225), (75, 275), (175, 275), (375, 275), (475, 275), (75, 325), (175, 325), (225, 325), (325, 325), (375, 325), (475, 325), (75, 375), (475, 375), (75, 425), (125, 425), (175, 425), (225, 425), (275, 425), (325, 425), (425, 425), (475, 425)])

        # Call cbs-mapf to plan
        

        print(obstacle)
        self.planner = Planner(GRID_SIZE, ROBOT_RADIUS, obstacle)
        before = time.time()
        START = [(50,50),(550-50,550-50)]
        GOAL = [(550-50,550-50),(50,50)]
        self.path = self.planner.plan(START, GOAL, assign=direct_assign, debug=True)
        after = time.time()
        print('Time elapsed:', "{:.4f}".format(after-before), 'second(s)')

        # Assign each agent a colour
        self.colours = self.assign_colour(len(self.path))

        # Put the path into dictionaries for easier access
        d = dict()
        for i, path in enumerate(self.path):
            self.draw_path(self.canvas, path, i)  # Draw the path on canvas
            d[i] = path
        self.path = d

    '''
    Transform opposite vertices of rectangular obstacles into obstacles
    '''
    @staticmethod
    def vertices_to_obsts(obsts):
        def drawRect(v0, v1):
            o = []
            base = abs(v0[0] - v1[0])
            side = abs(v0[1] - v1[1])
            for xx in range(0, base, 30):
                o.append((v0[0] + xx, v0[1]))
                o.append((v0[0] + xx, v0[1] + side - 1))
            o.append((v0[0] + base, v0[1]))
            o.append((v0[0] + base, v0[1] + side - 1))
            for yy in range(0, side, 30):
                o.append((v0[0], v0[1] + yy))
                o.append((v0[0] + base - 1, v0[1] + yy))
            o.append((v0[0], v0[1] + side))
            o.append((v0[0] + base - 1, v0[1] + side))
            return o
        static_obstacles = []
        for vs in obsts.values():
            static_obstacles.extend(drawRect(vs[0], vs[1]))
        return static_obstacles

    '''
    Randomly generate colours
    '''
    @staticmethod
    def assign_colour(num):
        def colour(x):
            x = hash(str(x+42))
            return ((x & 0xFF, (x >> 8) & 0xFF, (x >> 16) & 0xFF))
        colours = dict()
        for i in range(num):
            colours[i] = colour(i)
        return colours

    def draw_rect(self, pts_arr: np.ndarray) -> None:
        for pts in pts_arr:
            cv2.rectangle(self.canvas, tuple(pts[0]), tuple(pts[1]), (0, 0, 255), thickness=3)

    def draw_path(self, frame, xys, i):
        for x, y in xys:
            cv2.circle(frame, (int(x), int(y)), 10, self.colours[i], -1)

    '''
    Press any key to start.
    Press 'q' to exit.
    '''
    def start(self):
        cv2.namedWindow('frame', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('frame', (500, 500))
        wait = True
        try:
            i = 0
            while True:
                frame = deepcopy(self.canvas)
                for id_ in self.path:
                    x, y = tuple(self.path[id_][i])
                    cv2.circle(frame, (x, y), ROBOT_RADIUS-5, self.colours[id_], 5)
                cv2.imshow('frame', frame)
                if wait:
                    cv2.waitKey(0)
                    wait = False
                k = cv2.waitKey(100) & 0xFF 
                if k == ord('q'):
                    break
                i += 1
        except Exception:
            cv2.waitKey(0)
            cv2.destroyAllWindows()

def load_scenario(fd):
    with open(fd, 'r') as f:
        global GRID_SIZE, ROBOT_RADIUS, RECT_OBSTACLES, START, GOAL
        data = yaml.load(f, Loader=yaml.FullLoader)
        GRID_SIZE = data['GRID_SIZE']
        ROBOT_RADIUS = data['ROBOT_RADIUS']
        RECT_OBSTACLES = data['RECT_OBSTACLES']
        START = data['START']
        GOAL = data['GOAL']

'''
Use this function to show your START/GOAL configurations
'''
def show_pos(pos):
    cv2.namedWindow('frame', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('frame', (500, 500))
    frame = np.ones((1080,1920,3), np.uint8)*255
    for x, y in pos:
        cv2.circle(frame, (x, y), ROBOT_RADIUS-5, (0, 0, 0), 5)
    cv2.imshow('frame', frame)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == '__main__':
    # From command line, call:
    # python3 visualizer.py scenario1.yaml
    load_scenario(sys.argv[1])
    # show_pos(START)
    r = Simulator()
    r.start()
