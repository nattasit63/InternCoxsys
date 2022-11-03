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

GRID_SIZE = 16
ROBOT_RADIUS = 4
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
        self.current_start = []
        self.current_goal =[]
        self.obs_list = []
        self.obs_ind = []
        self.index = 0
        self.current_index =[]
        self.alive_agent = []
        self.agent_max_index = []
        self.fleet = []
        self.id = []
        self.del_id = []
        
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
        self.fleet = int_path(fleet)
        for i in range(len(fleet)):
            self.current_index.append(0)
            self.id.append(i)
            self.alive_agent.append(i)
        
        for path in fleet:
                self.agent_max_index.append(len(path))
           
    def planning(self,start,goal):
        print(f'Planning at start:{start} , goal: {goal}')
        self.current_start= start.copy()
        self.current_goal = goal.copy()
        path = self.planner.plan(starts= start,goals=goal,debug=False,assign=direct_assigner)
        return path
    
    def tracking(self):
        return self.current_start,self.current_goal
    
    def optimal_plan(self,Trigger=None,arrive_id=None,current_all_pos=None):
        # print('*'*50)
        # print(f'alive agent : {self.alive_agent}')
        # print(f'arrive_id   : {arrive_id} {self.current_index}')
        state = 0
        def to_int(x):
            ans = []
            for i in x:
                ans.append([int(i[0]),int(i[1])])
            return ans

        if Trigger==None:
            # print(self.fleet)
            start,goal = self.get_plan_data_list(self.fleet,0)
            return self.planning(start,goal)
        if Trigger:
            # id = self.alive_agent.index(arrive_id)       
            # self.current_index[id]+=1
            # if self.current_index[id]<=self.agent_max_index[id]-2:
            #     q,new_goal = self.get_plan_data_list(self.fleet,self.current_index[id])
            #     self.current_goal[arrive_id] = to_int(new_goal)[id]
            #     start = to_int(current_all_pos)
            #     Trigger = False
            #     return self.alive_agent,self.planning(start,self.current_goal)
            # else:
            #     del self.fleet[id]
            #     del self.current_index[id]
            #     del self.alive_agent[id]
            if state == 0:
                print(f'state : {state}')
                try:
                    id = self.alive_agent.index(arrive_id)
                    state = 1
                except:
                    print('id not found')
                    # print(f'alive agent:{self.alive_agent}')
                    state = 4
            if state == 4:
                print(f'state : {state}')
                agent = []  #use for return
                start_live,goal_live  = [],[]
                for i in range(len(self.alive_agent)):
                    start_live.append([])
                    goal_live.append([])
                for live in self.alive_agent:
                    id_live = self.alive_agent.index(live)
                    self.current_index[id_live]+=1
                    agent.append(live)
                    # print(f'index in alive_agent : {id}')
                    start_live[live-1] = to_int(current_all_pos)[live]
                    q,new_goal_live = self.get_plan_data_list(self.fleet,self.current_index[id_live])
                    goal_live[live-1] = to_int(new_goal_live)[id_live]
                    # print(goal_live)
                path_live = self.planning(start_live,goal_live)
                # print(path_live)
                return agent,path_live

            if state == 1 :
                print(f'state : {state}')
                self.current_index[id]+=1
                if self.current_index[id]<=self.agent_max_index[arrive_id]-2:
                   state = 2
                else:
                    print(f'fleet : {self.fleet}')
                    print(f'currendIndex :{self.current_index} | ID:{id}')
                    del self.fleet[id]
                    del self.current_index[id]
                    del self.alive_agent[id]
                    self.del_id.append(id)
                    if self.fleet==[]:
                        return 'Complete'
                    state = 2
            if state==2:
                real_start,real_goal = [],[]
                print(f'state : {state}')
                
                q,new_goal = self.get_plan_data_list(self.fleet,self.current_index[id])
                self.current_goal[id] = to_int(new_goal)[id]
                start = to_int(current_all_pos)
                
                for i in range(len(self.alive_agent)):
                    real_start.append(start[self.alive_agent[i]])
                    real_goal.append(self.current_goal[i])
                # print(f'real_goal :{real_goal}')
                Trigger = False
                # return self.alive_agent,self.planning(start, self.current_goal)
                return self.alive_agent,self.planning(real_start, real_goal)
            # if state == 3 :
            #     print(f'state : {state}')
            #     del self.fleet[id]
            #     del self.current_index[id]
            #     del self.alive_agent[id]
            #     self.del_id.append(id)
            #     state = 2

    
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
#     traffic.initial(map_path=MAP_PATH,fleet=PATH)
#     traffic.optimal_plan()
#     traffic.optimal_plan(Trigger=True,arrive_id=0,current_all_pos=[[59.0, 216.0],[325.0, 287.0]])
#     traffic.optimal_plan(Trigger=True,arrive_id=0,current_all_pos=[[59.0, 216.0],[325.0, 287.0]])
#     traffic.optimal_plan(Trigger=True,arrive_id=0,current_all_pos=[[59.0, 216.0],[325.0, 287.0]])
#     traffic.optimal_plan(Trigger=True,arrive_id=1,current_all_pos=[[59.0, 216.0],[325.0, 287.0]])
#     traffic.optimal_plan(Trigger=True,arrive_id=0,current_all_pos=[[59.0, 216.0],[325.0, 287.0]])
#     traffic.optimal_plan(Trigger=True,arrive_id=0,current_all_pos=[[59.0, 216.0],[325.0, 287.0]])
#     traffic.optimal_plan(Trigger=True,arrive_id=0,current_all_pos=[[59.0, 216.0],[325.0, 287.0]])
#     traffic.optimal_plan(Trigger=True,arrive_id=0,current_all_pos=[[59.0, 216.0],[325.0, 287.0]])
#     print('*'*50)
#     traffic.optimal_plan(Trigger=True,arrive_id=1,current_all_pos=[[59.0, 216.0],[325.0, 287.0]])
#     traffic.optimal_plan(Trigger=True,arrive_id=1,current_all_pos=[[59.0, 216.0],[325.0, 287.0]])
#     traffic.optimal_plan(Trigger=True,arrive_id=1,current_all_pos=[[59.0, 216.0],[325.0, 287.0]])
#     traffic.optimal_plan(Trigger=True,arrive_id=1,current_all_pos=[[59.0, 216.0],[325.0, 287.0]])
#     traffic.optimal_plan(Trigger=True,arrive_id=1,current_all_pos=[[59.0, 216.0],[325.0, 287.0]])
#     traffic.optimal_plan(Trigger=True,arrive_id=1,current_all_pos=[[59.0, 216.0],[325.0, 287.0]])
#     traffic.optimal_plan(Trigger=True,arrive_id=1,current_all_pos=[[59.0, 216.0],[325.0, 287.0]])
#     traffic.optimal_plan(Trigger=True,arrive_id=1,current_all_pos=[[59.0, 216.0],[325.0, 287.0]])
#     traffic.optimal_plan(Trigger=True,arrive_id=1,current_all_pos=[[59.0, 216.0],[325.0, 287.0]])
#     traffic.optimal_plan(Trigger=True,arrive_id=1,current_all_pos=[[59.0, 216.0],[325.0, 287.0]])
#     traffic.optimal_plan(Trigger=True,arrive_id=1,current_all_pos=[[59.0, 216.0],[325.0, 287.0]])
#     traffic.optimal_plan(Trigger=True,arrive_id=1,current_all_pos=[[59.0, 216.0],[325.0, 287.0]])
#     traffic.optimal_plan(Trigger=True,arrive_id=1,current_all_pos=[[59.0, 216.0],[325.0, 287.0]])
#     traffic.optimal_plan(Trigger=True,arrive_id=1,current_all_pos=[[59.0, 216.0],[325.0, 287.0]])
#     traffic.optimal_plan(Trigger=True,arrive_id=1,current_all_pos=[[59.0, 216.0],[325.0, 287.0]])
#     traffic.optimal_plan(Trigger=True,arrive_id=1,current_all_pos=[[59.0, 216.0],[325.0, 287.0]])
#     traffic.optimal_plan(Trigger=True,arrive_id=1,current_all_pos=[[59.0, 216.0],[325.0, 287.0]])
#     traffic.optimal_plan(Trigger=True,arrive_id=1,current_all_pos=[[59.0, 216.0],[325.0, 287.0]])

    



# if __name__=='__main__':
#     PATH = [[[59.0, 216.0], [143.0, 211.0], [135.0, 94.0], [251.0, 92.0], [135.0, 94.0], [143.0, 211.0], [59.0, 216.0]], 
#     [[440.0, 700.0], [436.0, 593.0], [451.0, 290.0], [325.0, 287.0], [327.0, 208.0], [325.0, 287.0], [451.0, 290.0], [436.0, 593.0], [549.0, 599.0], [703.0, 708.0]
#     , [559.0, 478.0], [549.0, 599.0], [436.0, 593.0], [451.0, 290.0], [325.0, 287.0], [249.0, 290.0], [325.0, 287.0], [451.0, 290.0], [436.0, 593.0], [440.0, 700.0]]]

#     essential_pos = [[[59, 216], [440, 700]], [[251, 92], [327, 208], [249, 290], [559, 478], [703, 708]]]
#     MAP_PATH = '/home/natta/interface_ws/src/full_interface/config/map_example0.png'
        
#     traffic = Traffic_Management()

#     main()
