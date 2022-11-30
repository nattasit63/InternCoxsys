#!/usr/bin/env python3
import time
from typing import  List, Tuple
from cbs_mapf.planner import Planner
import cv2 ,sys
import numpy as np
import matplotlib.pyplot as plt
from cbs_mapf.agent import Agent
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from turtlee_interfaces.srv import Matcs
from std_srvs.srv import Empty
import multiprocessing as mp
from multiprocessing import Manager
'''
SIZE FOR TURTORIAL
# GRID_SIZE = 24
# ROBOT_RADIUS = 11
'''
GRID_SIZE = 12
ROBOT_RADIUS = 8




COLOR = [(255,0,0),(0,0,255),(0,255,0)]
RESULT = []

# MAP_PATH  ='/home/natta/interface_ws/src/full_interface/config/map_example0.png'

# SAMPLE =    [[[129, 164], [233, 159], [245, 69], [323, 101], [245, 69], [233, 159], [129, 164]], 
#              [[226, 427], [245, 384], [350, 421], [377, 314], [299, 242], [323, 320], [262, 316], [222, 294], [299, 242], [381, 163], [299, 242], [262, 316], [245, 384], [226, 427]]]

# trig_signal = False
# srv_id = 0




def insert_list(shared,original):
    try:
        for i in range(len(original)):
            shared[i] = original[i]
    except:
        pass

class Traffic_Management():
    
    def __init__(self):    
        self.grid_size = GRID_SIZE
        self.manager = Manager()
        self.start = self.manager.list()
        self.goal = self.manager.list()
        self.current_start = self.manager.list()
        self.current_goal =self.manager.list()
        self.obs_list = self.manager.list()
        self.obs_ind = self.manager.list()
        self.current_index =self.manager.list()
        self.alive_agent = self.manager.list()
        self.agent_max_index = self.manager.list()
        self.fleet = self.manager.list()
        self.id = self.manager.list()
        self.del_id = self.manager.list()
        self.current_all_pos = self.manager.list()
        # self.shared_list = self.manager.list(self.current_all_pos)


    def get_plan_data_list(self,x,id):
        start = []
        goal = [] 
        for i in range(len(x)):
            try:
                start.append(x[i][id])
            except:
                start.append(x[i][-1])
            try:
                goal.append(x[i][id+1])
            except:
                goal.append(x[i][-1])
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
        for point in fleet:       
            self.current_all_pos.append(point[0])
        self.mg_cap = self.manager.list(self.current_all_pos)
        self.mg_cur_ind = self.manager.list(self.current_index)
        self.mg_alive_agent = self.manager.list(self.alive_agent)
        self.mg_id = self.manager.list(self.id)
        self.mg_fleet =self.manager.list(self.fleet) 
        self.shared_list = self.manager.list([[]]*len(fleet))
        # print(f'len fleet:{self.shared_list}')
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
        state = 0
        self.current_all_pos = current_all_pos
        # print(f'cur_ID:{self.current_index}')
        try:
            get_pos = mp.Process(target=insert_list,args=(self.mg_cap,current_all_pos,))     
            get_cur_pos = mp.Process(target=insert_list,args=(self.mg_cur_ind,self.current_index,))
            get_id = mp.Process(target=insert_list,args=(self.mg_id,self.id,))
            get_pos.start()
            get_cur_pos.start()
            get_id.start()
            time.sleep(0.3)
        except:
            pass
        def to_int(x):
            ans = []
            for i in x:
                ans.append([int(i[0]),int(i[1])])
            return ans
        if Trigger==None:
            start,goal = self.get_plan_data_list(self.fleet,0)
            return self.planning(start,goal)
        if Trigger:       
            if state == 0:
                try:
                    id = self.mg_alive_agent.index(arrive_id)
                    # id = self.alive_agent.index(arrive_id)
                    state = 1
                except:
                    print('id not found')
                    state = 4

            if state == 1 :
                self.current_index[id]+=1
                # print(f'cur_ID trigger:{self.current_index}')
                get_cur_pos = mp.Process(target=insert_list,args=(self.mg_cur_ind,self.current_index,))
                get_cur_pos.start()
                
                time.sleep(0.3)
                # print(f'mg cur_ID:{self.mg_cur_ind}')
                if self.current_index[id]<=self.agent_max_index[arrive_id]-2:
                   state = 2
                else: 
                    del self.fleet[id]
                    del self.current_index[id]
                    del self.alive_agent[id]
                    del self.mg_alive_agent[id]
                    del self.mg_cur_ind[id]
                    del self.mg_fleet[id]
                    self.del_id.append(id)
                    
                   

                    state = 2
            if state==2:

                real_start,real_goal = [],[]
                # q,new_goal = self.get_plan_data_list(self.mg_fleet,self.current_index[id])
                try:
                    q,new_goal = self.get_plan_data_list(self.mg_fleet,self.mg_cur_ind[id])
                    self.current_goal[id] = to_int(new_goal)[id]
                    start = to_int(current_all_pos)
                    # for i in range(len(self.alive_agent)):
                    #     real_start.append(start[self.alive_agent[i]])
                    #     real_goal.append(self.current_goal[i])
                    for i in range(len(self.mg_alive_agent)):
                        real_start.append(start[self.mg_alive_agent[i]])
                        real_goal.append(self.current_goal[i])
                    Trigger = False
                    return self.mg_alive_agent,self.planning(real_start, real_goal)
                except:
                    state = 5
            
            if state == 4:
                agent = []  #use for return
                start_live,goal_live  = [],[]
                # for i in range(len(self.alive_agent)):
                #     start_live.append([])
                #     goal_live.append([])
                # for live in self.alive_agent:
                #     id_live = self.alive_agent.index(live)
                #     self.current_index[id_live]+=1
                #     agent.append(live)
                #     start_live[live-1] = to_int(current_all_pos)[live]
                #     q,new_goal_live = self.get_plan_data_list(self.fleet,self.current_index[id_live])
                #     goal_live[live-1] = to_int(new_goal_live)[id_live]
                # path_live = self.planning(start_live,goal_live)
                # print(f'state:{state}')
                # print(f'mg_alive_agent:{self.alive_agent}')
                for i in range(len(self.mg_alive_agent)):
                    start_live.append([])
                    goal_live.append([])
                for live in self.mg_alive_agent:
                    id_live = self.mg_alive_agent.index(live)
                    self.mg_cur_ind[id_live]+=1
                    agent.append(live)
                    start_live[live-1] = to_int(current_all_pos)[live]
                    # q,new_goal_live = self.get_plan_data_list(self.fleet,self.mg_cur_ind[id_live])
                    q,new_goal_live = self.get_plan_data_list(self.mg_fleet,self.mg_cur_ind[id_live])
                    goal_live[live-1] = to_int(new_goal_live)[id_live]
                path_live = self.planning(start_live,goal_live)
                return agent,path_live

            if state == 5:
                print('-'*50)
                print('\n')
                print('Traffic Done')
                print('\n')
                print('-'*50)
                return True,True

    def get_obstacle_ind(self,name):
        self.map_img = name 
        originalImage = cv2.imread(name)
        imS = cv2.resize(originalImage, (800, 800))
        grayImage = cv2.cvtColor(imS, cv2.COLOR_BGR2GRAY)      
        (thresh, blackAndWhiteImage) = cv2.threshold(grayImage, 127, 255, cv2.THRESH_BINARY)
        print('Converting ......')
        self.img_copy = imS.copy()
        for w in range (1,blackAndWhiteImage.shape[0]):
            for h in range (1,blackAndWhiteImage.shape[1]):
                pixel = blackAndWhiteImage[w,h]
                if pixel>=10:
                    self.img_copy[w][h]=(255,255,255)                 
                else:
                    self.obs_ind.append((h,w))
                    self.img_copy[w][h]=(0,0,0)
        print('Converted to Black-White image')
        # print('ok')
        # for i in self.obs_ind:
        #     # print(i)
        #     if i ==(100, 533) or i==(475, 403) or i==(13, 369) or i== (470, 26)or i ==(503,772):
        #         print('YESSSSS')
        return self.obs_ind

 
    def full_plan(self,name,fleet):
        obs_ind = self.get_obstacle_ind(name)

        all_start_list,all_goal_list = self.prepare_data(fleet)
        start_initial_time = time.time()
        planner = Planner(grid_size=GRID_SIZE,robot_radius=ROBOT_RADIUS,static_obstacles=obs_ind)  
        final_initial_time = time.time()   
        use_time =   final_initial_time - start_initial_time
        print(f'Initial libary time : {use_time} seconds')
        for i in range(len(all_start_list)):
            print(f'Start : {all_start_list[i]} , Goal : {all_goal_list[i]}')
            path = planner.plan(starts= all_start_list[i],goals=all_goal_list[i],debug=False,assign=direct_assigner)
            try:
                a = np.array(path)
                a=a.tolist()
                if a==[]:
                    print(f'*****INFEASIBLE  at  Start:{all_start_list[i]},Goal:{all_goal_list[i]}*************')
            except:
                pass
                
            RESULT.append(path)
            # for k in path:
            #     RESULT.append(path)

            for z in range(len(path)):
                for j in range(len(path[z])):         
                    x_pos = path[z][j][1]
                    y_pos = path[z][j][0]
                    self.img_copy[x_pos][y_pos] = COLOR[z%3]
            # cv2.imshow("Image Copy", self.img_copy)       
            # cv2.waitKey(0)
            # cv2.destroyAllWindows()

        return RESULT

        

 
    def get_server_service(self,trigger,id):
        print('-'*50)
        print('\n')
        
        print(f'Service TRIGGER : {trigger} | Id : {id}')
        # print(f'cap : {self.mg_cap}')
        # print(f'index serv: {self.mg_cur_ind}')

        self.optimal_plan(Trigger=trigger,arrive_id=id,current_all_pos=self.mg_cap)
        print('\n')
        print('-'*50)
        return 
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



class Traffic_Service_Server(Node):
    def __init__(self,Traffic):
        super().__init__('traffic_service_server')
        self.traffic = Traffic
        self.position_trigger = self.create_service(Matcs,'/matc_trigger_service',self.set_trigger_callback) 
        # print(self.traffic.pp)
    def set_trigger_callback(self,request,response):
        self.traffic.get_server_service( request.trigger,request.id)
        return response







'''
Service Trigger Example
'''
# def main(args=None):
#     rclpy.init(args=args)
#     mm = Traffic_Management()
#     mm.initial(map_path=MAP_PATH,fleet=PATH)
#     mm.optimal_plan()
#     a,b = mm.optimal_plan(Trigger=True,arrive_id=1,current_all_pos=[[160, 125], [711, 222], [449, 614], [657, 652]])
#     '''
#     calling service will trigger and return the next station of arrive id: 

#         ros2 service call /matc_trigger_service turtlee_interfaces/srv/Matcs '{trigger: True,id: 0}'
#     '''
#     # if a == True:
#     #     sys.exit()
#     traffic_srv = Traffic_Service_Server(mm)
#     rclpy.spin(traffic_srv)
#     traffic_srv.destroy_node()
#     rclpy.shutdown()
# if __name__=='__main__':

#     PATH = [[[131, 193], [164, 94], [324, 84], [325, 150], [324, 84], [164, 94], [131, 193]],
#     [[715, 275], [709, 228], [535, 278], [534, 405], [586, 577], [446, 585], [259, 716], [257, 592], [449, 292], [333, 239], [499, 144], [700, 150], [709, 228], [715, 275]], 
#     [[452, 697], [446, 585], [586, 577], [534, 405], [594, 406], [534, 405], [586, 577], [586, 633], [603, 740], [586, 633], [586, 577], [534, 405], [594, 406], [763, 407], [594, 406], [534, 405], [586, 577], [446, 585], [452, 697]]
#     ,[[669, 729], [656, 592], [452, 587], [378, 508], [160, 522], [240, 687], [322, 574], [160, 522], [378, 508], [452, 587], [656, 592], [669, 729]]]
    
#     essential_pos =[[[60, 449], [669, 729]], [[242, 415], [322, 574], [240, 687], [713, 490], [601, 519]]]
#     MAP_PATH ='/home/natta/interface_ws/src/full_interface/config/map_example0.png'
#     main()



'''
Full Planning tutorial
'''
if __name__=='__main__':

    #impossible path
    # PATH = [[[222, 618], [238, 529], [102, 532], [13, 369]], 
    # [[675, 133], [642, 195], [553, 112], [470, 26]]]

    # PATH = [[[131, 193], [164, 94], [324, 84],[164, 94], [131, 193]],[[715, 275], [709, 228], [535, 278], [534, 405], [586, 577], [446, 585]], 
    # [[452, 697], [446, 585], [586, 577],  [594, 406], [534, 405], [586, 577]]]
    
    # PATH = [[[20,379],[766,380]],[[406,438],[45,379]],[[785,384],[403,421]]]
    PATH = [[[45,342],[730,370]],[[750,330],[410,450]],[[405,410],[50,355]]]
    essential_pos =[[[60, 449], [669, 729]], [[242, 415], [322, 574], [240, 687], [713, 490], [601, 519]]]
    MAP_PATH ='/home/natta/interface_ws/src/full_interface/config/tutorial0.png'
    traffic = Traffic_Management()
    a = traffic.full_plan(MAP_PATH,PATH)
    print(f'Answer : {a}')
