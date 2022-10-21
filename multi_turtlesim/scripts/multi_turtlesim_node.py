#!/usr/bin/python3
from cmath import sqrt
import time
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import os 
import random
from std_msgs.msg import UInt8MultiArray
from geometry_msgs.msg import Twist, Point
from turtlesim.msg import Pose
from std_srvs.srv import Empty
from multi_turtlesim_interfaces.srv import Pick, SpawnParcel, GetTurtleIDs, GetPosition
from functools import partial
import yaml
import pygame
import numpy as np
import math,yaml
from multi_turtlesim.traffic import Traffic_Management
import multi_turtlesim.traffic as mt
import multiprocessing as mp
from turtlee_interfaces.srv import Setgoal
from cbs_mapf.planner import Planner
# import multi_turtlesim.controller as controller


class Parcel():
    def __init__(self,id,position:Point,destination):
        self.id = id
        self.position = position
        self.isPicked = False
        self.destination = destination
class Turtle():
    def __init__(self,id,pose:Pose,image):
        self.id = id
        self.pose = Pose()
        self.pose.x = pose.x
        self.pose.y = pose.y
        self.pose.theta = pose.theta
        self.image = image
        self.cmd = Twist()
        self.counter = 0
        self.parcel = None
        
    def update(self,cmd,dt):
        dtheta = cmd.angular.z*dt
        R_w = self.theta_to_R(dtheta)
        q = self.theta_to_cmplx(self.pose.theta)
        q_new = np.matmul(R_w,q)
        self.pose.theta = math.atan2(q_new[1],q_new[0])
        S = np.array([[0,1],[-1,0]]).T
        q_theta = self.theta_to_cmplx(dtheta)
        if abs(dtheta)<0.000000001:
            V = np.eye(2)
        else:
            s = math.sin(dtheta)
            c = math.cos(dtheta)
            
            V = np.array([[s/dtheta,(1-c)/dtheta],[(c-1)/dtheta,s/dtheta]]).T
            #np.concatenate([np.array([[0,1]]).T-np.matmul(S,q_theta),q_theta-np.array([[1,0]]).T],1)/(cmd.angular.z*dt)
        v = np.array([[cmd.linear.x,0.0]]).T
        p = np.matmul(self.theta_to_R(self.pose.theta),np.matmul(V,v))
        self.pose.x = self.pose.x+float(p[0])
        self.pose.y = self.pose.y+float(p[1])
    def theta_to_cmplx(self,theta):
        c = math.cos(theta)
        s = math.sin(theta)
        return np.array([[c,s]]).T
    def theta_to_R(self,theta):
        c = math.cos(theta)
        s = math.sin(theta)
        return np.array([[c,s],[-s,c]]).T
    def set_pose(self,pose):
        self.pose = pose
    def add_publisher(self,pub):
        self.publisher = pub
    def add_subscriber(self,sub):
        self.subscriber = sub
    def cmd_callback(self,msg:Twist):
        self.counter = 0
        self.cmd = msg
    def pick_parcel(self,request,response,parcels):
        if not request.id in parcels.keys():
            response.success = False 
            print(f'No parcel of the id {request.id} is available.')
            return response
        parcel = parcels[request.id]
        parcel_position = np.array([parcel.position.x,parcel.position.y])
        if parcel.isPicked:
            response.success = False 
            print(f'Parcel of the id {request.id} is being transported by another turtle.')
            return response
        pickup_radius = 2
        if np.linalg.norm(parcel_position-np.array([self.pose.x,self.pose.y]))<=pickup_radius:
            response.success = True
            parcel.position.x = self.pose.x
            parcel.position.y = self.pose.y
            parcel.isPicked = True
            self.parcel = parcel
            
            return response
        else:
            response.success = False 
            print(f'Parcel of the id {request.id} is too far.')
            return response

    def place_parcel(self,request,response,parcels):
        
        R = self.theta_to_R(self.pose.theta)
        p = np.matmul(R,np.array([[0, 0.5]]).T)+np.array([[self.pose.x,self.pose.y]]).T
        self.parcel.position.x = float(p[0])
        self.parcel.position.y = float(p[1])
        self.parcel.isPicked = False
        self.parcel = None
        return response
class GUI():
    def __init__(self):
        self.screen_width = 800
        self.screen_height = 800
        #[69,86,255]/255
        self.screen = pygame.display.set_mode([self.screen_width, self.screen_height])
        pygame.display.set_caption('Multi-Turtlesim')
        pygame.init()
        self.isQuit = False
        multi_turtlesim_path = get_package_share_directory('multi_turtlesim')
        
        map_path = os.path.join(multi_turtlesim_path,'config','map.yaml')
        with open(map_path) as f:
            self.map_parameters = yaml.load(f, Loader=yaml.loader.SafeLoader)
        
        self.map_image = pygame.image.load(os.path.join(multi_turtlesim_path,'images','map',self.map_parameters['image'])).convert()
        # self.map_image = pygame.image.load(MAP_PATH).convert()
        self.map_image = pygame.transform.scale(self.map_image,(self.screen_width, self.screen_height))
        rect = self.map_image.get_rect()
        rect = rect.move((0,0))
        self.obstacle  = mt.get_obstacle_ind(name='/home/natta/traffic_manager/src/multi_turtlesim/images/map/map_example0.png')
        # self.obstacle = Traffic_Management.get_obstacle_ind(name='/home/natta/traffic_manager/src/multi_turtlesim/images/map/map_example0.png')
        
        # print('Obstacle = ',self.obstacle)
        self.map_image.set_colorkey((255,255,255))
    
    def update(self,turtles,parcels):
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                self.isQuit = True
            
        # Fill the background with white
        self.screen.fill((69, 86, 255))
        cell_size = mt.GRID_SIZE
        # print(f'Grid size : {cell_size}')
        for i in range(self.screen_width//cell_size):
            pygame.draw.line(self.screen,(255,255,255),(0,cell_size*i),(self.screen_width,cell_size*i))
            pygame.draw.line(self.screen,(255,255,255),(cell_size*i,0),(cell_size*i,self.screen_width))
        multi_turtlesim_path = get_package_share_directory('multi_turtlesim')
        self.screen.blit(self.map_image,(0,0))
            
        img_parcel = pygame.image.load(os.path.join(multi_turtlesim_path,'images','parcel','parcel.png'))
        img_parcel = pygame.transform.scale(img_parcel, (cell_size,cell_size)).convert_alpha()        
        
        for id,turtle in turtles.items():
            img = pygame.image.load(os.path.join(get_package_share_directory('multi_turtlesim'),'images','turtle',turtle.image))
            img = pygame.transform.rotate(img, turtle.pose.theta*180/math.pi-90)
            size = np.min([math.floor(img.get_size()[0]/2),math.floor(img.get_size()[1]/2)])  
            
            self.screen.blit(img,(turtle.pose.x/5.0*250-size,self.screen_width-turtle.pose.y/5.0*250-size))
            
            if turtle.parcel:
                img = pygame.transform.rotate(img_parcel, turtle.pose.theta*180/math.pi)
                
                self.screen.blit(img,(turtle.pose.x/5.0*250-size-math.floor(img.get_size()[0]/2),self.screen_width-turtle.pose.y/5.0*250-size-math.floor(img.get_size()[1]))) 
        for id,parcel in parcels.items():
            if not parcel.isPicked:
                # print(f'Parcel is placed at {parcel.position.x},{parcel.position.y}')
                self.customer_circle(parcel.position.x/16.0*800,parcel.position.y/16*800,5)
                # self.screen.blit(img_parcel,(parcel.position.x/5.0*250,self.screen_width-parcel.position.y/5.0*250))
                # pygame.draw.rect(self.screen,(255,0,0),pygame.Rect(parcel.position.x/5.0*250,self.screen_width-parcel.position.y/5.44445*250,cell_size,cell_size))

        pygame.display.flip()
    
    def customer_circle(self,pos_x,pos_y,radius):
        pygame.draw.circle(self.screen, (255,0,0),[pos_x,pos_y],radius)
class MultiTurtleSim(Node):
    def __init__(self,gui:GUI):
        super().__init__('multi_turtle_sim')
        self.turtles = {}
        multi_turtlesim_path = get_package_share_directory('multi_turtlesim')
        
        # load and keep track of used images 
        self.images  = os.listdir(os.path.join(multi_turtlesim_path,'images','turtle'))
        self._available_turtle_ids = range(1,len(self.images)+1)   
        # initialize parcels
        self.parcels = {}
        self.parcel_count = 0
        # establish timer
        self.timer_period = 0.1
        self.id_publisher = self.create_publisher(UInt8MultiArray,'/turtle_IDs',10)
        self.timer = self.create_timer(self.timer_period,self.timer_callback)
        self.spawn_server = self.create_service(SpawnParcel,'/spawn_parcel',self.spawn_parcel)
        self.get_IDS_server = self.create_service(GetTurtleIDs,'/get_turtle_ids',self.get_turtle_ids_callback)
        self.get_position = self.create_service(GetPosition,'/get_position',self.get_position_callback)
        self.gui = gui
    def get_turtle_IDs(self):
        IDs = []
        for id,turtle in self.turtles.items():
            IDs.append(id)
        return IDs
    def get_position_callback(self,request,response):
        IDs = self.get_turtle_IDs()
        if request.id in IDs:
            response.flag = True
            response.position = Point()
            response.position.x = self.turtles[request.id].pose.x
            response.position.y = self.turtles[request.id].pose.y
        else:
            response.position = Point()
            response.flag = False
        return response
    def get_turtle_ids_callback(self,request,response):
        response.ids = self.get_turtle_IDs()
        return response
    def timer_callback(self):
        robot_id = []
        for id,turtle in self.turtles.items():
            robot_id.append(int(id))
            turtle.counter = turtle.counter+self.timer_period
            if turtle.counter<0.5:
                turtle.update(turtle.cmd,self.timer_period)
            else:
                turtle.update(Twist(),self.timer_period)
            turtle.publisher.publish(turtle.pose)
        id_msg = UInt8MultiArray()
        id_msg.data = robot_id
        self.id_publisher.publish(id_msg)
        if self.gui.isQuit:
            self.destroy_node()
        else:
            self.gui.update(self.turtles,self.parcels)
    def spawn_turtle(self,pose):
        if self.images:
            idx = random.randrange(len(self.images))
            id = self._available_turtle_ids[0]
            self._available_turtle_ids = sorted(self._available_turtle_ids[1:])
            turtle = Turtle(id,pose,self.images.pop(idx))            
            turtle.add_publisher(self.create_publisher(Pose,'turtle'+str(id)+'/pose',10))
            turtle.add_subscriber(self.create_subscription(Twist,'turtle'+str(id)+'/cmd_vel',turtle.cmd_callback,10))
            self.create_service(Pick,'turtle'+str(id)+'/pick_parcel',partial(turtle.pick_parcel,parcels=self.parcels))
            self.create_service(Empty,'turtle'+str(id)+'/place_parcel',partial(turtle.place_parcel,parcels=self.parcels))
            self.turtles[id] = turtle
            return id
        else:
            self.get_logger().warning('No new turtle has spawned. Maximum number of turtless has reached.')
            return -1    
    def kill_turtle(self,id):
        if id in self._available_turtle_ids:
           self.get_logger().warning('No turtle with id: {id}')
        else:
            self.destroy_publisher(self.turtles[id].publisher)
            self.destroy_subscription(self.turtles[id].subscriber)
            
            self.turtles.pop(id)
            self._available_turtle_ids = sorted(self._available_turtle_ids+[id]) 
    def spawn_parcel(self,request,response):
        self.parcel_count = self.parcel_count+1
        self.parcels[self.parcel_count] = Parcel(self.parcel_count,request.position,request.destination)
        return response
class Controller(Node):
    def __init__(self,name,start_position,que):
        super().__init__(str(name)+'_controller')
        print(f'Node : {name}_controller spined !')
        self.agent_name = name
        self.publisher = self.create_publisher(Twist,str(name)+'/cmd_vel',10)
        # self.command_publisher = self.create_publisher(Twist,str(name)+'/cmd_vel',10)
        self.pose_subscription = self.create_subscription(Pose,'/'+str(name)+'/pose',self.pose_callback,10)
        self.set_goal_service = self.create_service(Setgoal,'/'+str(name)+'_set_goal',self.set_goal_callback)
        self.goal = start_position
        self.start_point = start_position
        self.current_postion = []
        self.first = True
        self.is_arrive = 0
        self.pose = Pose()
        timer_period = 0.1
        self.timer = self.create_timer(timer_period,self.timer_callback)
    def timer_callback(self):
        msg = self.control()     
        self.publisher.publish(msg)
        id_agent = (int(str(self.agent_name[-1])))
        idx = (id_agent-1)*3
        result[idx]=id_agent
        result[idx+1]=self.current_postion[0]
        result[idx+2]=self.current_postion[1]
        if self.check_reach_goal() and self.is_arrive==0:
            self.is_arrive =1
            print(str(self.agent_name),' Arrive')
            # qq.put(str(self.agent_name))
            id_agent = (int(str(self.agent_name[-1])))
            # print(f'id agent : {id_agent}')
            idx = (id_agent-1)*3
            # print('current pose :',current_pose)
            result[-1] = id_agent
            result[idx]=id_agent
            result[idx+1]=self.current_postion[0]
            result[idx+2]=self.current_postion[1]
            return True
            

    def check_reach_goal(self):
        current_position = np.array([self.pose.x,self.pose.y])
        if not self.first:
            dp = self.goal-current_position
            if np.linalg.norm(dp)<0.2:
                return True

    def pose_callback(self,msg):
        self.pose = msg
    def set_goal_callback(self,request,response):
        self.first=False
        self.is_arrive=0
        self.goal = np.array([request.x,request.y])
        return response
    
    def control(self):
        msg = Twist()
        current_position = np.array([self.pose.x,self.pose.y])
        self.current_postion = current_position
        dp = self.goal-current_position
        e = np.arctan2(dp[1],dp[0])-self.pose.theta
        K = 4.0
        w = K*np.arctan2(np.sin(e),np.cos(e))
        if np.linalg.norm(dp)>0.15:
            v = 0.2
        else:
            v = 0.0
            w = 0.0
        msg.linear.x = v
        msg.angular.z = w
        return msg


# PATH = [[[59.0, 216.0],[251.0, 92.0], [135.0, 94.0], [143.0, 211.0], [59.0, 216.0]], 
# [[440.0, 700.0], [436.0, 593.0], [451.0, 290.0], [325.0, 287.0], [327.0, 208.0], [325.0, 287.0], [451.0, 290.0], [436.0, 593.0], [549.0, 599.0], [703.0, 708.0], [559.0, 478.0], [549.0, 599.0], [436.0, 593.0], [451.0, 290.0], [325.0, 287.0], [249.0, 290.0], [325.0, 287.0], [451.0, 290.0], [436.0, 593.0], [440.0, 700.0]]]

PATH = [[[59.0, 216.0], [143.0, 211.0], [135.0, 94.0], [251.0, 92.0], [135.0, 94.0], [143.0, 211.0], [59.0, 216.0]], 
[[440.0, 700.0], [436.0, 593.0], [451.0, 290.0], [325.0, 287.0], [327.0, 208.0], [325.0, 287.0], [451.0, 290.0], [436.0, 593.0], [549.0, 599.0], [703.0, 708.0], [559.0, 478.0], [549.0, 599.0], [436.0, 593.0], [451.0, 290.0], [325.0, 287.0], [249.0, 290.0], [325.0, 287.0], [451.0, 290.0], [436.0, 593.0], [440.0, 700.0]]]
essential_pos = [[[59, 216], [440, 700]], [[251, 92], [327, 208], [249, 290], [559, 478], [703, 708]]]
MAP_PATH = '/home/natta/interface_ws/src/full_interface/config/map_example0.png'
    

def convert_to_TurtlesimScreen(fleet_pixel,w,h):
    ans = []
    head = []
    real_ans = []
    for i in range(len(fleet_pixel)):
        z=[]
        for j in range(len(fleet_pixel[i])):
            q=[]
            q.append(fleet_pixel[i][j][0]*800.00/w)
            q.append(fleet_pixel[i][j][1]*800.00/h)
            z.append(q)
        ans.append(z)
    for i in range(len(ans)):
        z=[]
        for j in range(len(ans[i])):
            q=[]       
            q.append(ans[i][j][0]*16.00/800.00)
            q.append(ans[i][j][1]*16.00/800.00)
            z.append(q)
            if j==0:
                head.append([ans[i][0][0]*16.00/800.00,16.00-ans[i][0][1]*16.00/800.00])
        real_ans.append(z)
    return real_ans,head


def traffic_to_sim(q):
    a =[]
    for i in range(len(q)):
        a.append([])
    for i in range(len(q)):
        for j in q[i]:
            a[i].append([j[0]*16.00/800.00,abs(16.00-(j[1]*16.00/800.00))])
    return a

def sub_path(path):
    path = np.array(path*1.00)
    path = path.tolist()
    sub_path = traffic_to_sim(path)
    return sub_path

def get_plan_data_list(x,id):
    start = []
    goal = []
    for i in range(len(x)):
        start.append(x[i][id])
        goal.append(x[i][id+1])
    # print('START :',start)
    # print('GOAL  :',goal)
    return start,goal
def service_spawn_parcel(cus):
        # print(f'input service : {cus}')
        def add_cmd(x,y):
            result = '"'+'position: '+'{'+'x: '+str(x)+','+'y: '+str(y)+',z: 0'+'}'+'"'
            return result
        num_agent = len(cus)
        for i in range(num_agent):
            remove_last = cus[i].copy()
            # remove_last = remove_last[:-1]
            for j in remove_last:
                # print(f'x,y : {j[0]},{j[1]}')
                custom_cmd = add_cmd(j[0],j[1])
                cmd = 'ros2 service call /spawn_parcel multi_turtlesim_interfaces/srv/SpawnParcel '+custom_cmd            
                os.popen(cmd).read()
                
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
                    
def spin_main_node(head):
    rclpy.init(args=None)
    gui = GUI()
    node = MultiTurtleSim(gui)
    for loc in head:
        # print(f'loc : {loc}')
        init_pose = Pose()
        init_pose.x = loc[0]
        init_pose.y = loc[1]
        node.spawn_turtle(init_pose)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

def controller_node_run(agent,start,q):
    rclpy.init(args=None)
    controller = Controller(name=agent,start_position=start,que=q)
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

def go_to_goal(agent,goal):
    def add_cmd(x,y):
        result = '"' +'{'+'x: '+str(x)+', '+'y: '+str(y)+'}'+'"'
        return result
    def agent_name(agent):
        result = "/" + str(agent)+"_set_goal "
        return result
    
    custom_cmd = add_cmd(goal[0],goal[1])
    agent_srv_cmd = agent_name(agent)
    cmd = 'ros2 service call '+agent_srv_cmd+'turtlee_interfaces/srv/Setgoal '+ custom_cmd

    return os.popen(cmd).read()
def main(args=None):
    global result
    RUN = True
    original_len = []
    procs=[]
    srv_pros=[]
    current_start = []
    new_start = []
    current_goal = []
    new_goal = []
    last_goal_sub_path=[]
    state = 0
    idx_path = 0
    idx_agent=0
    index = 0  #major index
    sub_index = 0
    path,head = convert_to_TurtlesimScreen(PATH,800.00,800.00)
    customer,q = convert_to_TurtlesimScreen(essential_pos,800.00,800.00)
    main_node = mp.Process(target=spin_main_node,args=([head]))
    main_node.start()
    num_agent = len(head)
    # service_spawn_parcel(cus=customer)
    result = mp.Array('d',(len(head)*3)+1)
    for i in range(len(head)):
        name_agent =  'turtle'+str(i+1)       
        run_controller = mp.Process(target=controller_node_run,args=(name_agent,head[i],result))
        procs.append(run_controller)
        procs[i].start()
    
    while RUN:
        if state == 0 :
            path_to_int = int_path(PATH)
            for i in path_to_int:
                original_len.append(len(i))
            fleet_pixel_equal = traffic.equal_len(path_to_int)
            get_obstacle = traffic.get_obstacle_ind(MAP_PATH) 
            start,goal = get_plan_data_list(fleet_pixel_equal,index)
            state = 1
        
        if state==1:
            last_goal_sub_path=[]        
            path = traffic.optimal_plan(start_list=start,goal_list=goal,obstacle=get_obstacle)
            subpath = sub_path(path)
            for i in range(len(subpath)):
                last_goal_sub_path.append(subpath[i][-1])      
            print(f'last_goal_sub_path : {last_goal_sub_path}')
            for i in subpath:
                current_start.append(i[sub_index])
                current_goal.append(i[sub_index+1])
            state = 2

        if state==2:
            srv_pros=[]  
            for i in range(len(current_goal)):
                name = f'turtle{i+1}'
                try:
                    goal_srv = mp.Process(target=go_to_goal,args=(name,subpath[i][sub_index],))
                    srv_pros.append(goal_srv)
                    srv_pros[i].start()
                except:
                    pass
            state=3
        
        if state==3:
            dis = lambda x1,x2,y1,y2 : ((x1-x2)**2+(y1-y2)**2)**1/2
            if result[-1]!=0.0 :
                arrive_id = int(result[-1])
                x1,y1 = last_goal_sub_path[arrive_id-1][0],last_goal_sub_path[arrive_id-1][1]
                x2,y2 = result[arrive_id*3-len(head)],result[arrive_id*3-len(head)+1]
                eul = dis(x1,x2,y1,y2)
                # print(f'This Id Goal : {last_goal_sub_path[arrive_id-1]}')
                # print(f'arrive pose : {x2,y2}')
                # print(f'eul : {dis(x1,x2,y1,y2)}')
                

                if eul<=0.01:
                    # print(f'arrive pose : {x2,y2}')
                    index+=1
                    result[-1]=0.0
                    sub_index=0
                    # new_list=[]
                    # i0,i1,i2 = arrive_id*3-3,arrive_id*3-2,arrive_id*3-1
                    # for i in range(len(result)):
                    #     if i!=i0 and i!=i1 and i!=i2:
                    #         new_list.append(result[i])
                    # del new_list[-1]
 
                    p = [[result[1],result[2]],[result[4],result[5]]]
                    print(f' v = {last_goal_sub_path}')


                    '''
                    เปลี่ยน Current goal ให้อิงตามตำแหน่งล่าสุด แต่่จ้องไปinvertกลับไป map pixel ก่อน
                    '''
                    state = 1
                else:
                    result[-1]=0.0
                    sub_index+=1
                    state=2
  


        # if state == 1 :
        #     start,goal = get_plan_data_list(fleet_pixel_equal,index)     
        #     path = traffic.optimal_plan(start_list=start,goal_list=goal,obstacle=get_obstacle)
        #     path = np.array(path*1.00)
        #     path = path.tolist()
        #     sub_path = traffic_to_sim(path)
        #     print(f'sub_path : {sub_path}')
        #     len_sub = len(sub_path[0]) 
        #     state = 5

        # if state == 2:
        #     '''
        #     Initial first goal
        #     '''
        #     goal_srv = mp.Process(target=go_to_goal,args=('turtle1',sub_path[idx_agent][idx_path],))
        #     goal_srv2 = mp.Process(target=go_to_goal,args=('turtle2',sub_path[idx_agent+1][idx_path],))
        #     goal_srv.start()
        #     goal_srv2.start()
        #     time.sleep(0.264)
        #     state=3

        # if state == 3:#sub path
        #     if result[-1]!=0.0 :
        #         id_arrive_subpath = int(result[-1])
                
  
        # if state == 3:
        #     if result[-1]!=0.0:   
        #         arrive_id = int(result[-1])             
        #         state = 4
        # if state==4:
                 
        #     if idx_path==len_path-1:
                
        #         idx_path=0
        #         result[-1] = 0.0
                
        #         index+=1
        #         state=1
        #     else:
        #         if index == original_len[arrive_id-1]:                 
        #             new_x,new_y = result[arrive_id*3-len(head)],result[arrive_id*3-len(head)+1]
        #             print(f'arrive : {arrive_id}')
        #             print(f'result : {result[:]}') 
        #             print(f'new x : {new_x} | new y : {new_y}')
        #         result[-1] = 0.0
        #         idx_path+=1
                
                

        #         state = 2
   
            
   

if __name__=='__main__': 
    traffic = Traffic_Management()
    
    main()


[[[8.78, 1.8000000000000007], [8.78, 2.1999999999999993], [8.78, 2.5999999999999996], [8.78, 3.0], [8.78, 3.4000000000000004], [8.78, 3.8000000000000007], [8.78, 4.199999999999999], [8.78, 4.199999999999999], [8.78, 4.199999999999999], [8.78, 4.199999999999999], [8.78, 4.199999999999999]]
, [8.78, 2.1999999999999993]]