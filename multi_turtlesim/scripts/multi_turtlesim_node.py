#!/usr/bin/python3
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
import networkx
import pygame
from ament_index_python.packages import get_package_share_directory
import os 
import numpy as np
import math,yaml
from multi_turtlesim.traffic import Traffic_Management
import multi_turtlesim.traffic as mt
# from multi_turtlesim.gui import GUI 
# from multi_turtlesim.entity import Turtle, Parcel 

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
                self.customer_circle(parcel.position.x/16.0*800,parcel.position.y/16*800,5)
                # self.screen.blit(img_parcel,(parcel.position.x/5.0*250,self.screen_width-parcel.position.y/5.0*250))
                # pygame.draw.circle(self.screen, (255,0,0),pygame.Rect(parcel.position.x/5.0*250,self.screen_width-parcel.position.y/5.44445*250,cell_size,cell_size))
                # pygame.draw.rect(self.screen,(255,0,0),pygame.Rect(parcel.position.x/5.0*250,self.screen_width-parcel.position.y/5.44445*250,cell_size,cell_size))
        # Flip the display
        pygame.display.flip()
    
    def customer_circle(self,pos_x,pos_y,radius):
        pygame.draw.circle(self.screen, (255,0,0),[pos_x,pos_y],radius)
        # pygame.display.flip()

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
        self.timer_period = 0.3
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

PATH = [[[29.0, 210.0], [122.0, 202.0], [28.0, 101.0], [23.0, 31.0], [228.0, 29.0], [23.0, 31.0], [28.0, 101.0], [122.0, 202.0], [29.0, 210.0]],
        [[614.0, 726.0], [558.0, 728.0], [551.0, 586.0], [498.0, 587.0], [418.0, 584.0], [411.0, 402.0], [248.0, 381.0], [411.0, 402.0], [418.0, 584.0], [498.0, 587.0], [551.0, 586.0], [539.0, 266.0], [747.0, 299.0], [539.0, 266.0], [551.0, 586.0], [498.0, 587.0], [418.0, 584.0], [431.0, 706.0], [418.0, 584.0], [411.0, 402.0], [391.0, 277.0], [260.0, 278.0], [301.0, 181.0], [253.0, 100.0], [423.0, 174.0], [391.0, 277.0], [411.0, 402.0], [418.0, 584.0], [498.0, 587.0], [551.0, 586.0], [558.0, 728.0], [614.0, 726.0]]]

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
                head.append([ans[i][0][0]*16.00/800.00,ans[i][0][1]*16.00/800.00])
        real_ans.append(z)
    return real_ans,head

def spawn_parcel(path):
    def add_cmd(x,y):
        result = '"'+'position: '+'{'+'x: '+str(x)+','+'y: '+str(y)+',z: 0'+'}'+'"'
        return result
    num_agent = len(path)
    for i in range(num_agent):
        print('-'*20)
        remove_last = path[i].copy()
        remove_last = remove_last[:-1]
        for j in remove_last:
            custom_cmd = add_cmd(j[0],j[1])
            cmd = 'ros2 service call /spawn_parcel multi_turtlesim_interfaces/srv/SpawnParcel '+custom_cmd
            os.popen(cmd)

def main(args=None):
    rclpy.init(args=args)
    gui = GUI()
    node = MultiTurtleSim(gui)
    path,head = convert_to_TurtlesimScreen(PATH,800.00,800.00)
    for loc in head:
        init_pose = Pose()
        init_pose.x = loc[0]
        init_pose.y = loc[1]
        node.spawn_turtle(init_pose)
    for id,turtle in node.turtles.items():
        print(f'id: {id}, image: {turtle.image}')
    spawn_parcel(path=PATH)
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__=='__main__':
   

    main()
