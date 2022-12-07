#!/usr/bin/env python3
import sys,time,os,yaml,pygame,math,random
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import UInt8MultiArray
from geometry_msgs.msg import Twist, Point
from turtlesim.msg import Pose
from std_srvs.srv import Empty
from multi_turtlesim_interfaces.srv import Pick, SpawnParcel, GetTurtleIDs, GetPosition
from functools import partial
import numpy as np

# from multi_turtlesim.traffic import Traffic_Service_Server
# import  multi_turtlesim.traffic as mt
import  full_interface.traffic as mt
import multiprocessing as mp
from turtlee_interfaces.srv import Setgoal
from full_interface.traffic import Traffic_Management,Traffic_Service_Server

def initialize(fleet,customer_pos,map_loc):
    # PATH = [[[131, 193], [164, 94], [324, 84], [325, 150], [324, 84], [164, 94], [131, 193]],
    # [[715, 275], [709, 228], [535, 278], [534, 405], [586, 577], [446, 585], [259, 716], [257, 592], [449, 292], [333, 239], [499, 144], [700, 150], [709, 228], [715, 275]], 
    # [[452, 697], [446, 585], [586, 577], [534, 405], [594, 406], [534, 405], [586, 577], [586, 633], [603, 740], [586, 633], [586, 577], [534, 405], [594, 406], [763, 407], [594, 406], [534, 405], [586, 577], [446, 585], [452, 697]]]
    # essential_pos = [[[131, 193], [715, 275], [452, 697]], [[325, 150], [333, 239], [594, 406], [763, 407], [586, 633], [603, 740], [259, 716]]]
    # MAP_PATH =  '/home/natta/interface_ws/src/full_interface/config/map_example0.png'
    PATH = fleet
    essential_pos = customer_pos
    MAP_PATH = map_loc
    return PATH,essential_pos,MAP_PATH

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
        if abs(dtheta)<0.000001:
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
    def __init__(self,Traffic):
        self.screen_width = 800
        self.screen_height = 800
        self.traffic = Traffic
        #[69,86,255]/255
        self.screen = pygame.display.set_mode([self.screen_width, self.screen_height])
        pygame.display.set_caption('Multi-Turtlesim')
        pygame.init()
        self.isQuit = False
        multi_turtlesim_path = get_package_share_directory('multi_turtlesim')
        
        map_path = os.path.join(multi_turtlesim_path,'config','map.yaml')
        with open(map_path) as f:
            self.map_parameters = yaml.load(f, Loader=yaml.loader.SafeLoader)
        
        # self.map_image = pygame.image.load(os.path.join(multi_turtlesim_path,'images','map',self.map_parameters['image'])).convert()
        self.map_image = pygame.image.load(MAP_PATH).convert()
        self.map_image = pygame.transform.scale(self.map_image,(self.screen_width, self.screen_height))
        rect = self.map_image.get_rect()
        rect = rect.move((0,0))
        # self.obstacle  = Traffic_Management(MAP_PATH,fleet=PATH).get_obstacle_ind(name='/home/natta/traffic_manager/src/multi_turtlesim/images/map/map_example0.png')
        # self.obstacle = self.traffic.get_obstacle_ind(name='/home/natta/traffic_manager/src/multi_turtlesim/images/map/map_example0.png')
        self.obstacle = self.traffic.obs_ind
        self.map_image.set_colorkey((255,255,255))
    
    def update(self,turtles,parcels):
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                self.isQuit = True
            
        # Fill the background with white
        self.screen.fill((69, 86, 255))
        multi_turtlesim_path = get_package_share_directory('multi_turtlesim')
        self.screen.blit(self.map_image,(0,0))
            
        img_parcel = pygame.image.load(os.path.join(multi_turtlesim_path,'images','parcel','parcel.png'))    
        
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
        print(f'Node : {name}_controller is spinning !')
        self.agent_name = name
        self.publisher = self.create_publisher(Twist,str(name)+'/cmd_vel',10)
        self.pose_subscription = self.create_subscription(Pose,'/'+str(name)+'/pose',self.pose_callback,10)
        self.set_goal_service = self.create_service(Setgoal,'/'+str(name)+'_set_goal',self.set_goal_callback)
        self.goal = start_position
        self.start_point = start_position
        self.current_postion = []
        self.prev_pos = np.array([0.0,0.0])
        self.first = True
        self.service_trigger = False
        self.is_dup = False
        self.is_arrive = 0
        self.pose = Pose()
        timer_period = 0.8
        self.timer = self.create_timer(timer_period,self.timer_callback)
    def timer_callback(self):
        msg = self.control()     
        self.publisher.publish(msg)
        id_agent = (int(str(self.agent_name[-1])))
        idx = (id_agent-1)*3
        result[idx]=id_agent
        result[idx+1]=self.current_postion[0]
        result[idx+2]=self.current_postion[1]
        # print(f'{self.agent_name} is {self.is_arrive}')
        if self.check_reach_goal() and self.is_arrive==0:
                self.is_arrive =1
                self.send_data()
                return True
        else:
            result[-1] = 0.0

    def send_data(self):
        id_agent = (int(str(self.agent_name[-1])))
        idx = (id_agent-1)*3
        result[-1] = id_agent
        result[idx]=id_agent
        result[idx+1]=self.current_postion[0]
        result[idx+2]=self.current_postion[1]
        self.is_arrive=0
        self.prev_pos[0],self.prev_pos[1] = self.current_postion[0],self.current_postion[1]

    def check_reach_goal(self):
        current_position = np.array([self.pose.x,self.pose.y])
        if not self.first:
            dp = self.goal-current_position
            if np.linalg.norm(dp)<0.5:
                # print(f'{self.agent_name} REACH GOAL')
                return True
    def pose_callback(self,msg):
        self.pose = msg
    def set_goal_callback(self,request,response):
        self.first=False
        self.is_arrive=0
        self.goal = np.array([request.x,request.y])
        self.prev_goal = self.goal.copy()
        return response
        
    def control(self):
        msg = Twist()
        current_position = np.array([self.pose.x,self.pose.y])
        self.current_postion = current_position
        dp = self.goal-current_position
        e = np.arctan2(dp[1],dp[0])-self.pose.theta
        K = 2.8
        w = K*np.arctan2(np.sin(e),np.cos(e))
        if np.linalg.norm(dp)>0.3:
            v = 0.15
        else:
            v = 0.0
            w = 0.0
        msg.linear.x = v
        msg.angular.z = w
        return msg
class Function():
    def get_pos_result(self,num_agent,result):
        list_pos = []
        res = result
        for i in range(num_agent):
            list_pos.append([])

        for i in range(len(list_pos)):
            list_pos[i]=[res[i*3+1],res[i*3+2]]
        return list_pos
    def sim_to_traffic(self,data):
        result=[]
        for i in range(len(data)):
            result.append([]) 
        for i in range(len(data)):
            result[i]= [int(round(data[i][0]*800.00/16.00)),int(round(800.00-(data[i][1]*800.00/16.00)))]
        return result
    def convert_to_TurtlesimScreen(self,fleet_pixel,w,h):
        # print(f'flett_px:{fleet_pixel}')
        # print(f'w : {w}')
        # print(f'h : {h}')
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
    def traffic_to_sim(self,q):
        # print(f'traffic_tosim : q = {q[0]}')
        a =[]
        for i in range(len(q)):
            a.append([])
        for i in range(len(q)):
            for j in q[i]:
                a[i].append([j[0]*16.00/800.00,abs(16.00-(j[1]*16.00/800.00))])
        return a
    def sub_path(self,path):
        # print(f'path ; {path}')
        path = np.array(path*1.00)
        path = path.tolist()
        sub_path = self.traffic_to_sim(path)
        return sub_path
    def get_plan_data_list(self,x,id):
        start = []
        goal = []
        for i in range(len(x)):
            start.append(x[i][id])
            goal.append(x[i][id+1])
        # print('START :',start)
        # print('GOAL  :',goal)
        return start,goal
    def go_to_goal(self,agent,goal):
        # print(f' RECIEVE : agent={agent} and goal ={goal}')
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
    def service_spawn_parcel(self,cus):
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
    def fullplan_to_sim(self,full_path):
        # x = np.array(full_path)
        # x = x.tolist()     
        x= full_path
        for i in range(len(x)):
            x[i] = np.array(x[i]*1.00)
            for j in range(len(x[i])):
                for l in range(len(x[i][j])):
                    x[i][j][l][0] =  x[i][j][l][0]*16.00/800.00
                    x[i][j][l][1] = abs(16.00-(x[i][j][l][1]*16.00/800.00))
        return x
                
def spin_main_node(head):
    rclpy.init(args=None)
    gui = GUI(traffic)
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
def spin_traffic_node(mm):
    rclpy.init(args=None)
    traffic_node = Traffic_Service_Server(mm)
    # print('----------------------------------------------------------')
    rclpy.spin(traffic_node)
    traffic_node.destroy_node()
    rclpy.shutdown()
def main(fleet,customer_pos,full_path):
    global result
    RUN,first_run = True,True
    function = Function()
    procs,goal_srv_pros,current_goal,cur_id,id_agent = [],[],[],[],[]
    state,sub_index,major_index = 0,0,0
    max_len = 0
    path,head = function.convert_to_TurtlesimScreen(fleet,800.00,800.00)
    customer,q = function.convert_to_TurtlesimScreen(customer_pos,800.00,800.00)
    """
    Just for this example :
        Use Multi processing to spin multiple of nodes
        -Main node : Multi Turtlesim node
        -Sub node  : Turtle Controller (amount up to number of agent)
        -Service   : Create parcels,Go2Goal
    """
    main_node = mp.Process(target=spin_main_node,args=([head]))
    main_node.start()
    # function.service_spawn_parcel(cus=customer)
    result = mp.Array('d',(len(head)*3)+1)
    for i in range(len(head)):
        cur_id.append(0)
        id_agent.append(i)
        name_agent =  'turtle'+str(i+1)       
        run_controller = mp.Process(target=controller_node_run,args=(name_agent,head[i],result))
        procs.append(run_controller)
        procs[i].start()

    """
    Visualise in while loop
    """

    turtlesim_path = function.fullplan_to_sim(full_path) #Convert pixel to turtlesim screen
    num_path = len(turtlesim_path)
    print(f'NUM_path : {num_path}')

    while RUN:
        if state == 0:
            # print(len(turtlesim_path[0]))
            id = id_agent
            goal_srv_pros=[]
            # print(f'Id : {id}')
            
            # print(f'Max len : {max_len} | major index : {major_index}  | sub_index : {sub_index}')
            if major_index == num_path-1 and first_run==False:
                print('-'*80,'\n')
                print('\t\tComplete ','\n')
                print('-'*80,'\n')
                sys.exit()
            else:
                first_run == False
                try:
                    for i in range(len(id)): 
                        time.sleep(0.3)
                        name = f'turtle{id[i]+1}'
                        goal_srv = mp.Process(target=function.go_to_goal,args=(name,turtlesim_path[major_index][i][sub_index],))
                        goal_srv_pros.append(goal_srv)
                        goal_srv_pros[i].start()
                        max_len = len(turtlesim_path[major_index][i])
                except:
                    print('-'*80,'\n')
                    print('\t\tComplete ','\n')
                    print('-'*80,'\n')
                    sys.exit()
                time.sleep(0.3)
                state = 1
            if state ==1:
                if major_index<num_path:
                    if sub_index<=max_len-2:
                        sub_index+=1
                        
                    else:
                        major_index+=1
                        sub_index =0
                    state = 0
                else:
                    print('-'*80,'\n')
                    print('\t\tComplete','\n')
                    print('-'*80,'\n')
                    sys.exit()



    




def run(fleet,customer_pos,map_loc):
    global traffic
    PATH,essential_pos,MAP_PATH = initialize(fleet,customer_pos,map_loc)
    traffic = Traffic_Management()
    obs = traffic.get_obstacle_ind(MAP_PATH)
    full_planning_path = traffic.full_plan(fleet=PATH,obstacle=obs)

    print(full_planning_path)
    # main(fleet,customer_pos,full_planning_path)




if __name__=='__main__':
    PATH = [[[45,342],[730,370]],[[750,330],[410,450]],[[405,410],[50,355]]]

    PATH = [[[189, 598], [266, 490], [40, 378], [77, 210], [40, 378], [266, 490], [189, 598], [266, 490], [355, 578], [554, 593], [615, 605], [691, 690], [576, 727], [554, 593], [355, 578], [266, 490], [189, 598], [266, 490], [40, 378], [178, 207], [141, 155], [77, 210], [40, 378], [266, 490], [189, 598]]]
    
    
    essential_pos =[[[60, 449], [669, 729]], [[242, 415], [322, 574], [240, 687], [713, 490], [601, 519]]]
    MAP_PATH ='/home/natta/interface_ws/src/full_interface/config/tutorial0.png'
    MAP_PATH ='/home/natta/interface_ws/src/full_interface/config/map_example0.png'

    run(PATH,essential_pos,MAP_PATH)





 
