#!/usr/bin/env python3
from array import array
import time
from unicodedata import name
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
import multiprocessing as mp
import os
from std_srvs.srv import Empty
import sys, errno  
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlee_interfaces.srv import Setgoal
from traffic import Traffic_Management

MAP_PATH  ='/home/natta/interface_ws/src/full_interface/config/map_demo.pgm'
SAMPLE =    [[[129, 164], [233, 159], [245, 69], [323, 101], [245, 69], [233, 159], [129, 164]], 
             [[226, 427], [245, 384], [350, 421], [377, 314], [299, 242], [323, 320], [262, 316], [222, 294], [299, 242], [381, 163], [299, 242], [262, 316], [245, 384], [226, 427]]]
#Input to this file
PIXEL = [[[18.06, 23.233333333333334], [32.62, 22.525], [34.3, 9.775], [45.22, 14.308333333333334], [34.3, 9.775], [32.62, 22.525], [18.06, 23.233333333333334]],
        [[31.64, 60.49166666666667], [34.3, 54.4], [49.0, 59.641666666666666], [52.78, 44.483333333333334], [41.86, 34.28333333333333], [45.22, 45.333333333333336], [36.68, 44.766666666666666], [31.08, 41.65], [41.86, 34.28333333333333], [53.34, 23.091666666666665], [41.86, 34.28333333333333], [36.68, 44.766666666666666], [34.3, 54.4], [31.64, 60.49166666666667]]]
map_demo_size = [112,85]
NUM_AGENT = len(PIXEL)
RUN = True

# qq = mp.Queue()




###########
class Controller(Node):
    def __init__(self,name,start_position,que):
        super().__init__(str(name)+'_controller')
        self.agent_name = name
        self.command_publisher = self.create_publisher(Twist,str(name)+'/cmd_vel',10)
        self.pose_subscription = self.create_subscription(Pose,'/'+str(name)+'/pose',self.pose_callback,10)
        self.set_goal_service = self.create_service(Setgoal,'/'+str(name)+'_set_goal',self.set_goal_callback)
        self.goal = start_position
        self.start_point = start_position
        self.first = True
        self.is_arrive = 0
        self.pose = Pose()
        timer_period = 0.1
        self.timer = self.create_timer(timer_period,self.timer_callback)
    def timer_callback(self):
        
        msg = self.control()
        
        self.command_publisher.publish(msg)
        if self.check_reach_goal() and self.is_arrive==0:
            self.is_arrive =1
            print(str(self.agent_name),' Arrive')
            # qq.put(str(self.agent_name))
            id_agent = (int(str(self.agent_name[-1])))
            idx = (id_agent-1)*3
            print('current pose :',current_pose)
            result[-1] = id_agent
            result[idx]=id_agent
            result[idx+1]=current_pose[0]
            result[idx+2]=current_pose[1]
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
        global current_pose
        msg = Twist()
        current_position = np.array([self.pose.x,self.pose.y])
        current_pose = current_position
        dp = self.goal-current_position
        e = np.arctan2(dp[1],dp[0])-self.pose.theta
        K = 10.0
        w = K*np.arctan2(np.sin(e),np.cos(e))
        if np.linalg.norm(dp)>0.2:
            v = 0.5
        else:
            v = 0.0
            w = 0.0
        msg.linear.x = v
        msg.angular.z = w
        return msg

def convert_to_TurtlesimScreen(fleet_pixel,w,h):
    ans = []
    head = []
    real_ans = []
    for i in range(len(fleet_pixel)):
        z=[]
        for j in range(len(fleet_pixel[i])):
            q=[]
            q.append(fleet_pixel[i][j][0]*500.00/w)
            q.append(fleet_pixel[i][j][1]*500.00/h)
            z.append(q)
        ans.append(z)
    for i in range(len(ans)):
        z=[]
        for j in range(len(ans[i])):
            q=[]       
            q.append(ans[i][j][0]*11.120889/500.0000)
            q.append(ans[i][j][1]*11.120889/500.0000)
            z.append(q)
            if j==0:
                head.append([ans[i][0][0]*11.120889/500.0000,ans[i][0][1]*11.120889/500])
        real_ans.append(z)
    return real_ans,head

def multi_node_run(name,start,q):
    rclpy.init(args=None)
    name = Controller(name,start_position=start,que=q)
    rclpy.spin(name)
    name.destroy_node()
    rclpy.shutdown()
    


def run_turtlenode():
    cmd = 'ros2 run turtlesim turtlesim_node'
    os.popen(cmd)

def spawn_agent(num_agent,pos):
    def add_cmd(x,y,name):
        result = '"' +'{'+'x: '+str(x)+', '+'y: '+str(y)+', '+'name: '+str(str(name))+'}'+'"'
        return result
    kill_turtle1 = 'ros2 service call /kill turtlesim/srv/Kill "{name : "turtle1"}"'
    for i in range(num_agent):
        spawn_pos = pos[i]
        name = 'agent'+str(i+1)
        
        custom_cmd = add_cmd(spawn_pos[0],spawn_pos[1],name)
        cmd = 'ros2 service call /spawn turtlesim/srv/Spawn '+ custom_cmd   
        os.popen(cmd)
    os.popen(kill_turtle1)

def go_to_goal(agent,goal):
    cmd_list = []
    def add_cmd(x,y):
        result = '"' +'{'+'x: '+str(x)+', '+'y: '+str(y)+'}'+'"'
        return result
    def agent_name(agent):
        result = "/" + str(agent)+"_set_goal "
        return result
    for i in range(len(goal)):
        custom_cmd = add_cmd(goal[i][0],goal[i][1])
        agent_srv_cmd = agent_name(agent)
        cmd = 'ros2 service call '+agent_srv_cmd+'turtlee_interfaces/srv/Setgoal '+ custom_cmd
        
    return os.popen(cmd)
  
def get_plan_data_list(x,id):
    start = []
    goal = []
    for i in range(len(x)):
        start.append(x[i][id])
        goal.append(x[i][id+1])
    print('START :',start)
    print('GOAl  ;',goal)
    return start,goal


def main(args=None):

    state = 0
    minor_state = 0
    index = 1
    idx_path = 0
    idx_agent=0
    procs = []
    srv_procs = []
    cmd_srv = []
    num_agent = NUM_AGENT
    cvt,head = convert_to_TurtlesimScreen(PIXEL,map_demo_size[0],map_demo_size[1])
    
    
    for i in range(num_agent):    
        name_agent =  'agent'+str(i+1)       
        run_controller = mp.Process(target=multi_node_run,args=(name_agent,head[i],result))
        # run_controller = threading.Thread(target=multi_node_run, args=(name_agent,head[i],))
        procs.append(run_controller)
        procs[i].start()

    # msg_terminal = mp.Process(target=print_terminal)
    # procs.append(msg_terminal)
    # msg_terminal.start()
    while(RUN):
        if state==0:
            try:
                open_turtlesim = mp.Process(target=run_turtlenode)
                procs.append(open_turtlesim)
                open_turtlesim.start()
            except IOError as e:  
                if e.errno == errno.EPIPE:
                    pass
            state =1        
        elif state ==1:   
            cvt,head = convert_to_TurtlesimScreen(PIXEL,map_demo_size[0],map_demo_size[1])
            spawn_agent(len(cvt),head)
            state = 2
        
        elif state==2:
            if minor_state==0:     
                ##to do len of data equal
                fleet_pixel_equal = traffic.equal_len(SAMPLE)
                get_obstacle = traffic.get_obstacle_ind(MAP_PATH)
                minor_state =1
            if minor_state==1:
                start,goal = get_plan_data_list(fleet_pixel_equal,index)     
                path = traffic.optimal_plan(start_list=start,goal_list=goal,obstacle=get_obstacle)
                ##Apply turtlesim scale
                path = np.array(path*11.120889/500.00)
                path = path.tolist()
                len_path = len(path[0]) 
                
                minor_state=2
            
            if minor_state==2:
                # print('22222222222')
                goal_srv = mp.Process(target=go_to_goal,args=('agent2',[path[idx_agent+1][idx_path]],))
                goal_srv0 = mp.Process(target=go_to_goal,args=('agent1',[path[idx_agent][idx_path]],))
                print('goal srv')
                goal_srv.start()
                goal_srv0.start()
                print('goal srv start')
                minor_state=3
                '''
                To do list : check is Arrive? on multiple process
                '''
                # for i in range(num_agent):
                #     name_agent =  'agent'+str(i+1) 
                #     goal_srv = mp.Process(target=go_to_goal,args=(name_agent,[path[i][0]],))
                #     goal_srv.start()

                
            if minor_state==3:
                # print('len_path : ' ,len_path)    
                if result[-1]!=0.0:
                    print('33333333')      
                    print(result[-1])
                    minor_state = 4
            
            if minor_state==4:
                if idx_path==len_path-1:
                    idx_path=0
                    result[-1] = 0.0
                    index+=1
                    minor_state=1
                else:
                    result[-1] = 0.0
                    idx_path+=1
                    minor_state = 2
                # msg_share_mem = qq.empty()
                # while msg_share_mem is not False:
                #     print(qq.get())
                    

               
        
        
            


if __name__ == '__main__':
    traffic = Traffic_Management()
    result = mp.Array('d',(NUM_AGENT*3)+1)
    main()
    