#!/usr/bin/python3

from PIL import Image
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor

from ament_index_python.packages import get_package_share_directory
import os 
import random, math
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import Empty
from multi_turtlesim_interfaces.srv import Pick, SpawnParcel, GetTurtleIDs, GetPosition
from multi_turtlesim_interfaces.action import PlanPath, PlanPathID
from functools import partial
import yaml
import networkx
from cbs_mapf.planner import Agent

from multi_turtlesim.entity import Turtle, Parcel 
from typing import List, Tuple, Dict, Callable, Set

def direct_assign(starts: List[Tuple[int, int]], goals: List[Tuple[int, int]]):
    # The number of start positions must be equal to the number of goal positions
    assert(len(starts) == len(goals))
    agents = []
    for i in range(len(starts)):
        agents.append(Agent(starts[i], goals[i]))
    
    return agents


class TrafficClient(Node):
    def __init__(self):
        super().__init__('traffic_client')
        
        self.get_ids_client = self.create_client(GetTurtleIDs,'/get_turtle_ids')
        
        while not self.get_ids_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('no simulation is running, waiting again...')
        self.turtle_ids = self.send_get_IDs_request()

        self.get_position_client = self.create_client(GetPosition,'/get_position')
        while not self.get_position_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('no /get_position is running, waiting again...')
    def send_get_position_request(self,id):
        # call and wait for position response from the controller
        req = GetPosition.Request()
        req.id = id
        self.future = self.get_position_client.call_async(req)
        while not self.future.done():
            #rclpy.spin_once(self)
            self.get_logger().info('Get current position...')
        # print(self.future.result().position)
        return self.future.result().position,self.future.result().flag
    def send_get_IDs_request(self):
        # call and wait for position response from the controller
        req = GetTurtleIDs.Request()
        # print(req)
        self.future = self.get_ids_client.call_async(req)
        rclpy.spin_until_future_complete(self,self.future)
            
        return self.future.result().ids
        
def dist(x1,y1,x2,y2):
    return ((x1-x2)**2+(y1-y2)**2)**0.5      
class TrafficController(Node):    
    def __init__(self,client_node:TrafficClient):
        super().__init__('traffic_controller')
        self.client_node = client_node
        #load traffic info    
        multi_turtlesim_traffic_path = get_package_share_directory('multi_turtlesim_traffic')
        
        traffic_path = os.path.join(multi_turtlesim_traffic_path,'config','traffic.yaml')
        with open(traffic_path) as f:
            self.traffic_info = yaml.load(f, Loader=yaml.loader.SafeLoader)
        # create graph
        self.graph = networkx.Graph()
        id = 0
        for node in self.traffic_info['vertices']:
            self.graph.add_node(id,pos=node)
            id = id + 1
        for edge in self.traffic_info['edges']:
            self.graph.add_edge(edge[0],edge[1])
        self.pos = networkx.get_node_attributes(self.graph,'pos')
        networkx.set_edge_attributes(self.graph,{e: math.sqrt((self.pos[e[0]][0]-self.pos[e[1]][0])**2+(self.pos[e[0]][1]-self.pos[e[1]][1])**2)  for e in self.graph.edges()},"cost")
        # establish timer
        self.plan_server= self.create_service(Empty,'/plan',self.plan)
        
        self.turtle_ids = self.client_node.send_get_IDs_request()

        self.plan_path_server = ActionServer(self,PlanPath,'/plan_path',self.plan_path_callback)
        self.plan_path_server = ActionServer(self,PlanPathID,'/plan_path_id',self.plan_path_id_callback)
        
        self.timer_period = 0.5
        self.timer = self.create_timer(self.timer_period,self.timer_callback)
        
        multi_turtlesim_path = get_package_share_directory('multi_turtlesim')
        map_path = os.path.join(multi_turtlesim_path,'config','map.yaml')
        with open(map_path) as f:
            self.map_info = yaml.load(f, Loader=yaml.loader.SafeLoader)
        multi_turtlesim_path = get_package_share_directory('multi_turtlesim')
        map_path = os.path.join(multi_turtlesim_path,'images','map',self.map_info['image'])
        
        im = Image.open(map_path)
        width,height = im.size
        res = self.map_info['resolution']
        num_cell_width = int(width*res)
        num_cell_height = int(height*res)
        im = im.resize((num_cell_width,num_cell_height))
        map2 = [[0 for i in range(num_cell_width)] for i in range(num_cell_height)]
        map_data = [0 for i in range(len(list(im.getdata())))]
        id = 0
        obstacle = []
        for pixel in list(im.getdata()):
            if all(p == 255 for p in pixel):
                map2[id%num_cell_width][id//num_cell_width] = 0
                map_data[id] = int(0)
            else:
                map2[id%num_cell_width][id//num_cell_width] = 1
                map_data[id] = int(100)
                obstacle.append((int((id%num_cell_width+1/2)*50),int((id//num_cell_width+1/2)*50)))
            id = id + 1
        for x in range(0,width+int(1/res),int(1/res)):
            obstacle.append((x,0))
            obstacle.append((x,(height//int(1/res)+1)*(int(1/res))))
        for y in range(0,height+int(1/res),int(1/res)):
            obstacle.append((0,y))
            obstacle.append(((width//int(1/res)+1)*(int(1/res)),y))
        
        #print(obstacle)
        
        cell_size = num_cell_height
        radius = 25

        # add traffic planner here
        
        map_msg = OccupancyGrid()
        map_msg.data = map_data
        map_msg.header.frame_id = '/map'
        t = self.get_clock().now().to_msg()
        map_msg.header.stamp = t
        map_msg.info.map_load_time = t
        map_msg.info.resolution = res
        map_msg.info.width = num_cell_width
        map_msg.info.height = num_cell_height
        map_msg.info.origin.position.x = self.map_info['origin'][0]
        map_msg.info.origin.position.y = self.map_info['origin'][1]
        map_msg.info.origin.position.z = self.map_info['origin'][2]
        self.map = map_msg
        self.pub = self.create_publisher(OccupancyGrid,'/map',10)
    def timer_callback(self):
        self.pub.publish(self.map)
    def plan_path_callback(self,goal_handle):
        self.get_logger().info(f'Executing Path Planning...')
        # obtain the start time and initial position when called
        # print(goal_handle.request.goal)
        init_time = self.get_clock().now()
        
        # set goal at the controller 
        
        source = goal_handle.request.source_id
        target = goal_handle.request.target_id
        path = networkx.astar_path(self.graph,source,target, weight="cost")  

        feedback_msg = PlanPath.Feedback() 
        first_time = True
        if path:
            goal_handle.succeed()
        else:
            # replan
            if first_time:
                dt = self.get_clock().now()-init_time
                feedback_msg.elaspedTime = dt
                goal_handle.publish_feedback(feedback_msg)
                path = networkx.astar_path(self.graph,source,target, weight="cost") 
                first_time = False
            else:
                goal_handle.abort()
        result = PlanPath.Result()
        result.path = path
        return result       
    def plan_path_id_callback(self,goal_handle):
        id = goal_handle.request.turtle_id
        self.get_logger().info(f'Executing Path Planning for Turtle with ID: {id} ...')

        init_time = self.get_clock().now()
        
        
        target = goal_handle.request.target_id
        
        p_turtle,flag = self.client_node.send_get_position_request(id)
        if not flag :
            self.get_logger().warning(f'No turtle with ID: {id}')
            goal_handle.abort()
            result = PlanPathID.Result()
            return result
        min_cost = 10000
        closest_node = -1
        for node,p in self.pos.items():
            d = dist(p[0],p[1],p_turtle.x,p_turtle.y)
            if d<min_cost:
                closest_node = node
                min_cost = d
        print(p_turtle)
        path = networkx.astar_path(self.graph,closest_node,target, weight="cost")  

        feedback_msg = PlanPathID.Feedback() 
        if not path:
            self.get_logger().warning(f'No path exists')
            goal_handle.abort()
            result = PlanPathID.Result()
            return result
        p0 = self.pos[path[0]]
        p1 = self.pos[path[1]]
        #a = dist(p_turtle.x,p_turtle.y,p0[0],p0[1])
        b = dist(p0[0],p0[1],p1[0],p1[1])
        c = dist(p_turtle.x,p_turtle.y,p1[0],p1[1])
        if b>c:
            path = networkx.astar_path(self.graph,path[1],target, weight="cost")  
        first_time = True
        if not path:
            self.get_logger().warning(f'No path exists')
            goal_handle.abort()
            result = PlanPathID.Result()
            return result
        if path:
            goal_handle.succeed()
        else:
            # replan
            if first_time:
                dt = self.get_clock().now()-init_time
                feedback_msg.elaspedTime = dt
                goal_handle.publish_feedback(feedback_msg)
                path = networkx.astar_path(self.graph,closest_node,target, weight="cost") 
                first_time = False
            else:
                goal_handle.abort()
        result = PlanPathID.Result()
        result.path = path
        return result         
    def plan(self,request,response):
        targets = {1:6,2:21,3:15}
        paths = []
        for id in self.turtle_ids:
            p_turtle,flag = self.client_node.send_get_position_request(id)
            min_cost = 10000
            closest_node = -1
            for node,p in self.pos.items():
                d = dist(p[0],p[1],p_turtle.x,p_turtle.y)
                if d<min_cost:
                    closest_node = node
                    min_cost = d
            path = networkx.astar_path(self.graph,closest_node,targets[id], weight="cost")  
            
            
            p0 = self.pos[path[0]]
            p1 = self.pos[path[1]]
            #a = dist(p_turtle.x,p_turtle.y,p0[0],p0[1])
            b = dist(p0[0],p0[1],p1[0],p1[1])
            c = dist(p_turtle.x,p_turtle.y,p1[0],p1[1])
            if b>c:
                path = networkx.astar_path(self.graph,path[1],targets[id], weight="cost")  
                
            paths.append(path)
        print(paths)
        return response


    
def main(args=None):
    rclpy.init(args=args)
    try:
        client_node = TrafficClient()
        node = TrafficController(client_node=client_node)
        # create executor
        executor = MultiThreadedExecutor(num_threads=4)
        # add nodes to the executor
        executor.add_node(node)
        executor.add_node(client_node)
        try:
            # spin both nodes in the executor
            executor.spin()
        finally:
            executor.shutdown()
            node.destroy_node()
            client_node.destroy_node()
    finally:
        rclpy.shutdown()

if __name__=='__main__':
    main()
