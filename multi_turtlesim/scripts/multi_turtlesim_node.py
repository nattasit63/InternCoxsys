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

from multi_turtlesim.gui import GUI 
from multi_turtlesim.entity import Turtle, Parcel 

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

    
def main(args=None):
    rclpy.init(args=args)
    gui = GUI()
    node = MultiTurtleSim(gui)
    spawn_location = [2,7,9]
    
    for loc in spawn_location:
        init_pose = Pose()
        init_pose.x = 0.5
        init_pose.y = 0.5
        
        node.spawn_turtle(init_pose)
    for id,turtle in node.turtles.items():
        print(f'id: {id}, image: {turtle.image}')
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
