#!/usr/bin/python3

from geometry_msgs.msg import Twist, Point
import numpy as np
from turtlesim.msg import Pose
import math


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
