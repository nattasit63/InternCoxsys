#!/usr/bin/python3
import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

from std_srvs.srv import Empty
from turtlesim_interfaces.srv import SetGoal


class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        self.command_publisher = self.create_publisher(Twist,'/cmd_vel',10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period,self.timer_callback)
        self.pose_subscription = self.create_subscription(Pose,'/pose',self.pose_callback,10)
        self.pose = Pose()
        self.set_goal_service = self.create_service(SetGoal,'/set_goal',self.set_goal_callback)
        self.enable_service = self.create_service(Empty,'/enable',self.enable_callback)
        self.notify_arrival_client = self.create_client(Empty,'/notify_arrival')
        
        self.goal = np.array([2.0,3.0])
        self.isEnable = False
        self.declare_parameters(namespace='',parameters=[('gain',5.0),])

    def timer_callback(self):
        if self.isEnable:
            msg = self.control()
            self.command_publisher.publish(msg)
    def pose_callback(self,msg):
        self.pose = msg
    def control(self):
        msg = Twist()
        current_position = np.array([self.pose.x,self.pose.y])
        dp = self.goal-current_position
        e = np.arctan2(dp[1],dp[0])-self.pose.theta
        K = self.get_parameter('gain').get_parameter_value().double_value
        w = K*np.arctan2(np.sin(e),np.cos(e))
        if np.linalg.norm(dp)>0.1:
            v = 1.0
        else:
            v = 0.0
            w = 0.0
            self.isEnable = False
            self.send_notify_arrival_request()

        msg.linear.x = v
        msg.angular.z = w
        return msg
    def set_goal_callback(self,request,response):
        self.goal = np.array([request.position.x,request.position.y])
        return response
    def enable_callback(self,request,response):
        self.isEnable = True
        return response
    def send_notify_arrival_request(self):
        req = Empty.Request()
        self.future = self.notify_arrival_client.call_async(req)

def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()