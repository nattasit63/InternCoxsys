 
from traceback import print_tb
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import numpy as np
import math
from nav_msgs.msg import Odometry
# from coconut_srv.srv import SetGoal


class ControllerRobot(Node):
    def __init__(self):
        super().__init__('controlrobot')

        timer_period = 0.1
        self.timer = self.create_timer(timer_period,self.timer_callback)
        self.init_time = self.get_clock().now()
        self.odom_subscription = self.create_subscription(Odometry,'/coconut/odom',self.odom_callback,10)
        self.goal=np.array([0.0,0.0])
        # self.current_pose=np.array([0.0,0.0])
        self.odom_publisher = self.create_publisher(Twist,'coconut/cmd_vel',10)
        self.state = 0
        self.k_w=4.5
        self.k_v=0.45

        # self.coconut_srv_setgoal = self.create_service(SetGoal, '/coconut/srv/setgoal', self.setgoal_srv_callback)

    def odom_callback(self,msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        self.current_pose = [self.current_x,self.current_y]

        self.euler_z = self.euler_from_quaternion(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
        return self.current_pose

    def euler_from_quaternion(self,x, y, z, w):

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        
        return  yaw_z
    
    def setgoal_srv_callback(self,request,response):
        self.goal = np.array([request.x,request.y])
        self.get_logger().info('Set Goal to : [%f,%f]' % (request.x,request.y))
        return response

    def timer_callback(self):
        if self.state==0:
            msg = Twist()
            dp = self.goal - self.current_pose
            e = math.atan2(dp[1],dp[0])-self.euler_z
            w = self.k_w*math.atan2(math.sin(e),math.cos(e))
            msg.angular.z = w
            self.odom_publisher.publish(msg)
            # print(e)
            if abs(e)<0.00001:
                self.state=1
                print("state=1")
                w=0.0
                msg.angular.z = w
                self.odom_publisher.publish(msg)
        elif self.state==1:
            dp = self.goal - self.current_pose
            dist = np.linalg.norm(dp)
            msg = Twist()
            # print(dist)
            if dist >0.1:
                msg.linear.x = self.k_v
                self.odom_publisher.publish(msg)
            else:
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.odom_publisher.publish(msg)
                
                print("COMPLETE")
    
def main(args=None):
    rclpy.init(args=args)
    controlrobot = ControllerRobot()
    rclpy.spin(controlrobot)
    controlrobot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()