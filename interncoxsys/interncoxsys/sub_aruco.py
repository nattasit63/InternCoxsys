#!/usr/bin/env python3
from cmath import pi
import sys
import geometry_msgs.msg
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray,UInt16MultiArray
from aruco.msg import Custom
import numpy as np

ArucoTopic = '/coconut/Aruco'


class ArucoSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(Custom,ArucoTopic,self.listener_callback,10)

    def listener_callback(self, msg):
        self.id = (msg.id.data).tolist()
        self.tvec = (msg.tvec.data).tolist()
        self.eul = (msg.eul.data).tolist()
        self.qua = (msg.qua.data).tolist()

        #collect message to list
        # id = [1,2,3,...]
        # translationVector = [[x1,y1,z1],[x2,y2,z2],[x3,y3,z3],...]
        # eulerAngle = [[r1,p1,y1],[r2,p2,y2],[r3,p3,y3],...]
        # quaternion = [[qx1,qy1,qz1,qw1],[qx2,qy2,qz2,qw2],[qx3,qy3,qz3,qw3],...]
        self.translationVector = []
        self.eulerAngle = []
        self.quaternion = []
        if self.id != []:
            for i in range(len(self.id)):
                x = self.tvec[(3*(i-1))+0]
                y = self.tvec[(3*(i-1))+1]
                z = self.tvec[(3*(i-1))+2]
                self.translationVector.append([x,y,z])
                
                r = self.eul[(3*(i-1))+0]*180/pi
                p = self.eul[(3*(i-1))+1]*180/pi
                y = self.eul[(3*(i-1))+2]*180/pi
                self.eulerAngle.append([r,p,y])

                qx = self.qua[(4*(i-1))+0]
                qy = self.qua[(4*(i-1))+1]
                qz = self.qua[(4*(i-1))+2]
                qw = self.qua[(4*(i-1))+3]
                self.quaternion.append([qx,qy,qz,qw])
        self.get_logger().info("Found "+str(len(self.id))+" Marker(s)")
        print("*************************************************\nID = ")    
        print(self.id)
        print("*************************************************\nTranslation Vector = ")    
        print(self.translationVector)
        print("*************************************************\nEuler Angle = ")  
        print(self.eulerAngle)
        print("*************************************************\nQuaternion = ")  
        print(self.quaternion)
def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = ArucoSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()