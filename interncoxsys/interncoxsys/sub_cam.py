#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32MultiArray,UInt16MultiArray
from aruco.msg import Custom
from ament_index_python.packages import get_package_share_directory

import cv2
import numpy as np
from utils import ARUCO_DICT, aruco_display, aruco_pose_estimation


arucoDict = arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_100)
arucoParams = cv2.aruco.DetectorParameters_create()

cam_matrix = [476.703,0.0,400.5,
                0.0,476.703,400.5,
                0.0,0.0,1.0]
cam_dist = [0.0,0.0,0.0,0.0,0.0]

CameraTopic = '/coconut/camera/image_raw'
ArucoTopic = '/coconut/Aruco'

marker_length = 0.1 #10cm


x = np.array(cam_matrix,dtype=np.float32)
# x = np.array(cam_matrix)
shape = (3,3)
cam_matrix = x.reshape(shape)

x = np.array(cam_dist,dtype=np.float32)
# x = np.array(cam_dist)
shape = (1,5)
cam_dist = x.reshape(shape)


class CameraSubsciber(Node):
    def __init__(self):
        super().__init__('subcam') #cameraShow_aum = node name
        self.sub = self.create_subscription(Image,CameraTopic,self.listener_callback,10) #create subsciber

        #########################################
        self.pub = self.create_publisher(Custom, ArucoTopic, 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        #########################################

        self.bridge = CvBridge()


    
    def listener_callback(self,img_msg):
        self.frame = self.bridge.imgmsg_to_cv2(img_msg,"bgr8") #camera frame rgb

        self.gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY) #change to gray frame
        self.corners, self.ids, self.rejected = cv2.aruco.detectMarkers(self.gray, arucoDict, parameters=arucoParams)  #get marker detail
        cv2.aruco.drawDetectedMarkers(self.frame, self.corners) 
        self.Marker = aruco_pose_estimation(self.frame,self.ids,self.corners,marker_length,cam_matrix,cam_dist)

        cv2.imshow("A",self.frame)
        cv2.waitKey(1)   

    def timer_callback(self):
        msg = Custom()
        try :
            Marker = self.Marker
            ID = Marker["id"]
            # ID = np.array(ID,dtype=np.uint16)
            ID=[int(ID[j]) for j in range(len(ID))]

            Tvec = Marker["Translation Vector"]
            Tvec = np.array(Tvec,dtype=np.float32)
            Tvec = [float(Tvec[j]) for j in range(len(Tvec))]

            Eul = Marker["Euler Angle"]
            Eul = np.array(Eul,dtype=np.float32)
            Eul = [float(Eul[j]) for j in range(len(Eul))]

            Qua = Marker["Quaternion"]
            Qua = np.array(Qua,dtype=np.float32)
            Qua = [float(Qua[j]) for j in range(len(Qua))]

            msg.id = UInt16MultiArray()
            msg.id.data = ID

            msg.tvec = Float32MultiArray()
            msg.tvec.data = Tvec

            msg.eul = Float32MultiArray()
            msg.eul.data = Eul

            msg.qua = Float32MultiArray()
            msg.qua.data = Qua

            self.pub.publish(msg)

            self.get_logger().info("Publishing...")
            # self.get_logger().info(
            #     'Publishing: \nID:'+ str(msg.id.data) +
            #     "\nTranslation Vector: " +str(msg.tvec.data) + 
            #     "\nEuler Angle: " + str(msg.eul.data) + 
            #     "\nQuaternion: " + str(msg.qua.data)
            #     )
        except:
            self.get_logger().info("Error...")
            pass

def main(args=None):
    rclpy.init()
    camera_service = CameraSubsciber()
    rclpy.spin(camera_service)
    camera_service.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__" :
    main()