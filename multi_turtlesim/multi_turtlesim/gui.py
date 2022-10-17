#!/usr/bin/python3

import pygame
from ament_index_python.packages import get_package_share_directory
import os 
import numpy as np
import math,yaml

class GUI():
    def __init__(self):
        self.screen_width = 550
        self.screen_height = 550
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
        self.map_image.set_colorkey((255,255,255))
    
    def update(self,turtles,parcels):
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                self.isQuit = True
            
        # Fill the background with white
        self.screen.fill((69, 86, 255))
        cell_size = 50
        for i in range(self.screen_width//cell_size):
            pygame.draw.line(self.screen,(255,255,255),(0,cell_size*i),(self.screen_width,cell_size*i))
            pygame.draw.line(self.screen,(255,255,255),(cell_size*i,0),(cell_size*i,self.screen_width))
        multi_turtlesim_path = get_package_share_directory('multi_turtlesim')
        self.screen.blit(self.map_image,(0,0))
            
        img_parcel = pygame.image.load(os.path.join(multi_turtlesim_path,'images','parcel','parcel.png'))
        img_parcel = pygame.transform.scale(img_parcel, (20,20)).convert_alpha()        
        
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
                self.screen.blit(img_parcel,(parcel.position.x/5.0*250,self.screen_width-parcel.position.y/5.0*250))
                #pygame.draw.rect(self.screen,(255,0,0),pygame.Rect(parcel.position.x/5.0*250,self.screen_width-parcel.position.y/5.44445*250,20,20))
        # Flip the display
        pygame.display.flip()