from os import stat
from turtle import Screen
import pygame as pg
import yaml

TFont = ("Arial", 12,'bold')
TFont2 = ("Arial", 8,'bold')

class Drawing():
    def __init__(self): 
        self.TFont = TFont
        self.TFont2 = TFont2
        self.endloop= 0
        firebrick = (178,34,34)
        navy = (0,0,128)
        brown = (139,69,19)
        seagreen = (46,139,87)
        self.color = [firebrick,navy,seagreen,brown]
        self.copy_screen_list = []
        self.index_screen = 0
        self.depot_pos = []
        self.customer_pos =[]
        self.type_list =[]
        self.num_add_depot = 0
        self.num_add_customer = 0
        self.amount_depot=0
        self.amount_customer=0

    def initial_screen(self,w,h):
        pg.init()
        pg.font.init()
        self.width,self.height = w,h
        self.font = pg.font.SysFont("Arial",12)
        self.font2 =  pg.font.SysFont("Arial",8)
        self.screen = pg.display.set_mode((self.width,self.height))
        
        pg.display.set_caption('map drawer')

    def overlay_map(self,root_filename):
        with open(root_filename,'r') as f:
            yml_dict = yaml.safe_load(f)
        self.image_file = yml_dict.get('image')
        self.resolution = yml_dict.get('resolution')
        self.origin     = yml_dict.get('origin')
        bg = pg.image.load(self.image_file)    # load image from yaml to overlay on pygame
        bg = pg.transform.scale(bg,(self.width, self.height))
        rect = bg.get_rect()
        screen = self.screen
        screen.fill((255,255,255))
        rect = rect.move((0,0))
        self.screen.blit(bg,rect)
        self.copy_screen = self.screen.copy()
        self.copy_screen_list.append(self.copy_screen)
        pg.display.update()

    def customer_circle(self,pos_x,pos_y,radius):
        pg.draw.circle(self.screen, self.color[2],[pos_x, pos_y],radius)

    def depot_circle(self,pos_x,pos_y,radius):
        pg.draw.circle(self.screen, self.color[0],[pos_x, pos_y],radius)


    def buildtext(self,text,posx,posy):
        pos=(posx,posy)
        label = self.font.render(str(text),1,(0,0,0))
        self.screen.blit(self.font.render(str(text),True, (0 ,0 ,0)),pos)
        pg.display.update()
    
    def buildtext_num(self,text,posx,posy):
        pos=(posx,posy)
        label = self.font2.render(str(text),1,(0,0,0))
        self.screen.blit(self.font2.render(str(text),True, (0 ,0 ,0)),pos)
        pg.display.update()

    
    def line(self,x1,y1,x2,y2):
        pg.draw.line(self.screen, (255, 0, 0), (x1, y1), (x2, y2),width=2)
        pg.display.flip()

    
    def create_depot_customer(self,size,type):
        
        current_type = 0
        
        mouse = pg.mouse.get_pos()
        pg.display.update()
        for event in pg.event.get():
            if event.type == pg.QUIT:
                pg.quit()
                exit()
            elif event.type == pg.MOUSEBUTTONDOWN and event.button==1 and size!=0: #if left click ,Add node at mouse pos
                px = mouse[0]
                py = mouse[1]
                p = [px,py]
                if type==0:
                    self.index_screen+=1
                    self.num_add_depot+=1
                    self.depot_pos.append(p)
                    self.depot_circle(px,py,size)
                    self.copy_screen = self.screen.copy()
                    self.copy_screen_list.append(self.copy_screen)
                    self.buildtext_num(str(self.num_add_depot),px,py)
                    
                elif type==1:
                    self.index_screen+=1
                    self.num_add_customer+=1
                    self.customer_pos.append(p)
                    self.customer_circle(px,py,size)
                    self.copy_screen = self.screen.copy()
                    self.copy_screen_list.append(self.copy_screen)
                    self.buildtext_num(str(self.num_add_customer),px,py)
                    
                # print("depot : " + str(self.depot_pos) +str("\n")+ "customer : " + str(self.customer_pos))
                print(self.num_add_depot,self.num_add_customer)
                current_type = type
                self.type_list.append(current_type)
                self.amount_depot = int(len(self.depot_pos))
                self.amount_customer = int(len(self.customer_pos))

            elif event.type == pg.MOUSEBUTTONDOWN and event.button==3:  #if right click ,Undo one time
                if self.index_screen > 0:
                    self.index_screen-=1
                    del self.copy_screen_list[-1]
                    self.screen.blit(self.copy_screen_list[self.index_screen],(0,0))             
                else:
                    self.index_screen =0

                if len(self.type_list)>0:
                    if self.type_list[-1]==0:
                        del self.depot_pos[-1]
                        del self.type_list[-1]
                        self.num_add_depot-=1
                        self.amount_depot = int(len(self.depot_pos))
                        
                    elif self.type_list[-1]==1:
                        self.num_add_customer-=1
                        del self.customer_pos[-1]
                        del self.type_list[-1]
                        self.amount_customer = int(len(self.customer_pos))
                else:
                    print('-----------------This is original map-------------------')

                # print("depot : " + str(self.depot_pos) +str("\n")+ "customer : " + str(self.customer_pos))
            
    def quit(self,isQuit):
        if isQuit == 1 :
            return pg.quit()
        else: 
            pass