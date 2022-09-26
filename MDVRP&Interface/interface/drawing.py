import pygame as pg
import yaml
from nwx import NX
# from traffic import Traffic_Management
TFont = ("Arial", 12,'bold')
TFont2 = ("Arial", 8,'bold')

class Drawing():

    def __init__(self): 
        self.nw = NX()
        # self.traffic = Traffic_Management()
        self.TFont = TFont
        self.TFont2 = TFont2
        self.endloop= 0
        firebrick = (178,34,34)
        navy = (0,0,128)
        brown = (139,69,19)
        seagreen = (46,139,87)
        aquamarine3 = (102,205,170)
        azure4 = (131,139,139)
        blue2 = (0,0,238)
        burlywood4 = (139,115,85)
        chartreuse3= (102,205,0)
        coral3 = (205,91,69)
        aliceblue	= (240,248,255)	 
        antiquewhite	= (250,235,215)	 
        antiquewhite1	=	  (255,239,219)	 
        aquamarine3	=  (102,205,170)	  
        blue2	=  (0,0,238)	 
        brown	=  (165,42,42)	 
        burlywood4	=	  (139,115,85)	 
        cobalt	=	  (61,89,171)	 
        cobaltgreen	=	  (61,145,64)	 
        coldgrey	=	  (128,138,135)	 
        coral	=	  (255,127,80)	 
        coral3	=	  (205,91,69)	 
        coral4	=	  (139,62,47)	 
        cornflowerblue	=	  (100,149,237)	 
        cornsilk1	=	  (255,248,220)	 
        cornsilk2	=	  (238,232,205)	 
        cornsilk3	=	  (205,200,177)	 
        cornsilk4	=	  (139,136,120)	 
        crimson	=	  (220,20,60)
        self.color = [firebrick,navy,seagreen,brown,aquamarine3,azure4,blue2,burlywood4,chartreuse3,coral3,aliceblue,antiquewhite,antiquewhite1,crimson,cornsilk4,cornsilk3,cornsilk2,cornsilk1,cornflowerblue,coral4
                      , coldgrey, coral,cobaltgreen,cobalt]
        self.copy_screen_list = []
        self.screen_edge=[]
        self.screen_edge_list=[]
        self.index_screen = 0
        self.depot_pos = []
        self.customer_pos =[]
        self.connect_pos = []
        self.customer_node_list = []
        self.type_list =[]
        self.num_add_depot = 0
        self.num_add_customer = 0
        self.num_add_connect_point= 0
        self.amount_depot=0
        self.amount_customer=0
        self.state = 0
        self.click_edge_state = 0
        self.first_edge_pair = 0
        self.second_edge_pair = 0
        self.first_edge_x = 0
        self.first_edge_y = 0
        self.second_edge_x = 0
        self.second_edge_y = 0
        self.edge_list = []
        self.index_edge_screen = 0
        self.get_into_state = 0
        self.all_via_point = []
        self.all_via_point_pos = [] 
        self.num_all_vp = 0
        self.index_customer_in_all_vp = []
        self.index_connect_point_in_all_vp = []
        self.index_depot_in_all_vp = []

    def initial_screen(self,w,h):
        pg.init()
        pg.font.init()
        self.width,self.height = 800,600
        self.font = pg.font.SysFont("Arial",12)
        self.font2 =  pg.font.SysFont("Arial",8)
        self.screen = pg.display.set_mode((self.width,self.height))
        # self.screen = pg.display.set_mode((800,600))
        
        pg.display.set_caption('map drawer (press ENTER to add edge)')

    def overlay_map(self,root_filename,type):
        # self.traffic.print_img(root_filename)
        if type=='yaml':
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
            self.copy_screen_list=[]
            self.copy_screen_list.append(self.copy_screen)
            pg.display.update() 
        elif type=='img':
            bg = pg.image.load(root_filename)    # load image from yaml to overlay on pygame
            bg = pg.transform.scale(bg,(self.width, self.height))
            rect = bg.get_rect()
            screen = self.screen
            screen.fill((255,255,255))
            rect = rect.move((0,0))
            self.screen.blit(bg,rect)
            self.copy_screen = self.screen.copy()
            self.copy_screen_list=[]
            self.copy_screen_list.append(self.copy_screen)
            pg.display.update()
        

    def customer_circle(self,pos_x,pos_y,radius):
        pg.draw.circle(self.screen, self.color[2],[pos_x, pos_y],radius)

    def depot_circle(self,pos_x,pos_y,radius):
        pg.draw.circle(self.screen, self.color[0],[pos_x, pos_y],radius)

    def connect_point_circle(self,pos_x,pos_y,radius):
        pg.draw.circle(self.screen, self.color[4],[pos_x, pos_y],radius)

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
        pg.draw.line(self.screen, (0, 0, 0), (x1, y1), (x2, y2),width=2)
        pg.display.flip()
    
    def connect_point(self,current,next,color):
        pg.draw.line(self.screen, color, (current[0], current[1]), (next[0], next[1]),width=2)
        pg.display.flip()
    
    def clear_variable(self):
        self.copy_screen_list = []
        self.screen_edge=[]
        self.screen_edge_list=[]
        self.index_screen = 0
        self.all_via_point = []
        self.all_via_point_pos = [] 
        self.depot_pos = []
        self.customer_pos =[]
        self.connect_pos =[]
        self.type_list =[]
        self.num_add_depot = 0
        self.num_add_customer = 0
        self.num_add_connect_point= 0
        self.amount_depot=0
        self.amount_customer=0
        self.state = 0
        self.click_edge_state = 0
        self.first_edge_pair = 0
        self.second_edge_pair = 0
        self.first_edge_x = 0
        self.first_edge_y = 0
        self.second_edge_x = 0
        self.second_edge_y = 0
        self.edge_list = []
        self.index_edge_screen = 0
        self.customer_node_list = []
    
    def create_depot_customer(self,size,type):
        global amount_customer
        global amount_depot
        global amount_all_vp
        global customer_index
        global connect_index
        current_type = 0
        mouse = pg.mouse.get_pos()
        pg.display.update()
        if self.state==0:
            for event in pg.event.get():
                if event.type == pg.QUIT:
                    pg.quit()
                    exit()
                if event.type == pg.KEYDOWN:
                    if event.key == pg.K_RETURN :
                        self.state = 1
                        self.screen_edge = self.screen.copy()
                        self.screen_edge_list.append(self.screen_edge )
                        self.before_edge_screen = self.screen.copy()
                        print('Click node to pair')
                        print('Index depot  :  ',self.index_depot_in_all_vp)
                        print('Index customer  :  ',self.index_customer_in_all_vp)
                        print('Index connect point  :  ',self.index_connect_point_in_all_vp)
                elif event.type == pg.MOUSEBUTTONDOWN and event.button==1 and size!=0: #if left click ,Add node at mouse pos
                    px = mouse[0]
                    py = mouse[1]
                    p = [px,py]
                    if type==0:  #depot
                        self.index_screen+=1
                        self.num_add_depot+=1
                        self.depot_pos.append(p)
                        self.depot_circle(px,py,size)

                        self.num_all_vp+=1
                        self.index_depot_in_all_vp.append(self.num_all_vp)
                        self.all_via_point.append(self.num_all_vp)
                        self.all_via_point_pos.append(p)

                        self.buildtext_num(str(self.num_add_depot),px,py)
                        self.copy_screen = self.screen.copy()
                        self.copy_screen_list.append(self.copy_screen)
                        
                    elif type==1:  #customer
                        self.index_screen+=1
                        self.num_add_customer+=1
                        self.customer_pos.append(p)
                        self.customer_circle(px,py,size)

                        self.num_all_vp+=1
                        self.index_customer_in_all_vp.append(self.num_all_vp     )
                        self.all_via_point.append(self.num_all_vp)
                        self.all_via_point_pos.append(p)

                        self.buildtext_num(str(self.num_add_customer),px,py)
                        self.copy_screen = self.screen.copy()
                        self.copy_screen_list.append(self.copy_screen)
                        
                        
                    
                    elif type==2 : #connect point
               
                        self.index_screen+=1
                        self.num_add_connect_point+=1
                        self.connect_pos.append(p)
                        self.connect_point_circle(px,py,size)

                        self.num_all_vp+=1
                        self.index_connect_point_in_all_vp.append(self.num_all_vp)
                        self.all_via_point.append(self.num_all_vp)
                        self.all_via_point_pos.append(p)

                        self.buildtext_num(str(self.num_add_connect_point),px,py)
                        self.copy_screen = self.screen.copy()
                        self.copy_screen_list.append(self.copy_screen)

                        

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

                            self.num_all_vp-=1
                            del self.index_depot_in_all_vp[-1]
                            del self.all_via_point[-1]
                            del self.all_via_point_pos[-1]
                            
                        elif self.type_list[-1]==1:
                            self.num_add_customer-=1
                            del self.customer_pos[-1]
                            del self.type_list[-1]
                            self.amount_customer = int(len(self.customer_pos))

                            self.num_all_vp-=1
                            del self.index_customer_in_all_vp[-1]
                            del self.all_via_point[-1]
                            del self.all_via_point_pos[-1]

                   

                        elif self.type_list[-1]==2:
                            self.num_add_connect_point-=1
                            del self.connect_pos[-1]
                            del self.type_list[-1]

                            self.num_all_vp-=1
                            del self.index_connect_point_in_all_vp[-1]
                            del self.all_via_point[-1]
                            del self.all_via_point_pos[-1]



                    else:
                        print('-----------------This is original map-------------------')
            
                
                # print('STATE 0 = ',len(self.customer_pos))
        elif self.state == 1 :
            pg.display.set_caption('Click on node to pair your edge')
            mouse = pg.mouse.get_pos()
            r = size 
            for event in pg.event.get():
                if event.type == pg.QUIT:
                    pg.quit()
                    exit()
                if event.type == pg.MOUSEBUTTONDOWN and event.button==3:  #if right click ,Undo one time
                    if self.index_edge_screen > 0:         
                        self.index_edge_screen-=1
                        del self.screen_edge_list[-1]
                        del self.edge_list[-1]
                        try:
                            self.screen.blit(self.screen_edge_list[self.index_edge_screen],(0,0)) 
                        except:
                            print('----------------- NOT BUILD ANY MORE -------------------')
                        
                    else:
                        self.index_edge_screen =0
                        print('----------------- This is Original -------------------')

                if self.click_edge_state==0:
                    if event.type == pg.MOUSEBUTTONDOWN and event.button==1: #if left click ,Select pair
                        for i in range(0,len(self.all_via_point_pos)):
                            if self.all_via_point_pos[i][0]-r<=mouse[0]<=self.all_via_point_pos[i][0]+r and self.all_via_point_pos[i][1]-r<=mouse[1]<=self.all_via_point_pos[i][1]+r :
                                self.first_edge_pair = i+1
                                self.first_edge_x = self.all_via_point_pos[i][0]
                                self.first_edge_y = self.all_via_point_pos[i][1]
    
                                self.click_edge_state=1
              

                elif self.click_edge_state==1:
                    if event.type == pg.MOUSEBUTTONDOWN and event.button==1: #if left click , paired
                        for i in range(0,len(self.all_via_point_pos)):
                            if self.all_via_point_pos[i][0]-r<=mouse[0]<=self.all_via_point_pos[i][0]+r and self.all_via_point_pos[i][1]-r<=mouse[1]<=self.all_via_point_pos[i][1]+r :
                                self.second_edge_pair = i+1
                                second_edge_x = self.all_via_point_pos[i][0]
                                second_edge_y = self.all_via_point_pos[i][1]
                                if self.first_edge_pair!=self.second_edge_pair:
                                    #draw and add edge to list
                                    self.line(self.first_edge_x,self.first_edge_y,second_edge_x,second_edge_y)
                                    self.edge_list.append([self.first_edge_pair,self.second_edge_pair])
                                    # self.edge_list.append([self.second_edge_pair,self.first_edge_pair])
                                    self.screen_edge = self.screen.copy()
                                    self.screen_edge_list.append(self.screen_edge)
                                    self.click_edge_state=0
                                    self.index_edge_screen +=1
                                    print(self.edge_list)
        ''''         
        elif self.state == 1 :
            pg.display.set_caption('Click on node to pair your edge')
            mouse = pg.mouse.get_pos()
            r = size 
            for event in pg.event.get():
                if event.type == pg.QUIT:
                    pg.quit()
                    exit()
                if event.type == pg.MOUSEBUTTONDOWN and event.button==3:  #if right click ,Undo one time
                    if self.index_edge_screen > 0:         
                        self.index_edge_screen-=1
                        del self.screen_edge_list[-1]
                        del self.edge_list[-1]
                        try:
                            self.screen.blit(self.screen_edge_list[self.index_edge_screen],(0,0)) 
                        except:
                            print('----------------- NOT BUILD ANY MORE -------------------')
                        
                    else:
                        self.index_edge_screen =0
                        print('----------------- This is Original -------------------')

                if self.click_edge_state==0:
                    if event.type == pg.MOUSEBUTTONDOWN and event.button==1: #if left click ,Select pair
                        for i in range(0,len(self.customer_pos)):
                            if self.customer_pos[i][0]-r<=mouse[0]<=self.customer_pos[i][0]+r and self.customer_pos[i][1]-r<=mouse[1]<=self.customer_pos[i][1]+r :
                                self.first_edge_pair = i+1
                                self.first_edge_x = self.customer_pos[i][0]
                                self.first_edge_y = self.customer_pos[i][1]
    
                                self.click_edge_state=1
              

                elif self.click_edge_state==1:
                    if event.type == pg.MOUSEBUTTONDOWN and event.button==1: #if left click , paired
                        for i in range(0,len(self.customer_pos)):
                            if self.customer_pos[i][0]-r<=mouse[0]<=self.customer_pos[i][0]+r and self.customer_pos[i][1]-r<=mouse[1]<=self.customer_pos[i][1]+r :
                                self.second_edge_pair = i+1
                                second_edge_x = self.customer_pos[i][0]
                                second_edge_y = self.customer_pos[i][1]
                                if self.first_edge_pair!=self.second_edge_pair:
                                    #draw and add edge to list
                                    self.line(self.first_edge_x,self.first_edge_y,second_edge_x,second_edge_y)
                                    self.edge_list.append([self.first_edge_pair,self.second_edge_pair])
                                    self.screen_edge = self.screen.copy()
                                    self.screen_edge_list.append(self.scr en_edge)
                                    self.click_edge_state=0
                                    self.index_edge_screen +=1
                                    print(self.edge_list)
            
        '''      


        self.original_create = self.screen.copy()
        amount_customer =  self.amount_customer
        amount_depot   = self.amount_depot
        amount_all_vp  = self.num_all_vp
        customer_index = self.index_customer_in_all_vp
        connect_index = self.index_connect_point_in_all_vp

    def send_to_nwx(self):
        self.screen.blit(self.before_edge_screen,(0,0))
        self.major_matrix = self.nw.do_dist_matrix(list_of_node=self.all_via_point,list_of_edge=self.edge_list
                                                    ,pos_of_node=self.all_via_point_pos
                                                    ,customer_index_list=self.index_customer_in_all_vp
                                                    ,connect_point_list=self.index_connect_point_in_all_vp
                                                    ,depot_index=self.index_depot_in_all_vp)
        return self.major_matrix

    def back_to_original(self):
        self.screen.blit(self.original_create,(0,0))
        pg.display.flip()

    def visual(self,solution):
        self.true_route = []
        sol = solution.split('\n')
        sol = sol[:-1]

        for i in range(len(sol)):
            sol[i] = sol[i].split('\t')
            del sol[i][1]
            del sol[i][1]
            del sol[i][1]
            sol[i][0] = int(sol[i][0])
            sol[i][1] =sol[i][1].split()

        for j in range(len(sol)):
            depot = sol[j][0]
            for point in range(len(sol[j][1])):
                if sol[j][1][point] == '0':
                    sol[j][1][point]=depot
            self.true_route.append(sol[j][1])

        
        for i in range(len(self.true_route)):
            for j in range(len(self.true_route[i])):
                check_depot = isinstance(self.true_route[i][j],int)
                if check_depot:
                    self.true_route[i][j] = self.depot_pos[int(self.true_route[i][j])-1]
                else:
                    self.true_route[i][j] = self.customer_pos[int(self.true_route[i][j])-1]

        for i in range(len(self.true_route)):
            colors = self.color[i]
            for j in range(len(self.true_route[i])-1):
                current = self.true_route[i][j]
                next = self.true_route[i][j+1]
                self.connect_point(current,next,colors)
        return self.true_route



    # def visual(self,solution):  #for adj_matrix
        
    #     self.true_route = []
    #     sol = solution.split('\n')

    #     sol = sol[:-1]


    #     # print(sol)
    #     for i in range(len(sol)):
    #         sol[i] = sol[i].split('\t')
    #         del sol[i][1]
    #         del sol[i][1]
    #         del sol[i][1]
    #         sol[i][0] = int(sol[i][0])
    #         sol[i][1] =sol[i][1].split()
    #     # print('sol from loop [i] :',sol)

    #     for j in range(len(sol)):
    #         depot = sol[j][0]
    #         for point in range(len(sol[j][1])):
    #             if sol[j][1][point] == '0':
    #                 sol[j][1][point]=depot
    #         self.true_route.append(sol[j][1])
    #         # print('true route  = ',self.true_route)

    #     print('true_route from loop [j] :',self.true_route)
        


    #     for i in range(len(self.true_route)):
    #         for j in range(len(self.true_route[i])):
    #             check_depot = isinstance(self.true_route[i][j],int)
    #             if check_depot:
    #                 self.true_route[i][j] = self.depot_pos[int(self.true_route[i][j])-1]
    #             else:
    #                 self.true_route[i][j] = self.all_via_point_pos[int(self.true_route[i][j])-1]

        
    #     for i in range(len(self.true_route)):
    #         colors = self.color[i]
    #         for j in range(len(self.true_route[i])-1):
    #             current = self.true_route[i][j]
    #             next = self.true_route[i][j+1]
    #             self.connect_point(current,next,colors)
    #             # print(color_pos = (current,next))
        
    #     return self.true_route

    def visual_astar(self,solution):
        self.true_route =solution
        for i in range(len(self.true_route)):
            for j in range(len(self.true_route[i])):
                check_depot = isinstance(self.true_route[i][j],int)
                if check_depot:
                    self.true_route[i][j] = self.depot_pos[int(self.true_route[i][j])-1]
                else:
                    self.true_route[i][j] = self.all_via_point_pos[int(self.true_route[i][j])-1+amount_depot] 

        print('POS IN A*ROUTE = ',self.true_route)
        for i in range(len(self.true_route)):
            colors = self.color[i]
            for j in range(len(self.true_route[i])-1):
                current = self.true_route[i][j]
                next = self.true_route[i][j+1]
                self.connect_point(current,next,colors)
        
      
    def quit(self,isQuit):
        if isQuit == 1 :
            return pg.quit()
        else: 
            pass
    
 