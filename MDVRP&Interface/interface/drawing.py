
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
        aquamarine3 = (102,205,170)
        azure4 = (131,139,139)
        blue2 = (0,0,238)
        burlywood4 = (139,115,85)
        chartreuse3= (102,205,0)
        coral3 = (205,91,69)
        self.color = [firebrick,navy,seagreen,brown,aquamarine3,azure4,blue2,burlywood4,chartreuse3, coral3]
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
        self.width,self.height = 800,600
        self.font = pg.font.SysFont("Arial",12)
        self.font2 =  pg.font.SysFont("Arial",8)
        self.screen = pg.display.set_mode((self.width,self.height))
        # self.screen = pg.display.set_mode((800,600))
        
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
        self.copy_screen_list=[]
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
    
    def connect_point(self,current,next,color):
        pg.draw.line(self.screen, color, (current[0], current[1]), (next[0], next[1]),width=2)
        pg.display.flip()
    
    def create_depot_customer(self,size,type):
        global amount_customer
        global amount_depot
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
                # print(self.num_add_depot,self.num_add_customer)
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
        self.original_create = self.screen.copy()
        amount_customer =  self.amount_customer
        amount_depot   = self.amount_depot
                # print("depot : " + str(self.depot_pos) +str("\n")+ "customer : " + str(self.customer_pos))
    
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
      
    def quit(self,isQuit):
        if isQuit == 1 :
            return pg.quit()
        else: 
            pass
    
 