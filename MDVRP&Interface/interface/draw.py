import pygame as pg
from select_file import Select

select = Select()

class Draw():
    def __init__(self):
        pg.init()
        pg.font.init()
        self.width = 449
        self.height = 501
        self.screen = pg.display.set_mode((self.width,self.height))
        pg.display.set_caption('gui')

        self.update = pg.display.update()
        self.font = pg.font.SysFont("Cordia New",self.height//30)
        firebrick = (178,34,34)
        navy = (0,0,128)
        brown = (139,69,19)
        seagreen = (46,139,87)
        self.color = [firebrick,navy,seagreen,brown]
    def circle(self,pos_x,pos_y,radius):
        pg.draw.circle(self.screen, (0, 200, 0),[pos_x, pos_y],radius)
        self.update
    def buildtext(self,text,posx,posy):
        pos=(posx,posy)
        self.font.render(str(text),1,(0,0,0))
        self.screen.blit(self.font.render(str(text),True, (0 ,0 ,0)),pos)
        pg.display.update()
    
    def line(self,x1,y1,x2,y2):
        pg.draw.line(self.screen, (255, 0, 0), (x1, y1), (x2, y2),width=2)
        pg.display.flip()
    

    def check_close(self):
        pg.display.update()
        for event in pg.event.get():
            if event.type == pg.QUIT:
                return pg.quit(),exit()
    
