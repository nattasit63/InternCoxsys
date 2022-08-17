import rclpy
from rclpy.node import Node
import tkinter as tk
from tkinter import *
from tkinter import messagebox
import sys
from tkinter import Tk, font , filedialog
from tk_ import TK_ 
from PIL import ImageTk,Image
from drawing import Drawing
import cv2
import yaml


class GUI(Node):
    def __init__(self):
        super().__init__('gui')
        self._tk = TK_()
        self.draw = Drawing()

        #Initial Value
        self.mode = 0
        self.clear = 0
        self.select = 0
        self.isYAML = 0
        self.node_size_val = 0
        self.node_type_val = 0
        self.timer_period = 300
        self.pg_quit = 0
        #Initial screen
        screen_width = 1376
        self.w = screen_width
        screen_height = 750
        self.h = screen_height
        self.root = Tk()
        self.root.title('Create your own system')
        self.TFont = ("Times New Roman", 15)
        self.bg_color = "dim gray"
        #screen visualize
        self.root.geometry(str(screen_width)+'x'+str(screen_height))
        self.root['background']=self.bg_color
        self.root.eval('tk::PlaceWindow . center')
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing) # To Immediately close program


        self.current_node_size = tk.DoubleVar()
        self.switch_var = IntVar()
 

    def select_file_path(self):
        my_path_default = "$HOME/interface_ws/src/full_interface/config"
        self.root_filename = filedialog.askopenfilename(initialdir=my_path_default,title='Select .yaml file',filetypes=(("yaml file","*.yaml"),("all files","*.*")))
        print("Path to File : ",self.root_filename)
        self.select =1
        if(self.root_filename[-4:]=='yaml'):
            with open(self.root_filename,'r') as f:
                yml_dict = yaml.safe_load(f)
            self.image_file = yml_dict.get('image')

            img = cv2.imread(self.image_file, cv2.IMREAD_UNCHANGED)
            self.pg_height = img.shape[0]
            self.pg_width = img.shape[1]
            self.isYAML=1
        else:
            img = cv2.imread(self.root_filename, cv2.IMREAD_UNCHANGED)
            self.pg_height = img.shape[0]
            self.pg_width = img.shape[1]
            self.isYAML=0
            # print(self.pg_height,self.pg_width)
        return self.root_filename

    def get_node_size_value(self,event):
        if event ==None or event==0 or event=='0':
            event=0
            self.node_size_val=5
        self.node_size_val=event
        return self.node_size_val

    def switcher_depot_customer(self,posx,posy):
        Radiobutton(self.root, text = 'Depot', variable = self.switch_var,value = 0,bg=self.bg_color,fg="light grey",activeforeground='black',activebackground='lightgrey',command=self.get_switch).place(x=posx,y=posy)
        Radiobutton(self.root, text = 'Customer', variable = self.switch_var,value = 1,bg=self.bg_color,fg="light grey",activeforeground='black',activebackground='lightgrey',command=self.get_switch).place(x=posx,y=posy+50)  

    def get_switch(self):
        self.node_type_val = self.switch_var.get()
        return self.node_type_val
    
    def on_closing(self):
        self.root.destroy()
        sys.exit()

    def on_closing2(self):
        self.map_scr.destroy()
        # sys.exit()

    def OK_step2(self):
        self.pg_quit = 0
        # self.pg_quit = 1
        self.mode=12
        self.draw.quit(self.pg_quit)
  
        return self.pg_quit

    def update(self):
        w,h = self.w,self.h
        if self.mode==0:
            self._tk.create_txt_pos(self._tk.intro_txt,w/68.8,h/37.5,"light grey",self.bg_color)
            self._tk.create_txt_pos(self._tk.intro_txt2,w/68.8,h/37.5 + h/18.75,"light grey",self.bg_color)
            self._tk.create_txt_pos(self._tk.intro_txt3,w/68.8,h/37.5 + h/18.75 +h/15,"light grey",self.bg_color)
            self._tk.create_btn(self.root,'Create file',int(h/150),int(w/91.73),self._tk.create_file_btn,w/2.29,h/2)
            self._tk.create_btn(self.root,'Import file',int(h/150 ),int(w/91.73),self._tk.import_file_btn,w/2.29,h/2+h/3.75)      
            self.mode = self._tk.mode
        elif self.mode == 1 :
            if self.clear==0:
                self._tk.clear_screen(self.root)
                self.clear=1
                self._tk.create_txt_pos('STEP 1 : ',w/68.8,h/37.5 +20 ,"light grey",self.bg_color)
            else:
                self._tk.create_btn(self.root,'Import map',int(h/375),int(w/137.6),self.select_file_path,w/68.8 + w/13.76,h/37.5)
                if self.select==1:
                    self._tk.create_txt_pos(self.root_filename,w/68.8+250,h/37.5+20  ,"light grey",self.bg_color)
                   
                    if self.isYAML==1:
                        self.draw.initial_screen(self.pg_width,self.pg_height)
                        self.draw.overlay_map(self.root_filename)
                        self._tk.disable_btn(self.root,'Import map',int(h/375),int(w/137.6),self.select_file_path,w/68.8 + w/13.76,h/37.5)
                        self.mode = 10
            
        elif self.mode==10: # add depot
            self._tk.create_txt_pos('  adjust your node size  ->',w/68.8 + w/13.76,h/37.5+60   ,"light grey",self.bg_color)
            self._tk.create_txt_pos('STEP 2 : ',w/68.8,h/37.5+60 ,"light grey",self.bg_color)
            self._tk.info(self._tk.info_txt,w/68.8+850,h/37.5+60 ,"light grey",self.bg_color)
            self._tk.info(self._tk.info_txt2,w/68.8+850,h/37.5+60+30 ,"light grey",self.bg_color)
            self._tk.info(self._tk.info_txt3,w/68.8+850,h/37.5+60+50 ,"light grey",self.bg_color)
            self._tk.info(self._tk.info_txt4,w/68.8+850,h/37.5+60+70 ,"light grey",self.bg_color)
            self._tk.info(self._tk.info_txt5,w/68.8+850,h/37.5+60+90 ,"light grey",self.bg_color)
            self.timer_period = 150
            self.mode=11

        elif self.mode ==11:
            node_size_slider =Scale(self.root, from_=0, to=30, orient=HORIZONTAL,length=300,tickinterval=5,variable=self.current_node_size,command=self.get_node_size_value)
            node_size_slider.place(x=w/68.8+350 ,y=h/37.5+60 )
            self.ok_button = tk.Button(self.root, text='OK',height= 1, width=3,command=self.OK_step2,fg='white',bg='DarkOliveGreen4').place(x=w/68.8+350+200,y=h/37.5+130 +50)
            self._tk.create_txt_pos('  select type of node  ->',w/68.8 + w/13.76,h/37.5+130  ,"light grey",self.bg_color)
            self.switcher_depot_customer(w/68.8+350,h/37.5+130)       
            self.draw.create_depot_customer(int(self.node_size_val),int(self.node_type_val))

        elif self.mode ==12:
            self.ok_button = tk.Button(self.root, text='OK',height= 1, width=3,command=self.OK_step2,fg='white',bg='DarkOliveGreen4',state=DISABLED).place(x=w/68.8+350+200,y=h/37.5+130 +50)
            self._tk.create_txt_pos('STEP 3 : ',w/68.8,h/37.5+180+40 ,"light grey",self.bg_color)
            self.timer_period = 300
            self._tk.create_txt_pos('Total Depot    : ' + str(self.draw.amount_depot),w/68.8+w/13.76,h/37.5+180+40 ,"light grey",self.bg_color)
            self._tk.create_txt_pos('Total Customer : ' + str(self.draw.amount_customer),w/68.8+w/13.76,h/37.5+180+40+40 ,"light grey",self.bg_color)
            self.mode=13
            # print(self.draw.amount_depot,self.draw.amount_customer)
        
        elif self.mode == 13 : 
            self._tk.create_btn(self.root,'Input Data',int(2),int(6),self._tk.info_screen,w/2,h/2)




        self.root.after(self.timer_period,self.update)
        
    

def main(args=None):
    rclpy.init(args=args)
    gui = GUI()
    gui.update()
    gui.root.mainloop()
    rclpy.spin(gui)
    gui.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':

    main()