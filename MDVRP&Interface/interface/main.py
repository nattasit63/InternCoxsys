
import rclpy
from rclpy.node import Node
import tkinter as tk
from tkinter import *
from tkinter import messagebox
import tkinter.messagebox
import sys
from tkinter import Tk, font , filedialog
import tk_
from tk_ import TK_ 
import drawing
from drawing import Drawing
from write_file import Write_file
from solver import Solver
from nwx import NX
import nwx

import cv2
import yaml



class GUI(Node):
    def __init__(self):
        super().__init__('gui')
        self._tk = TK_()
        self.draw = Drawing()
        self.write_file = Write_file()
        self.solve = Solver()
        self.nw = NX()
        #Initial Value
        self.mode = 0
        self.clear = 0
        self.select = 0
        self.isYAML = 0
        self.isIMG = 0
        self.is_calculate = 0
        self.node_size_val = 0
        self.node_type_val = 0
        self.timer_period = 300
        self.pg_quit = 0
        self.slide_set = 0
        self.info_to_write = []
        self.type_file = ''
        self.write_file_path = ''
        self.cost = 0
        self.sol = ''
        #Initial screen
        screen_width = 1200
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
        my_path_default = "/home/natta/interface_ws/src/full_interface/config/"
        self.root_filename = filedialog.askopenfilename(initialdir=my_path_default,title='Select .yaml file',filetypes=(("yaml file","*.yaml"),("pgm file","*.pgm"),("all files","*.*"),("png file","*.png")))
        print("Path to File : ",self.root_filename)
        self.select =1
        if self.root_filename.endswith('.yaml'):
                with open(self.root_filename,'r') as f:
                    yml_dict = yaml.safe_load(f)
                self.image_file = yml_dict.get('image')
                print(self.image_file )
                img = cv2.imread(self.image_file, cv2.IMREAD_UNCHANGED)
                try:
                    self.pg_height = img.shape[0]
                    self.pg_width = img.shape[1]
                except:
                    print('Your map dimension is too small ! \nTry : directly import from image file')
                    return sys.exit()
                self.isYAML=1
                self.type_file = 'yaml'
          
        elif self.root_filename.endswith('.png') or self.root_filename.endswith('.pgm'):
            img = cv2.imread(self.root_filename, cv2.IMREAD_UNCHANGED)
            self.pg_height = img.shape[0]
            self.pg_width = img.shape[1]
            self.isYAML=0
            self.isIMG =1
            self.type_file = 'img'
        return self.root_filename
    
    def select_config(self):
        my_path_default = "/home/natta/interface_ws/src/full_interface/data"
        self.root_filename = filedialog.askopenfilename(initialdir=my_path_default,title='Select Config file',filetypes=(("txt file","*.txt"),("all files","*.*")))
        print("Path to File : ",self.root_filename)
        if self.root_filename!=0 and self.root_filename!='' and self.root_filename!=None and self.root_filename!=() and self.root_filename!='()':
            self.select=1      
        else:
            print('Error Import')

    
    def calculate(self):      
        self.solve3 = Solver()
        self.solve3.insert_file_path(self.root_filename)
        self.cost,self.sol = self.solve3.run()
        
        satisfied = messagebox.askquestion('Confirmation','Are you satisfied this result ?\n"Yes" : confirm result\n"No"  : cancel')
        if satisfied == "yes":
            self.is_calculate = 1
        elif satisfied == "no":
            self.is_calculate = 0
        
    
    def save_solution(self):
        try:
            self.write_file.write_sol(self.cost,self.sol)
            tkinter.messagebox.showinfo('','Solution Saved !')
            self.mode = 22
        except:
            pass

    def save_solution2(self):
        try:
            self.write_file.write_sol(self.cost,self.sol)
            tkinter.messagebox.showinfo('','Solution Saved !')
            self.mode = 17  
        except:
            pass
    
    def get_node_size_value(self,event):
        if event ==None or event==0 or event=='0':
            event=0
            self.node_size_val=5
        self.node_size_val=event
        return self.node_size_val

    def switcher_depot_customer(self,posx,posy):
        Radiobutton(self.root, text = 'Depot', variable = self.switch_var,value = 0,bg=self.bg_color,fg="light grey",activeforeground='black',activebackground='lightgrey',command=self.get_switch).place(x=posx,y=posy)
        Radiobutton(self.root, text = 'Customer', variable = self.switch_var,value = 1,bg=self.bg_color,fg="light grey",activeforeground='black',activebackground='lightgrey',command=self.get_switch).place(x=posx,y=posy+25)  
        Radiobutton(self.root, text = 'Connect point', variable = self.switch_var,value = 2,bg=self.bg_color,fg="light grey",activeforeground='black',activebackground='lightgrey',command=self.get_switch).place(x=posx,y=posy+50) 

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
        self.draw.send_to_nwx()
        # self.draw.send_to_nwx()

        # self.nw.do_real_path()

        self.mode=12
        self.draw.quit(self.pg_quit)
  
        return self.pg_quit
    
    def re_calculate(self):
 
        self.solve2 = Solver()
        self.solve2.insert_file_path(self.write_file_path)
        self.draw.back_to_original()
        best_cost,solution = self.solve2.run()
        tkinter.messagebox.showinfo('Calculation','Done . . .!')
        self.draw.visual(solution)

    def home(self):
        try:
            self.draw.clear_variable()
            try:
                self._tk.clear_screen(self.root)
            except:
                pass
            self.root_filename = ''
            self.mode = 0
            self.clear = 0
            self.select = 0
            self.isYAML = 0
            self.is_calculate = 0
            self.node_size_val = 0
            self.node_type_val = 0
            self.timer_period = 300
            self.pg_quit = 0
            self.info_to_write = []
            self.write_file_path = ''
            self.cost = 0
            self.slide_set = 0
            self.sol = ''
            self.draw.amount_customer = 0
            self.draw.amount_depot = 0
            self.draw.endloop = 0
            self.draw.copy_screen_list = []
            self.draw.index_screen = 0
            self.draw.depot_pos = []
            self.draw.customer_pos =[]
            self.draw.type_list =[]
            self.draw.num_add_depot = 0
            self.draw.num_add_customer = 0
            self.draw.quit(1)
            try:
                self._tk.info_root.destroy()
            except:
                pass
            self._tk = TK_()
            self.draw = Drawing()
            self.write_file = Write_file()
            self.solve = Solver()
        except:
            pass

    def exit(self):
        self.root.destroy()
        sys.exit()
        
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
                self._tk.create_txt_pos('STEP 1 : ',w/68.8,h/37.5 +h/37.5 ,"light grey",self.bg_color)
            else:
                self._tk.create_btn(self.root,'Import map',int(h/375),int(w/137.6),self.select_file_path,w/68.8 + w/13.76,h/37.5)
                if self.select==1:
                    self._tk.create_txt_pos(self.root_filename,w/68.8+250,h/37.5 +h/37.5  ,"light grey",self.bg_color)
                   
                    if self.isYAML==1 or self.isIMG==1:
                        self.draw.initial_screen(self.pg_width,self.pg_height)
                        self.draw.overlay_map(self.root_filename,self.type_file)
                        self._tk.disable_btn(self.root,'Import map',int(h/375),int(w/137.6),self.select_file_path,w/68.8 + w/13.76,h/37.5)
                        self.mode = 10
                     
        elif self.mode==10: # add depot
            self._tk.create_txt_pos('  adjust your node size  ->',w/68.8 + w/13.76,h/37.5+h/12.5   ,"light grey",self.bg_color)
            self._tk.create_txt_pos('STEP 2 : ',w/68.8,h/37.5+h/12.5 ,"light grey",self.bg_color)
            self._tk.info(self._tk.info_txt,w/68.8+850,h/37.5+h/12.5 ,"light grey",self.bg_color)
            self._tk.info(self._tk.info_txt2,w/68.8+850,h/37.5+h/12.5+h/25 ,"light grey",self.bg_color)
            self._tk.info(self._tk.info_txt3,w/68.8+850,h/37.5+h/12.5+h/15 ,"light grey",self.bg_color)
            self._tk.info(self._tk.info_txt4,w/68.8+850,h/37.5+h/12.5+h/10.71428 ,"light grey",self.bg_color)
            self._tk.info(self._tk.info_txt5,w/68.8+850,h/37.5+h/12.5+h/8.3333 ,"light grey",self.bg_color)
            self.timer_period = 150
            self.node_size_val = 5
            self.mode=11

        elif self.mode ==11:
            self.node_size_slider =Scale(self.root, from_=0, to=30, orient=HORIZONTAL,length=w/4,tickinterval=5,variable=self.current_node_size,command=self.get_node_size_value)
            if self.slide_set==0:
                    self.node_size_slider.set(5)
                    self.slide_set=1
            self.node_size_slider.place(x=w/68.8+w/3.42857 ,y=h/37.5+h/12.5 )
            
            self.ok_button = tk.Button(self.root, text='OK',height= 1, width=3,command=self.OK_step2,fg='white',bg='DarkOliveGreen4').place(x=w/68.8+w/2.1818,y=h/37.5+h/4.16667)
            self._tk.create_txt_pos('  select type of node  ->',w/68.8+w/13.76,h/37.5+h/5.76923,"light grey",self.bg_color)
            self.switcher_depot_customer(w/68.8+350,h/37.5+h/5.76923)
            self.draw.create_depot_customer(int(self.node_size_val),int(self.node_type_val))

  
        elif self.mode ==12:
            self.ok_button = tk.Button(self.root, text='OK',height= 1, width=3,command=self.OK_step2,fg='white',bg='DarkOliveGreen4',state=DISABLED).place(x=w/68.8+w/2.181818,y=h/37.5 +h/4.16667)
            self._tk.create_txt_pos('STEP 3 : ',w/68.8,h/37.5+h/3.40909 ,"light grey",self.bg_color)
            self.timer_period = 300
            self._tk.create_txt_pos('Total Depot    : ' + str(self.draw.amount_depot),w/68.8+w/13.76,h/37.5+h/3.40909 ,"light grey",self.bg_color)
            self._tk.create_txt_pos('Total Customer : ' + str(self.draw.amount_customer),w/68.8+w/13.76,h/37.5+h/2.8846 ,"light grey",self.bg_color)
            
            self.mode=13

      
        elif self.mode == 13 :
            if self._tk.info_root_active==0:
                self._tk.create_btn(self.root,'Input Data',int(2),int(6),self._tk.info_screen,w/68.8+w/13.76,h/37.5+h/2.5)
  
            else:
                self._tk.disable_btn(self.root,'Input Data',int(2),int(6),self._tk.info_screen,w/68.8+w/13.76,h/37.5+h/2.5)
            
            if self._tk.mode == 14:             
                self.mode = 14
        
        elif self.mode == 14 :
            self.info_to_write = self._tk.entry_list
            pos = [self.draw.depot_pos,self.draw.customer_pos]
            self.write_file_path = self.write_file.write(tk_.export_data,drawing.amount_customer,drawing.amount_depot,pos)
            # pos = [self.draw.depot_pos,self.draw.all_via_point_pos]
            # self.write_file_path = self.write_file.write2(tk_.export_data,drawing.amount_all_vp,drawing.customer_index,drawing.amount_depot,pos,drawing.connect_index)
            self.mode = 15
        
        elif self.mode == 15 :
            self.solve.insert_file_path(self.write_file_path)
            best_cost,solution = self.solve.run()
            self.cost = best_cost
            self.sol = solution

            # self.nw.do_real_path(solution)
            # astar_route =  nwx.astar_path 
            print(self.nw.mapping_with_ui(solution))
            
            # self.draw.visual(solution)
            self.draw.visual_astar(self.nw.mapping_with_ui(solution))
          
            self.mode = 16
        
        elif self.mode == 16 :
            self._tk.create_txt_pos('STEP 4 : ',w/68.8,h/37.5+h/1.97368 ,"light grey",self.bg_color)
            self._tk.create_btn(self.root,'Save\nresult',int(h/375),int(w/137.6),self.save_solution2,w/68.8+w/13.76,h/37.5+h/1.97368)
            self._tk.create_btn(self.root,'Re-Calculate',int(h/375),int(w/137.6),self.re_calculate,w/68.8+w/13.76+w/12,h/37.5+h/1.97368)

        elif self.mode == 17 :
            self._tk.disable_btn(self.root,'Save\nresult',int(h/375),int(w/137.6),self.save_solution2,w/68.8+w/13.76,h/37.5+h/1.97368)
            self._tk.disable_btn(self.root,'Re-Calculate',int(h/375),int(w/137.6),self.re_calculate,w/68.8+w/13.76+w/12,h/37.5+h/1.97368)


        ###########################################  MODE IMPORT CONFIG #############################################################
        elif self.mode == 2 :
            if self.clear==0:
                self._tk.clear_screen(self.root)
                self.clear=1
                self._tk.create_txt_pos('STEP 1 : ',w/68.8,h/37.5+h/37.5 ,"light grey",self.bg_color)
            else:
                self._tk.create_btn(self.root,'Import Config',int(h/375),int(w/137.6),self.select_config,w/68.8 + w/13.76,h/37.5)
                if self.select==1 :
                    if self.root_filename!='' or self.root_filename!=() or self.root_filename!='()':
                        self._tk.create_txt_pos(self.root_filename,w/68.8+250,h/37.5+h/37.5  ,"light grey",self.bg_color)
                        self._tk.disable_btn(self.root,'Import Config',int(h/375),int(w/137.6),self.select_file_path,w/68.8 + w/13.76,h/37.5)
                        self._tk.create_txt_pos('STEP 2 : ',w/68.8,h/37.5+h/12.5 ,"light grey",self.bg_color)
                        self.mode = 20
                else:
                    self.mode = 2
        
        elif self.mode == 20 :           
            self._tk.create_btn(self.root,'Calculate',int(h/375),int(w/137.6),self.calculate,w/68.8 + w/13.76,h/37.5+h/12.5)
            if self.is_calculate==1:
                self._tk.disable_btn(self.root,'Calculate',int(h/375),int(w/137.6),self.calculate,w/68.8 + w/13.76,h/37.5+h/12.5)
                self._tk.create_txt_pos('STEP 3 : ',w/68.8,h/37.5+h/6.25 ,"light grey",self.bg_color)
                self.mode = 21
            else:
                self.mode = 20
        
        elif self.mode == 21:
            self._tk.create_btn(self.root,'Save',int(h/375),int(w/137.6),self.save_solution,w/68.8+ w/13.76,h/37.5+h/6.25)
        
        elif self.mode == 22:
            self._tk.disable_btn(self.root,'Save',int(h/375),int(w/137.6),self.save_solution,w/68.8+ w/13.76,h/37.5+h/6.25)
            self._tk.disable_btn(self.root,'Calculate',int(h/375),int(w/137.6),self.calculate,w/68.8 + w/13.76,h/37.5+h/12.5)

        

        if self.mode!=0:
            tk.Button(self.root, text='Home',height= 1, width=3,command=self.home,fg='white',bg='salmon').place(x=w/68.8,y=h/1.071428)
            tk.Button(self.root, text='exit',height= 1, width=3,command=self.exit,fg='white',bg='red').place(x=w/1.09,y=h/1.071428)
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