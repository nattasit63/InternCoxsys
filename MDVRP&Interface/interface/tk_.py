import tkinter as tk
from tkinter import *
from tkinter import messagebox

from tkinter import Tk, font
from tkinter import filedialog
import os
from PIL import Image, ImageTk
TFont = ("Arial", 12,'bold')
TFont2 = ("Arial",10)
class TK_():
    def __init__(self):
        self.TFont = TFont
        self.bg_color = '#856ff8'
        self.intro_txt = "This is a program to do fleet & traffic management of autonomous mobile robot."
        self.intro_txt2 = "\nIf you need to create your own system ,  Click on : Create file"
        self.intro_txt3 = "\nIf you need to use an exist file                 ,  Click on : Import file"
        self.mode = 0
        self.info_txt = "Instruction for STEP 2 :"
        self.info_txt2 = "          1.  Select your node type"
        self.info_txt3 = "          2.  Left click on map to create node"
        self.info_txt4 = "               Right click on map to undo"
        self.info_txt5 = "          3.  Click 'OK' to confirm your system" 



        
    
    def on_closing(self):
        self.info_root.destroy()

    def clear_screen(self,scr):
        for child in scr.winfo_children():
            child.destroy()
   
    def info(self,text,posx,posy,txt_color,bg_color):
        Label(text =str(text),fg=txt_color,bg=bg_color,font=TFont2,).place(x=posx,y=posy)  

    def create_txt_pos(self,text,posx,posy,txt_color,bg_color):   
        Label(text =str(text),fg=txt_color,bg=bg_color,font=TFont,).place(x=posx,y=posy)
    
    def create_btn(self,screen,text,h,w,command,posx,posy):
        tk.Button(screen, text=text,height=h, width=w,command=command,fg='white',bg='azure4').place(x=posx,y=posy)
    
    def disable_btn(self,screen,text,h,w,command,posx,posy):
        tk.Button(screen, text=text,height=h, width=w,command=command,fg='white',bg='azure4',state=DISABLED).place(x=posx,y=posy)

    def create_file_btn(self):
        self.mode=1
        print('-----------------------------MODE : CREATE FILE-----------------------------')
    
    def import_file_btn(self):
        self.mode=2
        print('-----------------------------MODE : IMPORT FILE-----------------------------')
    
    def info_screen(self):
        screen_width = 400
        self.w = screen_width
        screen_height = 600
        self.h = screen_height
        self.info_root = Tk()
        self.info_root.title('Input following data')
        self.TFont = ("Times New Roman", 15)
        #screen visualize
        self.info_root.geometry(str(screen_width)+'x'+str(screen_height))
        self.info_root.eval('tk::PlaceWindow . center')
        self.info_root.protocol("WM_DELETE_WINDOW", self.on_closing) # To Immediately close program
        self.info_root.mainloop()

    
    