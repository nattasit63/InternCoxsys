from tkinter import *
from tkinter import filedialog
import tkinter.messagebox
import tkinter
import yaml
import time

class Select():
    def __init__(self):
        #Initial
        self.flag_launch = 0
        self.image_file = ''
        self.fname=''
        # First screen
        self.root = Tk()
        self.root.title('open map')
        screen_width = 800
        screen_height = 600
        self.my_menu = Menu()
        self.sub_menu = Menu()
        self.depot_txt = StringVar()
        self.root.config(menu=self.my_menu)
        self.my_menu.add_cascade(label='File',menu=self.sub_menu)
        self.sub_menu.add_command(label='exit',command=self.exit_program)
        self.TFont = ("Cordia new", 10)      
        Button(self.root, text="select file",command=self.openFile).grid(row=1,column=0)
        self.root.geometry(str(screen_width)+'x'+str(screen_height))
        self.root.mainloop()

        # Info node screen
        # self.show_info = Tk()
        # self.show_info.title('Info')
        # self.show_info.geometry(str(screen_width)+'x'+str(screen_height))
        # self.depot_txt = StringVar()
        # self.config_name_txt = StringVar()
        # self.station_txt = StringVar()
        # self.station_list = []


        # to write data
        self.added_node = []
        self.node_pos = []
        self.edge_list = []
        self.edge_num = []
        

    def launch(self):
        self.flag_launch = 1
        self.root.destroy()
        return self.flag_launch
        # time.sleep(1)

    def openFile(self):
        # my_path_default = "$HOME/thesis_ws"
        print('openfile')
        my_path_default = "/home/natta/thesis_ws/src/interface/config"
        root_filename = filedialog.askopenfilename(initialdir=my_path_default,title='Select .yaml file',filetypes=(("yaml file","*.yaml"),("all files","*.*")))
        print("Path to File : ",root_filename)
        tkinter.messagebox.showinfo('File path',root_filename )
        Label(text = root_filename,fg="black",font=self.TFont).grid(row=1,column=1)
        if root_filename!='':
            Button(self.root,text='LAUNCH',bg='green',command=self.launch).grid(row=2,column=1)
        with open(root_filename,'r') as f:
            yml_dict = yaml.safe_load(f)
        self.image_file = yml_dict.get('image')
        self.resolution = yml_dict.get('resolution')
        self.origin     = yml_dict.get('origin')
        return 1
    
    def second_screen(self):
        pass
    
    def get_config(self):
        pass
 
    def exit_program(self):
        confirm = tkinter.messagebox.askquestion('Confirmation','Are you sure to exit program ?')
        if confirm == 'yes':
            self.root.destroy()
            exit() 

    