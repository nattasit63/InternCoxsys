from tkinter import ttk
import tkinter as tk
from tkinter import *
from tkinter import messagebox,Tk, font,filedialog
import tkinter
import drawing 
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

        self.state = 0   #info screen state
        self.info_root_active = 0

        self.my_entries = []
        self.var_max_veh_depot = []
        self.var_max_load_veh = []
        # self.var_route_dur = []
       
    def on_closing(self):
        self.info_root_active=0
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
    
    def confirm_info(self):
        global export_data
        state = 0
        self.entry_list=[]
        if state ==0:
            if self.my_entries[0].get().isdigit()==False  or self.my_entries[1].get().isdigit()==False:
                print('input error')
                state = 0
            else:
                if self.my_entries[0].get() == '0':
                    print('Input must be postive integer')
                    state = 0
                else:
                    if len(self.my_entries[0].get())>=2 and self.my_entries[0].get()[0] == '0':
                        print('Format Error (max vehicle for depot) ')
                        state = 0
                        if len(self.my_entries[1].get())>=2 and self.my_entries[1].get()[0] == '0':
                            print('Format Error (max load vehicle)')
                            state = 0
                    else:
                        state = 1
        
        if state ==1 :         
            for entry  in self.my_entries:
                #  
                if entry.get().isdigit()==False:
                    print('Format Error (customer demand)')
                    state = 0
                else:
                    self.entry_list.append(int(entry.get()))
            # print('tk = ',self.entry_list)
            export_data = self.entry_list
            self.mode = 14
            self.info_root.destroy()

        # print(state)
    def info_screen(self):

        screen_width = 550
        self.w = screen_width
        screen_height = 750
        self.h = screen_height
        self.info_root = tk.Tk()
        self.info_root.title('Input following data')
        self.TFont = ("Times New Roman", 15)
        

        #screen visualize
        self.info_root_active = 1
        self.info_root.geometry(str(screen_width)+'x'+str(screen_height))
        self.info_root.eval('tk::PlaceWindow . center')
        self.info_root.protocol("WM_DELETE_WINDOW", self.on_closing) # To Immediately close program


       
        self.var_route_dur =tk.IntVar()

        #create widget

        main_frame = Frame(self.info_root)
        main_frame.pack(fill=BOTH, expand=1)
        my_canvas = Canvas(main_frame)
        my_canvas.pack(side=LEFT, fill=BOTH, expand=1)
        my_scrollbar = ttk.Scrollbar(main_frame, orient=VERTICAL, command=my_canvas.yview)
        my_scrollbar.pack(side=RIGHT, fill=Y)
        # Configure The Canvas
        my_canvas.configure(yscrollcommand=my_scrollbar.set)
        my_canvas.bind('<Configure>', lambda e: my_canvas.configure(scrollregion = my_canvas.bbox("all")))
        second_frame = Frame(my_canvas)
        my_canvas.create_window((0,0), window=second_frame, anchor="nw")
        

        tk.Label(second_frame,text='maximum vehicle for depot : ',font=TFont2).grid(sticky="W",row=0,column=0)
        tk.Label(second_frame,text='maximum load for vehicle   : ',font=TFont2).grid(sticky="W",row=1,column=0)
        tk.Label(second_frame,text='route duration(not required | default = 0) : ',font=TFont2).grid(sticky="W",row=2,column=0)
        tk.Label(second_frame,text='Customer ID',font=TFont2).grid(sticky=W+E,row=3,column=0,ipady=20)
        tk.Label(second_frame,text='Demand',font=TFont2).grid(sticky=W+E,row=3,column=1,ipady=20)
    
        max_vehicle_depot = tk.Entry(second_frame,width=10)
        max_vehicle_depot.grid(row=0,column=1)
        self.my_entries.append(max_vehicle_depot)

        max_load_vehicle = tk.Entry(second_frame,width=10)
        max_load_vehicle.grid(row=1,column=1)
        self.my_entries.append(max_load_vehicle)

        route_duration = tk.Entry(second_frame,width=10)
        route_duration.grid(row=2,column=1)
        self.my_entries.append(route_duration)  

        for x in range(1,drawing.amount_customer+1):
            demand = tk.Entry(second_frame,width=10)
            tk.Label(second_frame,text=str(x),font=TFont2).grid(sticky=W+E,row=x+3,column=0)
            demand.grid(row=x+3,column=1)
            self.my_entries.append(demand)
        

        tk.Button(second_frame,text='Confirm',fg='white',bg='DarkOliveGreen4',width=4,height=1,command=self.confirm_info).grid(row=drawing.amount_customer+4,column=0,ipady=5)
        #run
        self.info_root.mainloop()

    
    