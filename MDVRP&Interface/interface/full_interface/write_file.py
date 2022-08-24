from tkinter import filedialog
from tkinter.filedialog import asksaveasfile
from tkinter import *
class Write_file():
    def __init__(self):
        self.customer_amount = 0
        self.depot_amount = 0 
        self.entry_data = []
        self.depot_pos = []
        self.customer_pos = []
        self.max_vehicle_depot = 0
        self.max_load_vehicle = 0
        self.route_duaration = 0
    
    def write(self,data,amount_customer,amount_depot,pos):
        self.customer_amount = amount_customer
        self.depot_amount = amount_depot
        self.entry_data = data
        self.depot_pos = pos[0]
        self.customer_pos = pos[1]
        self.max_vehicle_depot = self.entry_data[0]
        self.max_load_vehicle = self.entry_data[1]
        self.route_duration = self.entry_data[2]

        total_line =  1 + self.depot_amount + self.customer_amount  + self.depot_amount
        total_id = self.depot_amount + self.customer_amount

        root = Tk()
        root.withdraw()
        file  =  filedialog.asksaveasfile(filetypes=[('text file','*.txt')],defaultextension='.txt',title='Save file as',mode='w')
        root.destroy()
        #write first line
        file.write(str(self.max_vehicle_depot)+' '+str(self.customer_amount)+' '+str(self.depot_amount)+'\n')

        #write second part
        for i in range(0,self.depot_amount):
            file.write(str(self.route_duration)+' '+str(self.max_load_vehicle)+'\n')

        #write customer (third part)
        for i in range(1,self.customer_amount+1):
            file.write(str(i)+' '+str(pos[1][i-1][0])+' '+str(pos[1][i-1][1])+' '+str(self.route_duration)+' '+str(data[i+2])+' '+'0'+' '+'0'+' '+'0'+' '+'0'+' '+'0'+' '+'0'+'\n')

        #write depot (fourth path)
        for i in range(1,self.depot_amount+1):
            file.write(str(i+self.customer_amount)+' '+str(pos[0][i-1][0])+' '+str(pos[0][i-1][1])+' '+'0'+'   '+'0'+' '+'0'+' '+'0'+'\n')

        file.close()

        return file.name