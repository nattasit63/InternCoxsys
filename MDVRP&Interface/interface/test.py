from tkinter import filedialog
from tkinter.filedialog import asksaveasfile
from tkinter import *
import os
#Input
amount_customer = 20
amount_depot = 2
pos =  [[[74, 264], [242, 239]], 
        [[62, 317], [94, 298], [77, 228], [107, 238], [130, 271], [164, 301], [195, 346], [259, 306], 
        [164, 245], [204, 223], [226, 187], [238, 160], [282, 180], [254, 204], [263, 235], [312, 252], 
        [268, 266], [280, 282], [232, 287], [180, 275]]]

data =  [4, 100, 0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140, 150, 160, 170, 180, 190, 200]


#prepare data
customer_amount = amount_customer
depot_amount = amount_depot
entry_data = data
depot_pos = pos[0]
customer_pos = pos[1]
max_vehicle_depot = entry_data[0]
max_load_vehicle = entry_data[1]
route_duration = entry_data[2]

total_line =  1 + depot_amount + customer_amount  + depot_amount
total_id = depot_amount + customer_amount



def save_file():
    #select save data directory
    root = Tk()
    root.withdraw()
    file  =  filedialog.asksaveasfile(filetypes=[('text file','*.txt')],defaultextension='.txt',title='Save file as',mode='w')
    root.destroy()
    #write first line
    file.write(str(max_vehicle_depot)+' '+str(customer_amount)+' '+str(depot_amount)+'\n')

    #write second part
    for i in range(0,depot_amount):
        file.write(str(route_duration)+' '+str(max_load_vehicle)+'\n')

    #write customer (third part)
    for i in range(1,customer_amount+1):
        file.write(str(i)+' '+str(pos[1][i-1][0])+' '+str(pos[1][i-1][1])+' '+str(route_duration)+' '+str(data[i+2])+' '+'0'+' '+'0'+' '+'0'+' '+'0'+' '+'0'+' '+'0'+'\n')

    #write depot (fourth path)
    for i in range(1,depot_amount+1):
        file.write(str(i+customer_amount)+' '+str(pos[0][i-1][0])+' '+str(pos[0][i-1][1])+' '+'0'+'   '+'0'+' '+'0'+' '+'0'+'\n')

    file.close()


# print(save_file())