
import matplotlib.pyplot as plt
from tkinter import filedialog
from tkinter.filedialog import asksaveasfile
from tkinter import *
import os
from solver import Solver

from nwx import NX

import numpy
import networkx as nx

node_list = [1,2,3,4,5,6,7]
edge_list = [[1,2],[2,3],[5,6],[6,4],[2,7]]
pos = [[165,230],[198,172],[282,177],[290,456],[360,418],[420,451],[250,306]]

nw = NX()

print(nw.do_dist_matrix(node_list,edge_list,pos))


# Graph = nx.DiGraph()

# def show_nx_graph():
#     pos = nx.get_node_attributes(Graph,'pos')
#     # labels = nx.get_edge_attributes(G,'weight')
#     labels = nx.get_edge_attributes(Graph,'weight')
#     nx.draw_networkx_edge_labels(Graph,pos,edge_labels=labels)
#     nx.draw(Graph,pos,with_labels = True,arrows = True)
#     plt.show()



# def eul(c1,c2):
#     f = pos[c1-1]
#     s = pos[c2-1]
#     cost = ((f[0] - s[0])**2 + (f[1] - s[1])**2)**0.5
#     return "%.2f" % round(cost, 2)


# def create(nodelist,nodepos):
#     for i in range(len(nodelist)):
#         Graph.add_node(nodelist[i],pos=nodepos[i])   
#     for edge in edge_list:
#         Graph.add_edge(edge[0],edge[1],weight=(eul(edge[0],edge[1])))

# def astar(start_node,end_node):
#     astar_path = nx.astar_path(Graph,start_node,end_node, heuristic = None, weight="cost")
#     return astar_path


# def do_dist_matrix(list_of_node,list_of_edge,pos_of_node):
#     create(list_of_node,pos_of_node)
#     matrix_size = len(pos_of_node)
#     distance_matrix = numpy.zeros(shape=(matrix_size,matrix_size))
#     for i in range(len(list_of_edge)):
#         for node in range(len(list_of_edge[i])-1):
#             first,second = list_of_edge[i][node],list_of_edge[i][node+1]
#             cost = eul(first,second)
            
#             distance_matrix[first-1][second-1] = cost
#     for j in range(1,len(list_of_node)+1):
#         checker = node_list.copy()
#         del checker[j-1]
#         for k in checker:
#             sum=0.0
#             try:          
#                 path = astar(j,k)
#                 # print('PATH ACCEPT : ',(j,k),'  --> ',path)
#                 for l in range(len(path)-1):
#                     sum +=float(eul(path[l],path[l+1]))
#                     # print(path[l],path[l+1],eul(path[l],path[l+1]),'sum = ',sum)
#                 distance_matrix[j-1][k-1] = sum


#             except:
#                 # print('ERROR : ',(j,k))
#                 distance_matrix[j-1][k-1]=99999.99
    

#     return distance_matrix


# create(node_list,pos)

# print(do_dist_matrix(node_list,edge_list,pos))
# show_nx_graph()

















# solve = Solver()

# solve.insert_file_path('/home/natta/interface_ws/src/full_interface/data/2-30.txt')
# cost,sol = solve.run()

# print(sol)

# amount_depot = 2
# amount_customer = 7


# sol = ['1\t1\t227.63\t20\t0 4 3 2 1 0', '2\t1\t197.24\t15\t0 7 6 5 0', '']
# sol = sol[:-1]
# true_route = []



# for i in range(len(sol)):
#     sol[i] = sol[i].split('\t')
#     del sol[i][1]
#     del sol[i][1]
#     del sol[i][1]
#     sol[i][0] = int(sol[i][0])
#     sol[i][1] =sol[i][1].split()


# for j in range(len(sol)):
#     depot = sol[j][0]
#     for point in range(len(sol[j][1])):
#         if sol[j][1][point] == '0':
#             sol[j][1][point]=depot
#     true_route.append(sol[j][1])


# # print(sol)
# print(true_route)






# depot_pos = [[109, 285], [210, 316], [261, 246]]


# customer_pos = [[71, 326], [79, 281], [95, 256], [183, 314], [198, 299], [218, 298], [229, 232], [264, 274], [270, 266], [264, 194], [245, 312], [173, 264], [151, 285], [177, 230], [227, 167]]


# true_route = [[1, '1', '2', '3', '13', 1], [2, '11', '6', '5', '12', '4', 2], [3, '10', '15', '14', '7', '8', '9', 3]]

# for i in range(len(true_route)):
#     for j in range(len(true_route[i])):
#         check_depot = isinstance(true_route[i][j],int)
#         if check_depot:
#             true_route[i][j] = depot_pos[int(true_route[i][j])-1]
#         else:
#             true_route[i][j] = customer_pos[int(true_route[i][j])-1]


# print((true_route))
# pos=[]
# for i in range(len(true_route)):
#     for j in range(len(true_route[i])-1):
#         current = true_route[i][j]
#         next = true_route[i][j+1]
#         print('Current = ',current,'  , Next  = ',next)
       





# config_path = '/home/natta/interface_ws/src/full_interface/data/2-30.txt'
# sol_path= '/home/natta/interface_ws/src/full_interface/data/sol_2-30.txt'

# optimize_cost = 0.0
# major_separate_line = []

# major_route = []


# def get_sol_info(sol_path):  
#     depot_sep_route =[]
#     sol = open(sol_path,'r')  
#     lines = sol.readlines()
#     total_route_line = len(lines)-1
#     # get cost data
#     optimize_cost = lines[0]
#     # info vehicle
#     total_depot = int(lines[-1][0])
#     for j in range(0,total_depot):
#         depot_sep_route.append([])
#     for i in range(1,total_route_line+1):
#         test = lines[i]
#         this_line = test.split("\t")
#         this_line_route = this_line[-1].split('\n')[0].replace(' ',',').split(",")
#         depot_id = int(this_line[0])-1
#         depot_sep_route[depot_id].append(this_line_route)
#     return depot_sep_route

# # print(get_sol_info(sol_path))


# def get_config_info(config_path):
#     split_data=[]
#     customer_pos_info = []
#     depot_pos_info = []
#     con = open(config_path,'r')
#     lines = con.readlines()
#     total_line = len(lines)
#     major_line = lines[0].split()
#     veh_each_depot = major_line[0]
#     total_customer = int(major_line[1])
#     total_depot = int(major_line[2])
#     skip_line = int(total_depot) +1
    
#     for i in range(skip_line,total_line):  #split data
#         split_data.append(lines[i].split())

#     for j in range(len(split_data)-total_depot): #append customer pos
#         customer_pos_info.append([int(split_data[j][1]),int(split_data[j][2])])
    
#     for k in range(skip_line+total_customer-total_depot-1,len(split_data)):
#         depot_pos_info.append([int(split_data[k][1]),int(split_data[k][2])])

#     return depot_pos_info,customer_pos_info
    

# # print(get_config_info(config_path))

# def matcher(sol_path,config_path):
#     depot_id = 0
#     pos_each_route = []
#     raw_route = get_sol_info(sol_path)
#     depot_pos_info,customer_pos_info = get_config_info(config_path)
#     total_depot= len(raw_route)

#     for j in range(0,total_depot):
#         pos_each_route.append([])
#     try :
#         if len(depot_pos_info)==len(raw_route):
#             print('Matched !')
#             for route in raw_route:
#                 depot_id+=1
#                 # print('--------------------------')
#                 for i in range(len(route)):
#                     for j in range(len(route[i])):
     
#                         if route[i][j]==str(0):
#                             route[i][j] = depot_id
#             # print(raw_route)    
#             depot_id = 0
#             for new_route in raw_route:
#                 depot_id+=1
#                 for i in range(len(new_route)):
#                     for j in range(len(new_route[i])):
#                         # print(new_route[i][j])
#                         check_depot = isinstance(new_route[i][j],int)
#                         if check_depot:
#                             pos_each_route[depot_id-1].append([depot_pos_info[depot_id-1]])
#                             new_route[i][j] = depot_pos_info[depot_id-1]
#                         else :
#                             new_route[i][j] = customer_pos_info[int(new_route[i][j])-1]
#             print('ROUTE_POS :')
#             return raw_route
#     except:
#         return 'Fail ! your files are not match'

# print(matcher(sol_path,config_path))
























# #Input
# amount_customer = 20
# amount_depot = 2
# pos =  [[[74, 264], [242, 239]], 
#         [[62, 317], [94, 298], [77, 228], [107, 238], [130, 271], [164, 301], [195, 346], [259, 306], 
#         [164, 245], [204, 223], [226, 187], [238, 160], [282, 180], [254, 204], [263, 235], [312, 252], 
#         [268, 266], [280, 282], [232, 287], [180, 275]]]

# data =  [4, 100, 0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140, 150, 160, 170, 180, 190, 200]


# #prepare data
# customer_amount = amount_customer
# depot_amount = amount_depot
# entry_data = data
# depot_pos = pos[0]
# customer_pos = pos[1]
# max_vehicle_depot = entry_data[0]
# max_load_vehicle = entry_data[1]
# route_duration = entry_data[2]

# total_line =  1 + depot_amount + customer_amount  + depot_amount
# total_id = depot_amount + customer_amount



# def save_file():
#     #select save data directory
#     root = Tk()
#     root.withdraw()
#     file  =  filedialog.asksaveasfile(filetypes=[('text file','*.txt')],defaultextension='.txt',title='Save file as',mode='w')
#     root.destroy()
#     #write first line
#     file.write(str(max_vehicle_depot)+' '+str(customer_amount)+' '+str(depot_amount)+'\n')

#     #write second part
#     for i in range(0,depot_amount):
#         file.write(str(route_duration)+' '+str(max_load_vehicle)+'\n')

#     #write customer (third part)
#     for i in range(1,customer_amount+1):
#         file.write(str(i)+' '+str(pos[1][i-1][0])+' '+str(pos[1][i-1][1])+' '+str(route_duration)+' '+str(data[i+2])+' '+'0'+' '+'0'+' '+'0'+' '+'0'+' '+'0'+' '+'0'+'\n')

#     #write depot (fourth path)
#     for i in range(1,depot_amount+1):
#         file.write(str(i+customer_amount)+' '+str(pos[0][i-1][0])+' '+str(pos[0][i-1][1])+' '+'0'+'   '+'0'+' '+'0'+' '+'0'+'\n')

#     file.close()

#     return file.name
# #print(save_file())
