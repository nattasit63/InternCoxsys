
import matplotlib.pyplot as plt
from tkinter import filedialog
from tkinter.filedialog import asksaveasfile
from tkinter import *



import numpy as np
import networkx as nx
import matplotlib.pyplot as plt


# node_list = [1,2,3,4,5,6,7]
# edge_list = [[1,2],[2,3],[5,6],[6,4],[2,7]]
# pos = [[165,230],[198,172],[282,177],[290,456],[360,418],[420,451],[250,306]]




# nw = NX()
G = nx.Graph()


amount_depot = 2

node_pos = [[217, 246], [315, 490], [79, 163], [178, 136], [277, 77], [459, 98], [433, 500], [159, 509], [170, 204], [296, 133], [382, 84], [188, 342], [233, 495], [377, 242], [422, 374]]
edge_list = [[1, 9], [9, 1], [9, 3], [3, 9], [3, 4], [4, 3], [4, 10], [10, 4], [10, 5], [5, 10], [5, 11], [11, 5], [11, 6], [6, 11], [6, 14], [14, 6], [14, 15], [15, 14], [15, 7], [7, 15], [7, 2], [2, 7], [2, 13], [13, 2], [13, 8], [8, 13], [8, 12], [12, 8], [12, 1], [1, 12]]
node_list =[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15] 
route =[[1, '1', '7', '10', '5', '11', '6', 1], [2, '3', '4', '13', '12', '9', '8', '2', 2]]


def eul(node1,node2):
    pos1=node_pos[node1-1]
    pos2=node_pos[node2-1]
    cost = ((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)**0.5
    return "%.2f" % round(cost, 2)

def astar(start_node,end_node):
        astar_path = nx.astar_path(G,start_node,end_node, heuristic = None, weight="cost")
        return astar_path

def del_duplicate_route(route):
    pure = []
    temp = 0
    for z in range(len(route)):
        if route[z]!=temp:
            pure.append(route[z])
        temp = route[z]
    return pure
        # return list(dict.fromkeys(route))


##
for i in node_list:
    G.add_node(i,pos=node_pos[i-1])
for i in edge_list:
    G.add_edge(i[0],i[1],weight=(eul(i[0],i[1])))
pos = nx.get_node_attributes(G,'pos')
labels = nx.get_edge_attributes(G,'weight')
nx.draw_networkx_edge_labels(G,pos,edge_labels=labels)
nx.draw(G,pos,with_labels = True,arrows = True)
# plt.show()


#important
for i in route: #Do point in route to look up node list
    for j in range(len(i)):
        check_depot = isinstance(i[j],int)
        if not check_depot:
            i[j] = int(i[j])
            i[j] = i[j] + amount_depot


real_route = []
real_route_astar = []
for num in range(amount_depot):
    real_route.append([])
    real_route_astar.append([])

for i in route:
    for j in range(len(i)-1):
        depot = i[0]
        real_route[depot-1].append(astar(i[j],i[j+1]))

for i in range(len(real_route)):
    x = real_route[i]
    for el in sum(x,[]):
        real_route_astar[i].append(el)

print(real_route_astar)

for i in range(len(real_route_astar)):
    depot = real_route_astar[i][0]
    real_route_astar[i] = del_duplicate_route(real_route_astar[i])
    # real_route_astar[i].append(depot)
print(real_route_astar)

for i in real_route_astar:
    for j in range(len(i)):
        if i[j]==i[0] or i[j]==i[-1]:
            pass
        else:
            i[j] = str(i[j]-amount_depot)
            # pass
        
print(real_route_astar)




# print('original route :',route)
# print('astar route    :',real_route_astar)







# depot_index   =  [1, 2]
# customer_index =  [3, 4, 5, 6, 7, 8, 9]
# connect_index  =  [10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28]
# index_customer =   [3, 4, 5, 7, 8, 9, 10, 11, 12, 13]
# index_connect  =   [1, 2, 6]


# matrix = np.array(nw.do_dist_matrix(list_of_node,list_of_edge,pos_of_node,index_customer,index_connect))

# def del_connect_index_in_matrix(matrix,index_connect):
#     n=0
#     for i in index_connect:
#         matrix = np.delete(matrix,(i-1-n),axis=1)
#         matrix = np.delete(matrix,(i-1-n),axis=0)
#         n+=1
#         print('\n',matrix.shape,'\n')
#         print(matrix)
#         print('-----------------------------------------------------------------------','\n')
# # print(nw.do_dist_matrix(list_of_node,list_of_edge,pos_of_node,index_customer,index_connect))

# All_node_list =  [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28]
# List_of_edge  =  [[2, 26], [26, 2], [26, 27], [27, 26], [25, 27], [27, 25], [9, 25], [25, 9], [28, 25], [25, 28], [28, 24], [24, 28], [6, 24], [24, 6], [27, 9], [9, 27], [1, 11], [11, 1], [11, 10], [10, 11], [10, 1], [1, 10], [1, 12], [12, 1], [12, 11], [11, 12], [15, 1], [1, 15], [15, 10], [10, 15], [16, 15], [15, 16], [15, 18], [18, 15], [18, 3], [3, 18], [3, 20], [20, 3], [20, 19], [19, 20], [19, 4], [4, 19], [4, 17], [17, 4], [17, 18], [18, 17], [18, 19], [19, 18], [17, 20], [20, 17], [19, 2], [2, 19], [26, 19], [19, 26], [19, 27], [27, 19], [17, 16], [16, 17], [10, 5], [5, 10], [5, 11], [11, 5], [15, 5], [5, 15], [5, 12], [12, 5], [12, 7], [7, 12], [7, 13], [13, 7], [13, 12], [12, 13], [13, 8], [8, 13], [8, 14], [14, 8], [14, 13], [13, 14], [13, 21], [21, 13], [21, 14], [14, 21], [14, 23], [23, 14], [23, 21], [21, 23], [21, 22], [22, 21], [22, 16], [16, 22], [16, 21], [21, 16], [22, 24], [24, 22], [22, 28], [28, 22]]
# All_node_pos  =  [[202, 213], [369, 515], [344, 351], [506, 393], [474, 173], [730, 330], [415, 52], [615, 56], [752, 441], [383, 217], [379, 162], [372, 86], [492, 82], [614, 87], [386, 283], [495, 285], [502, 348], [411, 351], [399, 415], [337, 415], [620, 216], [577, 286], [680, 222], [679, 323], [674, 427], [436, 504], [557, 505], [621, 367]]
# depot_index   =  [1, 2]
# customer_index =  [3, 4, 5, 6, 7, 8, 9]
# connect_index  =  [10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28]



# matrix = nw.do_dist_matrix(All_node_list,List_of_edge,All_node_pos,customer_index,connect_index)

# print(len(matrix))







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
