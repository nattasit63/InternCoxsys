
import numpy as np
import networkx as nx
import matplotlib.pyplot as plt



class NX():

    def __init__(self):
        
        self.node_list=[]
        self.node_pos=[]
        self.edge_list=[]
        self.customer_index = []
        self.connect_point_index = []
        self.astar_path = ''  
        self.dist_matrix = []
        self.Graph = nx.Graph()
    
    
    def eul(self,c1,c2):
        f = node_pos[c1-1]
        s = node_pos[c2-1]
        cost = ((f[0] - s[0])**2 + (f[1] - s[1])**2)**0.5
        return "%.2f" % round(cost, 2)
    
    def create(self,nodelist,nodepos,edge_list):
        for i in range(len(nodelist)):
            self.Graph.add_node(nodelist[i],pos=nodepos[i])   
        for edge in edge_list:
            self.Graph.add_edge(edge[0],edge[1],weight=(self.eul(edge[0],edge[1])))

    def astar(self,start_node,end_node):
        astar_path = nx.astar_path(self.Graph,start_node,end_node, heuristic = None, weight="cost")
        return astar_path
    
    def show_nx_graph(self):
        pos = nx.get_node_attributes(self.Graph,'pos')
        # labels = nx.get_edge_attributes(G,'weight')
        labels = nx.get_edge_attributes(self.Graph,'weight')
        nx.draw_networkx_edge_labels(self.Graph,pos,edge_labels=labels)
        nx.draw(self.Graph,pos,with_labels = True,arrows = True)
        plt.show()

    def del_connect_index_in_matrix(self,matrix,index_connect):
        n=0
        for i in index_connect:
            matrix = np.delete(matrix,(i-1-n),axis=1)
            matrix = np.delete(matrix,(i-1-n),axis=0)
            n+=1
            # print('\n',matrix.shape,'\n')
            # print(matrix)
            # print('-----------------------------------------------------------------------','\n')
        return matrix

    def do_dist_matrix(self,list_of_node,list_of_edge,pos_of_node,customer_index_list,connect_point_list):
        global adj_dist_matrix
        global node_pos
        global node_list
        global edge_list
        # print('-------------List edge ----------------  =',list_of_edge)
        self.node_list = list_of_node
        self.edge_list = list_of_edge
        self.node_pos = pos_of_node
        node_list = list_of_node
        node_pos = pos_of_node
        edge_list = list_of_edge
        self.create(node_list,node_pos,edge_list)

        #Do major matrix
        matrix_size = len(pos_of_node)
        distance_matrix = np.zeros(shape=(matrix_size,matrix_size))
        for i in range(len(list_of_edge)):  #insert cost of directly edge
            for node in range(len(list_of_edge[i])-1):
                first,second = list_of_edge[i][node],list_of_edge[i][node+1]
                cost = self.eul(first,second)
                
                distance_matrix[first-1][second-1] = cost

        for j in range(1,len(list_of_node)+1): #insert a* cost
            checker = list_of_node.copy()
            del checker[j-1]
            for k in checker:
                sum=0.0
                try:          
                    path = self.astar(j,k)
                    # print('PATH ACCEPT : ',(j,k),'  --> ',path)
                    for l in range(len(path)-1):
                        sum +=float(self.eul(path[l],path[l+1]))
                        # print(path[l],path[l+1],eul(path[l],path[l+1]),'sum = ',sum)
                    distance_matrix[j-1][k-1] = sum


                except:
                    # print('ERROR : ',(j,k))
                    distance_matrix[j-1][k-1]=9999.99
        print('Major matrix is calculated')
        self.dist_matrix = distance_matrix

        adj_dist_matrix    =  self.dist_matrix    
        print(self.dist_matrix )


        self.customer_index = customer_index_list
        self.connect_point_index = connect_point_list

        # self.minor_matrix = distance_matrix.copy()
        # self.minor_matrix = self.del_connect_index_in_matrix(self.dist_matrix,connect_point_list)
        # print('Minor matrix is calculated')
        # print(self.minor_matrix)
   
        return distance_matrix
    
    def do_real_path(self,solution):
        self.create(node_list,node_pos,edge_list)


        self.real_route = []
        self.true_route = []
        sol = solution.split('\n')

        sol = sol[:-1]

        for i in range(len(sol)):
            sol[i] = sol[i].split('\t')
            del sol[i][1]
            del sol[i][1]
            del sol[i][1]
            sol[i][0] = int(sol[i][0])
            sol[i][1] =sol[i][1].split()

        for j in range(len(sol)):
            depot = sol[j][0]
            for point in range(len(sol[j][1])):
                if sol[j][1][point] == '0':
                    sol[j][1][point]=depot
            self.true_route.append(sol[j][1])

        list_node  =  list(self.Graph.nodes)  
      
    
        #[[1, '3', '2', '1', 1], [2, '8', '7', '9', 2], [3, '6', '5', '4', 3]]
        print(node_pos)
        print(list_node,type(list_node))
        print(self.true_route)
