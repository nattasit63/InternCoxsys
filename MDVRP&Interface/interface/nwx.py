import numpy
import networkx as nx
import matplotlib.pyplot as plt



class NX():
    def __init__(self):
        self.node_list=[]
        self.node_pos=[]
        self.edge_list=[]
        self.astar_path = ''  
        self.Graph = nx.DiGraph()  
    
    
    def eul(self,c1,c2):
        f = self.node_pos[c1-1]
        s = self.node_pos[c2-1]
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

    def do_dist_matrix(self,list_of_node,list_of_edge,pos_of_node):
        self.node_list = list_of_node
        self.edge_list = list_of_edge
        self.node_pos = pos_of_node
        self.create(list_of_node,pos_of_node,list_of_edge)
        matrix_size = len(pos_of_node)
        distance_matrix = numpy.zeros(shape=(matrix_size,matrix_size))
        for i in range(len(list_of_edge)):
            for node in range(len(list_of_edge[i])-1):
                first,second = list_of_edge[i][node],list_of_edge[i][node+1]
                cost = self.eul(first,second)
                
                distance_matrix[first-1][second-1] = cost
        for j in range(1,len(list_of_node)+1):
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
                    distance_matrix[j-1][k-1]=99999.99
        

        return distance_matrix