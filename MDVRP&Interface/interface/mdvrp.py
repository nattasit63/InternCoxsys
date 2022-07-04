import json
import networkx as nx
from networkx import DiGraph
import matplotlib.pyplot as plt
import numpy as np
from networkx import DiGraph, from_numpy_matrix, relabel_nodes, set_node_attributes
from numpy import array
from vrpy import VehicleRoutingProblem

G = nx.Graph()
# with open('/home/natta/thesis_ws/src/interface/config/example_mdvrp.json','r') as f:
#     data = json.load(f)
# nodes_json = data.get('nodes')
# for n,p in nodes_json.items():
#     print (n)

class MDVRP():
    def __init__(self):
        self.nodelist=[]
        self.nodepos=[]
        self.astar_path = 0
        self.astar_path_cost=[]
        self.adjMatrix=[]
        self.isPathAavailable=0
        self.demand_list=dict
        self.prob = 0
        self.route = []
        self.route_astar = []
        self.depot_edge_list = []
        
    
    def open_config(self):
        with open('/home/natta/thesis_ws/src/interface/config/example_mdvrp.json','r') as f:
            self.data = json.load(f)
        self.nodes_json = self.data.get('nodes')
        self.edges_json = self.data.get('edges')
        self.depot_json = self.data.get('depot')

        for n,p in self.nodes_json.items():
            self.nodelist.append(n)
            self.nodepos.append(p)
            G.add_node(str(n),pos=p)
        for s,e in self.edges_json.items():  #Add edge from yaml
            G.add_edge(str(e[0]),str(e[1]),weight = e[2])
            self.astar_path_cost.append(e[2])
        
        return 1

    def astar(self,graph,start_node,end_node):
        self.astar_path = nx.astar_path(graph,str(start_node),str(end_node), heuristic = None ,weight='weight')
        return self.astar_path
    
    def adjacency_matrix(self):
        sum=0
        size = len(self.nodelist)
        self.adjMatrix = np.zeros((size,size))
        for i in range(size):
            for j in range(size):
                sum = 0
                flag = 0  
                #0 = Have Path (No Error)   1 = Same Point   2 = No Path  
                if i==j :
                    flag = 1
                try:
                    path = self.astar(G,i,j)             
                except :
                    flag = 2

                if(flag == 0):
                    path = self.astar(G,i,j)
                    path_size = len(path)
                    for k in range(path_size-1):
                        for s,e in self.edges_json.items():
                            if(int(path[k])==e[0] and int(path[k+1])==e[1]):
                                sum += e[2]
                                break
                    for s,e in self.edges_json.items():
                        if int(i)==int(e[0]):
                            if int(j)==int(e[1]):
                                if(sum != 0):
                                    sum = min(e[2],sum)
                                else:
                                    sum = e[2]
                self.adjMatrix[i][j]= sum
        return self.adjMatrix

    def separate_depot(self):
        sub_edge_list=[]
        edge_list=[]

        for s,e in self.edges_json.items():
            sub_edge_list.append([e[0],e[1]])
        edge_list+=sub_edge_list      
        for depot in self.depot_json:
            depot_edge=[]
            now_node = 0
            for j in edge_list:
                if j[0]==depot:
                    depot_edge.append(j[0])
                    depot_edge.append(j[1])
                    now_node = j[1]
                    for k in edge_list:
                        if now_node==k[0]:
                            depot_edge.append(now_node)
                        else:
                            pass
                final = set(depot_edge)
                final = list(final)
            self.depot_edge_list.append(final)
        return self.depot_edge_list
    
    def depot_matrix(self):
        matrix = self.adjMatrix
        return matrix