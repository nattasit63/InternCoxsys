import json
from turtle import pen
import networkx as nx
from networkx import DiGraph
import matplotlib.pyplot as plt
import numpy as np
from networkx import DiGraph, from_numpy_matrix, relabel_nodes, set_node_attributes
from numpy import array
from vrpy import VehicleRoutingProblem

G = nx.DiGraph()

class VRPPD():
    def __init__(self):
        self.pickup_node =[]
        self.depot_node =[]
        self.dropoff_node =[]
        self.nodelist=[]
        self.astar_path = 0
        self.astar_path_cost=[]
        self.adjMatrix=[]
        self.isPathAavailable=0
        self.demand_list=dict
        self.prob = 0
        self.route = []
        self.route_astar = []
        self.nodepos=[]
        self.vrp_graph = None
        self.backup_graph = []
        self.best_route=dict
        self.route_astar=[]
    def open_config(self):
        with open('/home/natta/thesis_ws/src/interface/config/example_vrppd.json','r') as f:
            self.data = json.load(f)
        self.nodes_json = self.data.get('nodes')
        self.edges_json = self.data.get('edges')
        self.depot_json = self.data.get('depot')
        self.pickup_delivery_json = self.data.get('PICKUP_DELIVERIES')

        for n,p in self.nodes_json.items():
            self.nodelist.append(n)
            self.nodepos.append(p)
            G.add_node(int(n),pos=p)
        
        for s,e in self.edges_json.items():  #Add edge from yaml
            G.add_edge(int(e[0]),int(e[1]),weight = e[2])          
            self.astar_path_cost.append(e[2])

        self.G_backup = G.copy()

        return 1
        
    def astar(self,graph,start_node,end_node):
        self.astar_path = nx.astar_path(graph,start_node,end_node, heuristic = None ,weight='weight')
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
                            if(int(path[k])==int(e[0]) and int(path[k+1])== int(e[1])):
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
    
    def vrppd_solver(self,capacity,max_station):
        self.adjacency_matrix()
        A = array(self.adjMatrix,dtype=[("cost",float)])

        B = from_numpy_matrix(A,create_using=DiGraph())

        depot_node = int(self.nodelist[0])
        last_node = int(self.nodelist[-1])
        sink_node = last_node+1
        B.add_node(sink_node)

        
        self.nodelist.append(sink_node)

        for node in self.nodelist: #Connect Sink 
            if node != sink_node:
                B.add_edge(int(node),int(sink_node),cost=10)
        for node in self.nodelist:
            if int(node)!=int(depot_node):
                B.add_edge(int(depot_node),int(node),cost=10)
        
        
        B = relabel_nodes(B, {0: "Source", self.nodelist[-1]: "Sink"})
  
        for (u, v) in self.pickup_delivery_json.items():
            B.nodes[v[0]]["request"] = v[1]
            B.nodes[v[0]]["demand"] = v[2]
            B.nodes[v[1]]["demand"] = -v[2]
        
        # print(G.nodes)
        # print(G.edges)

        try:
            prob = VehicleRoutingProblem(B, load_capacity=capacity, num_stops=max_station, pickup_delivery=True)
            prob.solve(cspy=False,pricing_strategy="Exact")
            # print(prob.best_value)
            self.best_route = prob.best_routes
            return self.best_route
        except Exception as e:
            # print('ERROR')
            return e


    def a_star_for_vehicle(self):
        depot_node = int(self.nodelist[0])
        last_node = int(self.nodelist[-1])
        # print(last_node)
        sink_node = last_node
        self.G_backup.add_node(sink_node)

        for node in self.nodelist: #Connect Sink 
            if node != sink_node:
                self.G_backup.add_edge(int(node),int(sink_node),cost=10)

        # for node in self.nodelist:
        #     if int(node)!=int(depot_node):
        #         self.G_backup.add_edge(int(depot_node),int(node),cost=10)
        
        t=[]
        t_map=[]
        t_route=[]
        for s,e in self.best_route.items():
            t.append(e)
            t_map.append(e)
            for i in range(0,len(t)):
                self.route=[]
                t1=[]
                t2=[]
                temp=''
                b=[]
                c=[]
                len_route = len(t[i])
                for j in range(len_route):
                    if t[i][j] == 'Source':
                        t[i][j] = 0
                        t_map[i][j] = 0
                    if t[i][j] == 'Sink':
                        t[i][j] = self.nodelist[-1]
                        t_map[i][j] = self.nodelist[-1]
                for k in range(len_route-1):
                    t1+=(self.astar(self.G_backup,t[i][k],t[i][k+1]))
                    t2+=(self.astar(self.G_backup,t_map[i][k],t_map[i][k+1]))

                for z in range(len(t1)):
                    if t1[z]!=temp:
                        b.append(t1[z])
                        c.append(t1[z])
                    temp = t1[z]
                b[0]='Source'
                b[-1]='Sink'
                t_route.append(c)
            self.route_astar.append(b)  

        return self.route_astar

        