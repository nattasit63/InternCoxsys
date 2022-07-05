from vrp import VRPPD

vrppd = VRPPD()
vrppd.open_config()
adjMatrix = vrppd.adjacency_matrix()


from networkx import from_numpy_matrix,relabel_nodes,DiGraph
from numpy import array
from vrpy import VehicleRoutingProblem
import networkx as nx

pickups_deliveries =  { 
        "1" : [1,2,1],
        "2" : [2,3,2],
        "3" : [3,4,3],
        "4" : [5,6,1],
        "5" : [6,7,2],
        "6" : [6,8,2],
        "7" : [8,9,4],
        "8" : [7,8,3],
        "9" : [7,9,3]  }
    


A = array(adjMatrix,dtype=[("cost",int)])
G = from_numpy_matrix(A,create_using=DiGraph())
G.add_node(10)


    
for i in range(0,10):
    G.add_edge(i,10,cost = 10)

G = relabel_nodes(G, {0: "Source", 10: "Sink"})
for (u, v) in pickups_deliveries.items():
    G.nodes[v[0]]["request"] = v[1]
    G.nodes[v[0]]["demand"] = v[2]
    G.nodes[v[1]]["demand"] = -v[2]

try:
    prob = VehicleRoutingProblem(G, load_capacity=10, num_stops=6, pickup_delivery=True)
    prob.solve(cspy=False)
    print(prob.best_routes)
except Exception as e:
    print(e)