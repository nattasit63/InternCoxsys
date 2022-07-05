from array import array
from operator import le
from turtle import back
from mdvrp import MDVRP
import numpy as np
mdvrp = MDVRP()

mdvrp.open_config()
matrix = mdvrp.adjacency_matrix()

np_matrix = np.array(matrix)

#matrix for each depot
index = 0
new_matrix = np.delete(np_matrix,index,axis=1)
new_matrix = np.delete(new_matrix,index,0)


# print(matrix[:,1])  #column

# print(matrix)
# print('----------------------------------------------')
# print('----------------------------------------------')
# print(np_matrix)
# print('----------------------------------------------')
# print(new_matrix)

# print(mdvrp.separate_depot())   #To separate nodes in each depot to list

print(mdvrp.depot_matrix())

