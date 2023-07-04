import sumolib
import numpy as np
import pickle
import xml.etree.ElementTree as ET



net = sumolib.net.readNet("scenario/DCC.net.xml")
nodes = net.getNodes()
edges = net.getEdges()
nodes_len = len(nodes)
edges_len = len(edges)


f = open("edges_cost.txt", "w")


node_list = [None]
node_dict = {}
for i in range(1, len(nodes)+1):
    n = nodes[i-1]
    eid = n.getID()
    node_list.append(eid)
    node_dict[eid] = i
    
    
min_cost = np.Inf * np.ones((nodes_len+1, nodes_len+1))
edge_list = [None]
edge_dict = {}
k = 0

for e in edges:
    start = e.getFromNode().getID()
    end = e.getToNode().getID()
    ll = int(e.getLength())
    
    eid = e.getID()
    edge_list.append(eid)
    k += 1
    edge_dict[eid] = k
    min_cost[node_dict[start], node_dict[end]] = ll
    s = " ".join(map(lambda x:str(x), [node_dict[start], node_dict[end], ll, k])) + "\n"
    f.write(s)

f.close()

# with open('dublin1.pickle', 'wb') as f:
#     pickle.dump([node_dict, node_list, edge_dict, edge_list], f)