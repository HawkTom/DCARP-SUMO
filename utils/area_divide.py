import sumolib
from plot_net_selection import plot_edges
import xml.etree.ElementTree as ET
import matplotlib.pyplot as plt
import numpy as np
import pickle
import random
import sys




# junction_id = ["1426048874", "14957877", "26867634", "29396438", "659687", "696775774", "cluster_1396434747_1396434748", "cluster_3133124965_3133124968_4298585027_4298585028_4298585029_442394468", "cluster_5597784479_5597784482", "gneJ47", "gneJ71"]



# junction_id = [(3148,3248), (3629,3248),(4549,3248),(3148,1948),(2049,1948),(2049,1000), (4549,1000)]
# traci.start(["sumo-gui", "-c", "scenario/DCC_simulation.sumo.cfg"], port=7911)

# k = 0
# for j_id in junction_id:
#     k += 1
#     traci.poi.add("x"+str(k), j_id[0], j_id[1], (1,0,0,0))

# traci.close()

# x1 = (3148,3248)
# x2 = (3629,3248)
# x3 = (4549,3248)
# x4 = (3148,1948)
# x5 = (2049,1948)
# x6 = (2049,1000)
# x7 = (4549,1000)

# area 1&3 determined as the uncrowded area
# area 2&4&5 determined as the busy area
def divide_area(edges):
    area_edges = {"area1":[], "area2":[], "area3":[], "area4":[], "area5":[]}
    edges_area = {} #store the area of each edge that belongs to: busy or uncrowded
    for e in edges:
        from_node = e.getFromNode()
        to_node = e.getToNode()
        a = from_node.getCoord()
        b = to_node.getCoord()
        x, y = (a[0] + b[0])/2, (a[1] + b[1])/2

        if y >= 3248 and x < 3629:
            area = 1
        elif y >= 3248 and x >= 3629:
            area = 2
        elif y < 1000 and x >= 4549:
            area = 5
        elif y < 1000 and x < 4549:
            area = 3
        elif y >=1000 and y < 1948 and x < 2049:
            area = 3
        elif y >=1000 and y < 1948 and x >= 2049 and x < 4549:
            area = 4
        elif y >=1000 and y < 1948 and x >= 4549:
            area = 5
        elif y >=1948 and y < 3248 and x < 3148:
            area = 3
        elif y >=1948 and y < 3248 and x >= 3148 and x < 4549:
            area = 4
        elif y >=1948 and y < 3248 and x >= 4549:
            area = 5

        eid = e.getID()

        if area == 1:
            area_edges["area1"].append(eid)
            edges_area[eid] = "uncrowded"
        elif area == 2:
            area_edges["area2"].append(eid)
            edges_area[eid] = "busy"
        elif area == 3:
            area_edges["area3"].append(eid)
            edges_area[eid] = "uncrowded"
        elif area == 4:
            area_edges["area4"].append(eid)
            edges_area[eid] = "busy"
        elif area == 5:
            area_edges["area5"].append(eid)
            edges_area[eid] = "busy"

    print(len(area_edges["area1"]), len(area_edges["area2"]), len(area_edges["area3"]), len(area_edges["area4"]), len(area_edges["area5"]))
    return area_edges, edges_area

def plot_area(area_edges):
    args = ["-n", "scenario/DCC.net.xml", "--xlim", "-100,7200", "--ylim", "-100,5400", "--xticks", "-100,7201,2000,16", "--yticks", "-100,5401,1000,16", "--selected-width", "2", "--edge-width", ".5", "--edge-color", "#606060", "--selected-color", "#800000", "--online", "off"]
    plot_edges(area_edges, args)



def calculate_congestion(area_edges, edges):
    edges_max_speed = {}
    for e in edges:
        edges_max_speed[e.getID()] = e.getSpeed()

    f = open("congestion.txt","w") 
    f.close()
    tree = ET.ElementTree(file="scenario/edge_traffic2.xml")
    root = tree.getroot()
    interval_index = 0
    for interval in root:
        interval_index += 1
        edges_speed = {}
        for e in interval:
            eid = e.attrib["id"]
            try:
                s = float(e.attrib["speed"])
            except:
                s = 0
            # print(eid, s, type(s))
            edges_speed[eid] = s

        for area in area_edges:
            edges_congestion = ["{0}_{1}:".format(interval_index, area[-1])]
            for eid in area_edges[area]:
                if eid in edges_speed:
                    max_speed = edges_max_speed[eid]
                    speed = edges_speed[eid]
                    congestion = str(max(1 - speed/max_speed, 0))
                else:
                    congestion = '0'
                edges_congestion.append(congestion)
            f = open("congestion.txt","a") 
            f.write(" ".join(edges_congestion)+"\n")
            f.close()

def congest_analysis_by_plot():
    congest = {"area1":[], "area2":[], "area3":[], "area4":[], "area5":[]}
    with open("congestion.txt", "r") as f:
        for line in f.readlines():
            s = line.split()
            idx = s[0].split('_')
            interval = int(idx[0])
            area = int(idx[1][:-1])
            speed_ratio = list(map( lambda x:float(x), s[1:]))
            x = sum(i > 0.8 for i in speed_ratio)/len(speed_ratio)
            if area == 1:
                congest["area1"].append(x)
            elif area == 2:
                congest["area2"].append(x)
            elif area == 3:
                congest["area3"].append(x)
            elif area == 4:
                congest["area4"].append(x)
            elif area == 5:
                congest["area5"].append(x)

    # with open("congestion1.txt", "r") as f:
    #     for line in f.readlines():
    #         s = line.split()
    #         idx = s[0].split('_')
    #         interval = int(idx[0])
    #         area = int(idx[1][:-1])
    #         speed_ratio = list(map( lambda x:float(x), s[1:]))
    #         x = sum(i > 0.8 for i in speed_ratio)/len(speed_ratio)
    #         if area == 1:
    #             congest["area1"].append(x)
    #         elif area == 2:
    #             congest["area2"].append(x)
    #         elif area == 3:
    #             congest["area3"].append(x)
    #         elif area == 4:
    #             congest["area4"].append(x)
    #         elif area == 5:
    #             congest["area5"].append(x)


    x = np.array(list(range(1, 1441)))
    y1 = np.array(congest["area1"])
    y4 = np.array(congest["area4"])
    y3 = np.array(congest["area3"])
    y2 = np.array(congest["area2"])
    y5 = np.array(congest["area5"])
    plt.plot(x, y1, label ="area1")
    plt.plot(x, y2, label ="area2")
    plt.plot(x, y3, label ="area3")
    plt.plot(x, y4, label ="area4")
    plt.plot(x, y5, label ="area5")
    plt.ylim(0,1)
    plt.ylabel("Busy degree")
    plt.legend(loc="upper left")
    plt.show()





def generate_tasks():
    # busy area: area1, area3
    # non busy area: area2, area4, area5

    with open('dublin.pickle', 'rb') as f:
        map_data = pickle.load(f)

    node_dict = map_data[0] 
    node_list = map_data[1]
    edge_dict = map_data[2]
    edge_list = map_data[3] 

    f = open("initial_solution.txt", "r")
    seqs = f.readline().split(" ")
    demand = f.readline().split(" ")
    f.close()
    tasks_seqs = list(map(lambda x:int(x), seqs[2:-1]))
    demand_seqs = list(map(lambda x:int(x), demand[2:-1]))

    net = sumolib.net.readNet("scenario/DCC.net.xml")
    root = ET.Element('info', {'description': "depot in not center area, tasks distribute in the whole city"})
    # depot = ["13866376#0", "-13866376#4"]
    depot = ["23347664#0", "-23347664#1"]
    


    depot_from_node = net.getEdge(depot[1]).getFromNode().getID()
    depot_to_node = net.getEdge(depot[1]).getToNode().getID()

    ET.SubElement(root,'depot', {'id':'incoming', 'edge': depot[0], 'from_node': depot_from_node, 'from_index':str(node_dict[depot_from_node]), 'to_node':depot_to_node, 'to_index':str(node_dict[depot_to_node])}) # depot_out: 0
    ET.SubElement(root,'depot', {'id':'outgoing', 'edge': depot[1], 'from_node': depot_to_node, 'from_index':str(node_dict[depot_to_node]), 'to_node':depot_from_node,'to_index':str(node_dict[depot_from_node])}) # depot_in: 1

    ET.SubElement(root, 'capacity', {'value': str(200)})

    task_ele = ET.SubElement(root, "tasks")
    k = 0
    for i in range(len(tasks_seqs)):
        if tasks_seqs[i] == 0:
            continue
        k += 1
        
        tsk_id_index = tasks_seqs[i]
        tsk_eid = edge_list[tasks_seqs[i]]
        tsk_demand = demand_seqs[i]

        from_node = net.getEdge(tsk_eid).getFromNode().getID()
        to_node = net.getEdge(tsk_eid).getToNode().getID()

        ET.SubElement(task_ele,'task', {'id':str(k), 'edge': tsk_eid, 'from_node': from_node, 'from_index':str(node_dict[from_node]), 'to_node':to_node,'to_index': str(node_dict[to_node]), 'demand':str(tsk_demand), 'id_index':str(tsk_id_index),})
    
    tree=ET.ElementTree(root)
    tree.write("xml/scenario.xml")
    pass

"""
depot: 
    city center: 
        incoming = "13866376#0"
        outgoing = "-13866376#4"

    not city center : 
        incoming = "23347664#0" 
        outgoing = "-23347664#1"

tasks:
    not busy area: area1, area3
    busy area: area2, area4, area5
"""

def generate_scenarios(area_edges):
    depots = [["13866376#0", "-13866376#4"], ["23347664#0", "-23347664#1"]]
    start_time = [32400, 54000]
    # tasks: edge_ID, demand = edge_length/10

    with open('dublin.pickle', 'rb') as f:
        map_data = pickle.load(f)

    node_dict = map_data[0] 
    node_list = map_data[1]
    edge_dict = map_data[2]
    edge_list = map_data[3] 

    
    center_depot = ["13866376#0", "-13866376#4"] # city center
    not_center_depot = ["23347664#0", "-23347664#1"] # not city center

    not_busy_aval_edges = area_edges["area1"] + area_edges["area3"]
    busy_aval_edges = area_edges["area2"] + area_edges["area4"]+ area_edges["area5"]

    select0 = list(range(len(not_busy_aval_edges)))
    select1 = list(range(len(busy_aval_edges)))

    random.seed(10)
    random.shuffle(select0)
    random.shuffle(select1)

    select_not_busy = select0
    select_busy = select1


    depot = not_center_depot
    avail_edges = not_busy_aval_edges
    select_index = select_not_busy
    description = "6: depot in not center area, tasks distribute in busy area"
    xmlpath = "dcarp/scenario6.xml"

    net = sumolib.net.readNet("scenario/DCC.net.xml")
    root = ET.Element('info', {'description': description})
    depot_from_node = net.getEdge(depot[1]).getFromNode().getID()
    depot_to_node = net.getEdge(depot[1]).getToNode().getID()

    ET.SubElement(root,'depot', {'id':'incoming', 'edge': depot[1], 'from_node': depot_from_node, 'from_index':str(node_dict[depot_from_node]), 'to_node':depot_to_node, 'to_index':str(node_dict[depot_to_node])}) # depot_out: 0
    ET.SubElement(root,'depot', {'id':'outgoing', 'edge': depot[0], 'from_node': depot_to_node, 'from_index':str(node_dict[depot_to_node]), 'to_node':depot_from_node,'to_index':str(node_dict[depot_from_node])}) # depot_in: 1

    ET.SubElement(root, 'capacity', {'value': str(200)})
    task_ele = ET.SubElement(root, "tasks")
    k = 0

    remove = [274, 427, 1873, 2051, 2642, 2643, 3413, 3414, 4129, 5297, 5298, 5299, 5563, 6277, 6347, 6348]

    for i in select_index:
        tsk_eid = avail_edges[i]
        tsk_demand = net.getEdge(tsk_eid).getLength() / 10
        if tsk_demand < 10 or tsk_demand > 100:
            continue

        tsk_id_index = edge_dict[tsk_eid]
        if tsk_id_index in remove:
            continue
        print(tsk_id_index)

        k += 1

        from_node = net.getEdge(tsk_eid).getFromNode().getID()
        to_node = net.getEdge(tsk_eid).getToNode().getID()

        # ET.SubElement(task_ele,'task', {'id':str(k), 'edge': tsk_eid, 'from_node': from_node, 'from_index':str(node_dict[from_node]), 'to_node':to_node,'to_index': str(node_dict[to_node]), 'demand':str(int(tsk_demand)), 'id_index':str(tsk_id_index),})

        if (k >= 220):
            break
    
    tree=ET.ElementTree(root)
    tree.write(xmlpath)
    pass



net = sumolib.net.readNet("DCC.net.xml")
edges = net.getEdges()

info = divide_area(edges)
edges_area = info[1]
# with open('edges_area_info.pickle', 'wb') as f:
#     pickle.dump(edges_area, f)

# area_edges = divide_area(edges)
# calculate_congestion(area_edges, edges)

# plot_area(area_edges)
# congest_analysis_by_plot()

# area_edges = divide_area(edges)
# generate_scenarios(area_edges)

