import subprocess, sys
import xml.etree.ElementTree as ET


# try:
#     version = sys.argv[1]
#     print("version:", version)
# except:
#     raise Exception('Lack of the version of scheduling: dynamic or static?')

# subprocess.call([r'./dcarp-sumo.exe', '-s', str(1), '-i', str(1)])
# task_tabu_list = []
# with open('task_tabu_list.txt', 'r') as f:
#     task_tabu_list = f.read().splitlines()

# print(task_tabu_list)

# x = {}
# with open("xml/timepoint"+str(1)+".txt", 'r') as f:
#     # dynamic_times = [float(t.strip().split(',')[1]) for t in f.readlines()]
#     ll = f.readlines()
#     for t in ll:
#         info = t.strip().split(',')
#         timepoint = float(info[1])
#         new_tasks = [int(x) for x in info[2:-1]]
#         x[timepoint] = new_tasks

# print(x)
# t_now = 33321
# if t_now in x:
#     print("yws")
# print(dynamic_times)

# net = sumolib.net.readNet("scenario/DCC.net.xml")
# edges = net.getEdges()
# for e in edges:
#     eid = e.getID()
#     rr = traci.simulation.findRoute(depot_out.edge, eid)
#     if (len(rr.edges) == 0):
#         print(eid)

# exit(-1)

for scenario in range(1, 13):
    scenario_file = "dcarp/scenario{0}.xml".format(scenario)
    tree = ET.ElementTree(file=scenario_file)
    info = tree.getroot()


    description = info.attrib["description"]

    TASK_DISTRIBUTION = 1 # 1: whole; 2: only busy area; 3: only not busy area
    if "whole" in description:
        TASK_DISTRIBUTION = 1 # 1: whole; 2: only busy area; 3: only not busy area
    if "busy" in description:
        TASK_DISTRIBUTION = 2
    if "uncrowded" in description:
        TASK_DISTRIBUTION = 3
    print(description, TASK_DISTRIBUTION)