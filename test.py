import subprocess

# subprocess.call([r'./dcarp-sumo.exe', '-s', str(1), '-i', str(1)])
task_tabu_list = []
with open('task_tabu_list.txt', 'r') as f:
    task_tabu_list = f.read().splitlines()

print(task_tabu_list)


# net = sumolib.net.readNet("scenario/DCC.net.xml")
# edges = net.getEdges()
# for e in edges:
#     eid = e.getID()
#     rr = traci.simulation.findRoute(depot_out.edge, eid)
#     if (len(rr.edges) == 0):
#         print(eid)

# exit(-1)