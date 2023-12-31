import time, copy, os, sys
import sumolib, traci
import pickle 
import subprocess
import xml.etree.ElementTree as ET
from utils.carp import depot_in, depot_out, gc
from utils.carp import task, routine, solution, dis_flag
from utils.carp import cal_route_cost_with_vt
import numpy as np
from utils.plot_net_selection import plot_route

ADD_TASK_FLAG = True

class sumo_dcarp_init():

    # pre-defined new tasks
    # TODO: put new tasks in the seperate scenario file => NOT NECCESSARY
    new_tasks_whole = [3810,5933,1001,5831,5917,1830,3728,278,2788,4590,4729,942,3617,4041,2618,554,5545,2981,2721,354]
    busy_area = [6513,5609,2056,540,3935,2220,1199,3474,4086,3685,4340,4200,3389,5355,2901,3399,1606,5114,178,4063]
    not_busy_area = [5708,941,1411,6288,5831,6400,1832,5752,2618,6582,1326,5081,6521,5707,5343,4176,483,2786,498,1361]

    @classmethod
    def init(self, filepath):
        fs = sumo_dcarp_init.init_solution(filepath)
        sumo_dcarp_init.add_routes(fs)
        disflags = dis_flag() # distance flags is used to record the distancew that vehicles have traveled
        return fs, disflags

    # obtain the initial solution from the scenario file
    @classmethod
    def init_solution(self, scenario):
        global edge_list, edge_property
        # scenario = "dcarp/scenario1.xml"
        tree = ET.ElementTree(file=scenario)
        init_solution = tree.getroot().find("solution")

        # get task sequence and the corresponding demand sequence
        x1 = list(init_solution.iter("task"))[0].attrib["seq"]
        x2 = list(init_solution.iter("demand"))[0].attrib["seq"]
        tasks_seqs = list(map(lambda x:int(x), x1.split()))
        demand_seqs = list(map(lambda x:int(x), x2.split()))

        # construct routes and solutions from task sequences
        fs = solution() # first solution
        route = routine()
        for t, d in zip(tasks_seqs[1:], demand_seqs[1:]):
            if t == 0:
                route.complete()
                fs.add_routine(route) # add the route to the solution
                del route
                route = routine() # generate a new empty route
                continue
            #set the road has already been a task
            eid = edge_list[t]
            edge_property[eid]["Task"] = True
            tt = task(eid, d) # constrcut a task
            route.add_tasks(tt) # add task to the current route
            traci.gui.toggleSelection(tt.edge,  objType='edge') # highlight the task edge in the GUI
        return fs


    # add routes of the initial solution to vehicles in the simulation
    @classmethod
    def add_routes(self, es): # es: the initial solution
        route_num = 0
        
        for k in range(len(es.routines)):
            route_edges_seqs = ()
            route = es.routines[k]
            task_seqs = route.tasks_seqs
            for i in range(len(task_seqs)-1):
                start = task_seqs[i].edge
                end = task_seqs[i+1].edge
                rr = traci.simulation.findRoute(start, end, routingMode=1)
                route_edges_seqs += rr.edges[:-1] # add the route edges into the path

            route_edges_seqs += (depot_in.edge, ) # add the depot_in edge form a complete route
            route_num += 1
            route_id = str(time.time()+route_num).replace('.', '')
            vechile_id = "nv" + str(route_num)
            traci.route.add(route_id, route_edges_seqs)
            # type = delivery (self defined in rou.xml)
            traci.vehicle.add(vechile_id, route_id, typeID="carp", depart=traci.simulation.getTime()+1) 
            traci.vehicle.setColor(vechile_id, (255,0,0))
            traci.vehicle.setParameter(vechile_id, "has.rerouting.device", "true")
            traci.vehicle.setParameter(vechile_id, "device.rerouting.period", 100000.0)
            gc.add_vehicle(vechile_id) # record the vehicle to the global variable
            gc.set_veh_load(vechile_id) # set the vehicle load to 0 (initial state)
            es.veh_route[vechile_id] = k
            es.routines[k].vid = vechile_id
            traci.gui.toggleSelection(vechile_id)
            # break
        return route_num


class sumo_dcarp():

    def __init__(self):
        pass
    
    @classmethod
    def detect_vehicles(self):
        for vid in gc.vehicles:
            try:
                traci.vehicle.getLaneID(vid)
            except:
                gc.remove_vehicle(vid)
    
    @classmethod
    def reschedule(self, scenario, instance, info_path="", added_tasks=[]):
        global disflags, fs, edge_list, edge_property, task_candidate_set, version
        # determine which tasks have been served first.
        served_tasks = {}
        for vid in disflags.flags.keys():
            if vid not in gc.vehicles:
                served_tasks[vid] = -1
            else:
                flags = disflags.flags[vid]
                task_num = len(flags)
                dis = traci.vehicle.getDrivingDistance(vid, depot_in.edge, 1)
                # dis >= flags[j]: not been served; dis < flags[j]: been served
                for j in range(1, task_num+1):
                    if dis >= flags[j-1]:
                        break
                served_tasks[vid] = j
        
        # construct virtual tasks 
        for vid in disflags.flags.keys():
            if served_tasks[vid] < 0:
                continue

            # 1. obtain stop locations

            edge_id = traci.vehicle.getRoadID(vid)
            stop_edge = net.getEdge(edge_id).getID()
            

            # 2. obtaining served load
            i = fs.veh_route[vid]
            for j in range(1, served_tasks[vid]):
                remove_tasks = fs.routines[i].tasks_seqs.pop(1)
                if gc.veh_load[vid] == 1:
                    gc.veh_load[vid] -= 1
                gc.veh_add_load(vid, remove_tasks.demand)
            
            # make virtual tasks' demand not equal to 0
            if  gc.veh_load[vid] == 0:
                gc.veh_load[vid] += 1
            # construct new solutions
            vt = task(stop_edge, gc.veh_load[vid], vt=True)
            fs.routines[i].tasks_seqs.insert(1, vt)


            # fs.routines[i].cost = cal_route_cost_with_vt(fs.routines[i].tasks_seqs)

        remove_idx = []
        for vid in disflags.flags.keys():
            if served_tasks[vid] < 0:
                remove_idx.append(fs.veh_route[vid])
        remove_idx.sort(reverse=True)
        
        for idx in remove_idx:
            fs.routines.pop(idx)

        fs.veh_route = {}
        for i in range(len(fs.routines)):
            fs.veh_route[fs.routines[i].vid] = i
            
        fs.complete()

        if ADD_TASK_FLAG and instance <= 5 and version == "dynamic":
            np.random.seed(int(time.time()))
            added_tasks = []
            # according to road's properties to add tasks
            # 1. filter the edges which have not been tasks
            for edge_idx in task_candidate_set[:]:
                edge_name = edge_list[edge_idx]
                if edge_property[edge_name]["Task"] == True:
                    task_candidate_set.remove(edge_idx)
            # 2. choose some tasks in the task_candidate_set
            remain_task_num = 0
            for i in range(len(fs.routines)):
                remain_task_num += (len(fs.routines[i].tasks_seqs)-2)
            # added_tasks_num = round(remain_task_num * 0.2)
            added_tasks_num = min(10, 200 - remain_task_num)
            if added_tasks_num > len(task_candidate_set):
                added_tasks_num = len(task_candidate_set)
            
            if len(task_candidate_set) > 0:
                # added_tasks = np.random.choice(task_candidate_set, added_tasks_num, replace=False)
                shuffel_index = np.random.permutation(len(task_candidate_set))
                for idx in shuffel_index:
                    if (len(added_tasks) == added_tasks_num):
                        break
                    new_task = task_candidate_set[idx]
                    edge_name = edge_list[new_task]
                    # if edge_name in ["28391756#0", "-40701185", "-114277935#1", "-64226715#3", "-23045780#1"]:
                    #     continue
                    rr = traci.simulation.findRoute(edge_name, edge_name)
                    serv_cost = rr.travelTime
                    if serv_cost > 1:
                        added_tasks.append(new_task)
                    
            else:
                print("No task can be added!")
            for new_task in added_tasks:
                edge_property[edge_list[new_task]]["Task"] = True
            
            info_path.write("t{0},{1}".format(instance, traci.simulation.getTime()))
            for new_task in added_tasks:
                info_path.write(",{0}".format(new_task))
            info_path.write("\n")
        
        if version == "static":
            # the added tasks is input
            pass
        

        if version == "dynamic":
            folder = "xml/scenario{0}_instance{1}_D".format(scenario, instance)
            solution_path = folder + "/new_solution.xml"
        
        if version == "static":
            folder = "xml/scenario{0}_instance{1}_S".format(scenario, instance)
            solution_path = folder + "/new_solution.xml"
        

        if os.path.isfile(solution_path):
            # load new solution and assign to the vehicle
            sumo_dcarp.load_new_schedule(solution_path)
        else:
            if not os.path.exists(folder):
                # calculate and save the distance matrix for the map
                sumo_dcarp.duaroute_cal_route_cost(folder, added_tasks)

                # save the current solution for local search
                remain_tasks_num = sumo_dcarp.parse_tasks_seq(folder, scenario, instance, added_tasks)

            if remain_tasks_num > 3:
                # call C program
                subprocess.call([r'./dcarp-sumo.exe', '-s', str(scenario), '-i', str(instance), '-v', version])

                # load new solution and assign to the vehicle
                sumo_dcarp.load_new_schedule(solution_path)
            else:
                for i in range(len(fs.routines)):
                    if (fs.routines[i].tasks_seqs[1].vt):
                        fs.routines[i].tasks_seqs.pop(1)



    

    @classmethod
    def duaroute_cal_route_cost(self, folder, added_tasks):
        global edge_list,fs

        os.mkdir(folder)

        for v in gc.vehicles:
            try:
                traci.vehicle.getColor(v)
            except:
                gc.vehicles.remove(v)
        

        root = ET.Element('meandata')
        for eid in edge_list[1:]:
            t = []
            for v in gc.vehicles:
                t.append(float(traci.vehicle.getParameter(v, "device.rerouting.edge:"+eid)))
            ET.SubElement(root,'edge', {'id':eid, 'traveltime':str(np.mean(t))})
            # rr = traci.simulation.findRoute(eid, eid)
            # ET.SubElement(root,'edge', {'id':eid, 'traveltime':str(rr.travelTime)})
        tree=ET.ElementTree(root)
        tree.write(folder+"/weights.xml")

        tasks = [depot_out.edge, depot_in.edge]
        for r in fs.routines:
            for tsk in r.tasks_seqs[1:-1]:
                tasks.append(tsk.edge)
        
        for tsk in added_tasks:
            tsk_eid = edge_list[tsk]
            traci.gui.toggleSelection(tsk_eid,  objType='edge')
            tasks.append(tsk_eid)
        
        # trips only consist of the route between each two tasks.
        # It doesn't consist of the route of other tasks
        root = ET.Element('routes')
        curr_time = str(traci.simulation.getTime())
        k = 0
        for src in tasks:
            k += 1
            dst = src
            ET.SubElement(root,'trip', {'id':str(k), 'depart': curr_time, 'from':src, 'to':dst})
        
        for src in tasks:
            for dst in tasks: 
                if src == dst:
                    continue
                k += 1
                ET.SubElement(root,'trip', {'id':str(k), 'depart': curr_time, 'from':src, 'to':dst})
        tree=ET.ElementTree(root)
        tree.write(folder+"/trips.xml")
        
        subprocess.call(["duarouter","--route-files", folder+"/trips.xml","--net-file","scenario/DCC.net.xml","--weight-files",folder+"/weights.xml","--bulk-routing","true", "--output-file",folder+"/result.rou.xml"])

    @classmethod
    def parse_tasks_seq(self, folder, scenario, instance, added_tasks):
        global fs,net
        root = ET.Element('info', {'scenario': str(scenario), 'instance':str(instance)})
        depot_from_node = net.getEdge(depot_out.edge).getFromNode().getID()
        depot_to_node = net.getEdge(depot_out.edge).getToNode().getID()
        ET.SubElement(root,'depot', {'id':'0', 'edge': depot_out.edge, 'from_node': depot_from_node, 'from_index':str(node_dict[depot_from_node]), 'to_node':depot_to_node, 'to_index':str(node_dict[depot_to_node])}) # depot_out: 0
        ET.SubElement(root,'depot', {'id':'1', 'edge': depot_in.edge, 'from_node': depot_to_node, 'from_index':str(node_dict[depot_to_node]), 'to_node':depot_from_node,'to_index':str(node_dict[depot_from_node])}) # depot_in: 1

        ET.SubElement(root, 'capacity', {'value':str(gc.capacity)})

        k = 0
        remain_tasks_num = 0
        sol = ET.SubElement(root, 'solution')
        for vid in fs.veh_route:
            if vid not in gc.vehicles:
                continue
            r = fs.routines[fs.veh_route[vid]]
            # if all tasks in the route have been served and the remaining edges smaller than 10 edges. ignore this route. 
            route_edges = traci.vehicle.getRoute(vid)
            curr_idx = traci.vehicle.getRouteIndex(vid)
            if (len(route_edges) - curr_idx < 10) and len(r.tasks_seqs) == 3:
                continue
            
            rt = ET.SubElement(sol, 'route')
            for tsk in r.tasks_seqs[1:-1]:
                k += 1
                from_node = net.getEdge(tsk.edge).getFromNode().getID()
                to_node = net.getEdge(tsk.edge).getToNode().getID()
                if tsk.vt:
                    vt = vid[2:]
                else:
                    vt = '0'
                    remain_tasks_num += 1
                ET.SubElement(rt,'task', {'id':str(k), 'edge': tsk.edge, 'from_node': from_node, 'from_index':str(node_dict[from_node]), 'to_node':to_node,'to_index': str(node_dict[to_node]), 'demand':str(tsk.demand), 'id_index':str(edge_dict[tsk.edge]), 'vt':vt})

        if (len(added_tasks) > 0):
            rt = ET.SubElement(sol, 'route', {"property":"new"})
            for tsk in added_tasks:
                k += 1
                tsk_id = edge_list[tsk]
                from_node = net.getEdge(tsk_id).getFromNode().getID()
                to_node = net.getEdge(tsk_id).getToNode().getID()
                tsk_demand = int(net.getEdge(tsk_id).getLength() / 5)
                vt = '0'
                remain_tasks_num += 1
                ET.SubElement(rt,'task', {'id':str(k), 'edge': tsk_id, 'from_node': from_node, 'from_index':str(node_dict[from_node]), 'to_node':to_node,'to_index': str(node_dict[to_node]), 'demand':str(tsk_demand), 'id_index':str(edge_dict[tsk_id]), 'vt':vt})

        tree=ET.ElementTree(root)
        tree.write(folder+"/solution.xml")
        return remain_tasks_num

    @classmethod
    def load_new_schedule(self, solution_path):
        global fs
        new_sol = solution()
        
        tree = ET.ElementTree(file=solution_path)
        root = tree.getroot()
        for route in root:
            new_route = routine()

            route_tasks = list(route)

            vtsk = route_tasks.pop(0)
            e = edge_list[int(vtsk.attrib["IDindex"])]
            vt = int(vtsk.attrib["vt"])

            route_edges = []
            if (vt > 0):
                new_route.tasks_seqs.pop(0)
                vid = "nv"+str(vt)
            
            if (vt == 0): # assign a new vehicle
                vid = "nv" + str(int(gc.vehicles[-1][2:])+1)
                route_edges.append(depot_out.edge)
                gc.add_vehicle(vid)
                gc.set_veh_load(vid)
                demand = int(vtsk.attrib["demand"])
                new_route.add_tasks(task(e, demand))
                
            route_edges.append(e)

            for tsk in route_tasks:
                e = edge_list[int(tsk.attrib["IDindex"])]
                demand = int(tsk.attrib["demand"])
                new_route.add_tasks(task(e, demand))
                route_edges.append(e)
            
            route_edges.append(depot_in.edge)
            route_edges_paths = []
            for i in range(len(route_edges)-1):
                start = route_edges[i]
                end = route_edges[i+1]
                rr = traci.simulation.findRoute(start, end, routingMode=1)
                route_edges_paths += rr.edges[:-1]
            route_edges_paths.append(depot_in.edge)
            

            if (vt == 0):
                route_id = "new" + str(time.time())
                traci.route.add(route_id, route_edges_paths)
                traci.vehicle.add(vid, route_id, typeID="carp", depart=traci.simulation.getTime()+1) 
                traci.vehicle.setColor(vid, (255,0,0))
                traci.vehicle.setParameter(vid, "has.rerouting.device", "true")
                traci.vehicle.setParameter(vid, "device.rerouting.period", 100000.0)
                traci.gui.toggleSelection(vid)
            
            traci.vehicle.setRoute(vid, route_edges_paths)
            new_route.complete()
            new_route.tasks_seqs.insert(0, depot_out)
            new_route.vid = vid
            new_sol.add_routine(new_route)
            new_sol.veh_route[vid] = len(new_sol.routines)-1

        
        for vid in new_sol.veh_route:
            if vid in fs.veh_route:
                fs.routines[fs.veh_route[vid]] = copy.deepcopy(new_sol.routines[new_sol.veh_route[vid]])
            else:
                fs.add_routine(new_sol.routines[new_sol.veh_route[vid]])
                fs.veh_route[vid] = len(fs.routines) - 1


# remove vehicles which have returned to the depot
def remove_return_vehicle():
    veh_num = len(gc.vehicles)
    for i in range(veh_num):
        vid = gc.vehicles[i]
        try:
            traci.vehicle.getLength(vid) # it can't retrieval vehicle's information if it returns to the depot
        except:
            gc.vehicles[i] = None
    gc.vehicles = list(filter(lambda vid: vid != None, gc.vehicles)) # remove from our records

def is_all_not_in_juction():
    for v in gc.vehicles:
        lid = traci.vehicle.getLaneID(v)
        if (len(lid) == 0):
            return False
        if lid[0] == ":":
            return False
    return True

def is_all_start():
    for v in gc.vehicles:
        l = traci.vehicle.getDistance(v)
        if l < 0:
            return False
    return True
        
'''
getDistance: Returns the distance to the starting point like an odometer.
getDrivingDistance: Return the distance to the given edge and position along the vehicles route.
getDrivingDistance: To the start of the edge
'''
def set_task_dis_flag1():
    global fs, disflags
    disflags.clear()
    for vid in gc.vehicles:
        dd = traci.vehicle.getDrivingDistance(vid, depot_in.edge, 1)

        idx = fs.veh_route[vid]
        task_seqs = fs.routines[idx].tasks_seqs
        distance_for_task = []
        for i in range(1, len(task_seqs) - 1):
            # calculate the distance of each task to the incoming depot to help calculate tasks which have been served
            dd1 = dd - traci.vehicle.getDrivingDistance(vid, task_seqs[i].edge, 0)
            distance_for_task.append(dd1)
        distance_for_task.append(0)
        disflags.add_dis_flags(vid, distance_for_task)


def hidden_served_task():
    for vid in gc.vehicles:
        try:
            e = traci.vehicle.getRoadID(vid)
            if e == '':
                continue
            if traci.gui.isSelected(e,  objType='edge'):
                traci.gui.toggleSelection(e,  objType='edge')
        except:
            pass

class TestListener(traci.StepListener):

    def __init__(self, scenario):
        self.scenario = scenario

    
    def step(self, t=0):
        pass

class DCARPListener(traci.StepListener):
    
    def __init__(self, scenario):
        self.period = 1000
        self.detect_freq = 30
        self.count = 0
        self.flag1 = True
        self.flag2 = False
        self.cost = [0]*5
        self.fo = open("output/cost_change"+str(scenario)+".txt", "w")
        self.dynamic_time = open("xml/timepoint"+str(scenario)+".txt", "w")
        self.net_fig = None
        self.net_ax = None
        self.instance = 0
        self.scenario = scenario

    def step(self, t=0.0):
        self.count += 1
        remove_return_vehicle()
        hidden_served_task()
        # accumulate cost used

        if self.instance >= 10:
            return True

        if self.flag1: # flag1 is used to control if step into the following condition
            self.flag2 = is_all_start()
            if self.flag2:
                # calculate the distanc and distance flag
                # calculate the distance of each task to the incoming depot to help calculate tasks which have been served
                set_task_dis_flag1()
                self.flag1 = False
                # when all vehicles start, record cost one time.
                self.cost = [0]*5
                cost = self.detect_cost_change()
                self.cost.pop(0)
                self.cost.append(cost)

            else: # it still has the vehicle which has not started, so we need to wait
                return True
        
        # flag2 is used to refelect if all vehicles have started
        if (self.count % self.detect_freq) == 0 and self.flag2:

            if not is_all_not_in_juction():
                self.count -= 1
                return True

            cost = self.detect_cost_change()
            if cost / self.cost[-1] > 4:
                return True

            # if self.flag2 == False:
            #     t_now = traci.simulation.getTime()
            #     self.fo.write("{0},{1}\n".format(t_now, cost))

            self.cost.pop(0)
            self.cost.append(cost)
            print(self.cost)         
            if self.cost[0]!=0 and self.cost[-1] > self.cost[0] * 1.05 and self.flag2:

                cost = self.cost[-1]
                t_now = traci.simulation.getTime()
                self.fo.write("{0},{1}\n".format(t_now, cost))

                self.instance += 1
                print("scenario: ", self.scenario, "instance: ", self.instance, "current time:", t_now)
                #save the time for rescheduling for CARP without dynamic optimization
                # self.dynamic_time.write("t{0},{1}\n".format(self.instance, t_now))
                sumo_dcarp.reschedule(self.scenario, self.instance, info_path=self.dynamic_time)

                # for vid in gc.vehicles:
                #     route_edges_nv1 = traci.vehicle.getRoute(vid)
                #     self.net_fig, self.net_ax = route_visualization(route_edges_nv1, fig=self.net_fig, ax=self.net_ax)
                #     self.net_fig.savefig("output/img/{0}_{1}.png".format(vid, self.instance), dpi=600)

                cost = self.detect_cost_change() # calculate the cost after rescheduling
                
                self.fo.write("optimise,{0},{1}\n".format(t_now, cost))
    
                self.flag1 = True
                self.flag2 = False

        return True

    def detect_cost_change(self):
        # calculate cost and draw in the window
        remove_return_vehicle()
        cost_future = 0
        for vid in gc.vehicles:
            route_edges = traci.vehicle.getRoute(vid)
            curr_idx = traci.vehicle.getRouteIndex(vid)
            vcost = 0
            for eid in route_edges[curr_idx+1:]:
                vcost += float(traci.vehicle.getParameter(vid, "device.rerouting.edge:"+eid))
                # vcost1 += traci.edge.getAdaptedTraveltime(eid, traci.simulation.getTime()-2)
                # rr = traci.simulation.findRoute(eid, eid, routingMode=traci.constants.ROUTING_MODE_AGGREGATED)
                # vcost1 += rr.travelTime
            cost_future += vcost
            # if (vid == "nv18" and vcost > 50000):
            #     vcost_tmp = 0
            #     for eid in route_edges[curr_idx+1:]:
            #         ecost = float(traci.vehicle.getParameter(vid, "device.rerouting.edge:"+eid))
            #         vcost_tmp += ecost
            #         print(eid, ecost)
            # print(vid, vcost, vcost1)
        # print("cost_future", cost_future)
        return cost_future
    
    def vis_route_in_gui(self):
        route_edges_nv1 = traci.vehicle.getRoute('nv1')
        for e in route_edges_nv1:
            traci.gui.toggleSelection(e,  objType='edge')


class SCARPListener(traci.StepListener):
    
    def __init__(self, scn_idx) -> None:
        self.count = 0
        self.flag1 = True
        self.flag2 = False
        self.scenario = scn_idx
        self.instance = 0
        self.timepoint = []
        self.new_tasks = []
        self.load_added_tasks_time(scn_idx=scn_idx)
        
        
    def load_added_tasks_time(self, scn_idx):
        with open("xml/timepoint"+str(scn_idx)+".txt", 'r') as f:
            # self.added_tasks_time_points = [float(t.strip().split(',')[1]) for t in f.readlines()]
            ll = f.readlines()
            for t in ll:
                info = t.strip().split(',')
                self.timepoint.append(float(info[1]))
                self.new_tasks.append([int(x) for x in info[2:-1]])
                # self.added_tasks_info[timepoint] = new_tasks

    # def step(self, t=0): # added tasks one by one
    #     self.count += 1
    #     hidden_served_task()
    #     # print(self.count)
    #     if self.count <= 200:
    #         return True
        
    #     # added tasks and insert tasks
    #     t_now = traci.simulation.getTime()
    #     if (t_now in self.added_tasks_info):
    #         self.instance += 1
    #         sumo_dcarp.reschedule(self.scenario, self.instance, added_tasks=self.new_tasks)

    #     return True

    def step(self, t=0.0):
        remove_return_vehicle()
        hidden_served_task()
        # accumulate cost used

        if self.flag1: # flag1 is used to control if step into the following condition
            self.flag2 = is_all_start()
            if self.flag2:
                # calculate the distanc and distance flag
                # calculate the distance of each task to the incoming depot to help calculate tasks which have been served
                set_task_dis_flag1()
                self.flag1 = False
            else: # it still has the vehicle which has not started, so we need to wait
                return True
        
        t_now = traci.simulation.getTime()
        if self.instance >= len(self.timepoint):
            return True
        
        # flag2 is used to refelect if all vehicles have started
        if self.flag2 and t_now >= self.timepoint[self.instance]:
            if not is_all_not_in_juction():
                self.count -= 1
                return True
            t_now = traci.simulation.getTime()
            print("scenario: ", self.scenario, "instance: ", self.instance, "current time:", t_now)
            sumo_dcarp.reschedule(self.scenario, self.instance+1, added_tasks=self.new_tasks[self.instance])
            self.instance += 1

            # for vid in gc.vehicles:
            #     route_edges_nv1 = traci.vehicle.getRoute(vid)
            #     self.net_fig, self.net_ax = route_visualization(route_edges_nv1, fig=self.net_fig, ax=self.net_ax)
            #     self.net_fig.savefig("output/img/{0}_{1}.png".format(vid, self.instance), dpi=600)

            self.flag1 = True
            self.flag2 = False

        return True



png_index = -1
def route_visualization(route_edges, fig=None, ax=None):
    global png_index
    png_index += 1
    args = ["-n", "scenario/DCC.net.xml", "--xlim", "-100,7200", "--ylim", "-100,5400", "--xticks", "-100,7201,2000,16", "--yticks", "-100,5401,1000,16", "--selected-width", "2", "--edge-width", ".5", "--edge-color", "#606060", "--selected-color", "#800000", "--online", "on"] #"-o", "output/route_for_vid1_"+str(png_index)+".png",
    n_fig, n_ax = plot_route(route_edges, args, fig, ax)
    return n_fig, n_ax



# if (len(sys.argv) == 1):
#     raise ValueError("please input scenario index")
# scenario = int(sys.argv[1])

# version = "static"
# scenario = 1
try:
    version = sys.argv[1]
    scenario = int(sys.argv[2])
    print("version:", version)
except:
    raise ValueError('Lack of the version of scheduling: dynamic or static?')


# main program
with open('traffic/dublin.pickle', 'rb') as f:
    map_data = pickle.load(f)

node_dict = map_data[0] # node_dict[node_name]=node_idx
node_list = map_data[1] # node_list[node_idx]=node_name
edge_dict = map_data[2] # edge_dict[edge_name]=edge_idx
edge_list = map_data[3] # edge_list[edge_idx]=edge_name

#give each road two properties: 1). belong to busy area or not, 2) if it has already been a task
with open('traffic/edges_area_info.pickle', 'rb') as f:
    edges_area = pickle.load(f) # edges_area[edge_name]= "busy" or "uncrowded"

with open('task_tabu_list.txt', 'r') as f:
    task_tabu_list = f.read().splitlines()


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

edge_property = {}
task_candidate_set = []
for edge_name in edge_dict:
    if edge_name in task_tabu_list:
        continue

    d = {"id": edge_name, "Busy": edges_area[edge_name], "Task": False}
    edge_property[edge_name] = d

    if TASK_DISTRIBUTION == 1: # whole map
        task_candidate_set.append(edge_dict[edge_name])
    elif (TASK_DISTRIBUTION == 2 and edges_area[edge_name] == "busy"): # only busy area
        task_candidate_set.append(edge_dict[edge_name])
    elif (TASK_DISTRIBUTION == 3 and edges_area[edge_name] == "uncrowded"): # only not busy area
        task_candidate_set.append(edge_dict[edge_name])

# set the time setting of the simulation: begin, end, step
time_setting = info.find("time")
begin_time = time_setting.attrib["begin"]
end_time = time_setting.attrib["end"]
step_length = time_setting.attrib["step"]

# set depot according to the scenario
depot_setting = info.findall("depot")
if (depot_setting[0].attrib["id"] == "incoming"):
    depot_in.set_depot(depot_setting[0].attrib["edge"])
    depot_out.set_depot(depot_setting[1].attrib["edge"])

if (depot_setting[0].attrib["id"] == "outgoing"):
    depot_in.set_depot(depot_setting[1].attrib["edge"])
    depot_out.set_depot(depot_setting[0].attrib["edge"])


cap_setting = info.findall("capacity")[0]

# set the capacity of the vehicle
gc.set_cap(int(cap_setting.attrib["value"])+1)

# set thr edge map of the graph: edge id -> edge index
gc.set_edge_map(edge_dict)

# load the map and the traffic data
net = sumolib.net.readNet("scenario/DCC.net.xml")
depot_coord = net.getEdge(depot_out.edge).getFromNode().getCoord()
traci.start(["sumo-gui", "-c", "scenario/DCC_simulation.sumo.cfg", "--begin", begin_time, "--end", end_time, "--step-length", step_length, "--start", "true"]) #, "--start", "true"
traci.poi.add('depot', depot_coord[0], depot_coord[1], (1,0,0,0))


# obtain the initial solution and assign vehicles with correspongding scheduling path to the simulation
fs, disflags = sumo_dcarp_init.init(scenario_file)


# the above is the initial process
if version == "dynamic":
    listener = DCARPListener(scenario) # scenario index

if version == "static":
    listener = SCARPListener(scenario)

if version == "test":
    listener = TestListener(scenario)

traci.addStepListener(listener)


while len(gc.vehicles) > 0:
    remove_return_vehicle()
    traci.simulationStep()


with open("output/simulationStepStatic.txt", "a") as f:
    if version == "static":
        s = "scenario{0} S ".format(scenario)
    if version == "dynamic":
        s = "scenario{0} D ".format(scenario)

    s += time.strftime(r"%Y-%m-%d:%H:%M:%S", time.localtime()) 
    s +=  ": " + str(begin_time) + " to " + str(traci.simulation.getTime())
    s += "\n"
    f.write(s)
    print(s)
    
traci.close()
if version == "dynamic":
    listener.fo.close()
    listener.dynamic_time.close()
