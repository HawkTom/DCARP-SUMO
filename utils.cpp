
#include "globalvar.h"
#include "src/src.h"
#include "libs/tinyxml2.h"
#include <math.h>
using namespace tinyxml2;


void construct_edge_node_map(std::map<std::string, Arc> m);

// read tasks from xml file
// init min_costs
void init(Task *inst_tasks, CARPInd *solution)
{
    std::map<std::string, Arc> Arcs;
    // read tasks from xml file
    const char* arc_id;

    char path0[50];
    if (strcmp(version, "dynamic") == 0)
    {
        sprintf(path0, "xml/scenario%d_instance%d_D/solution.xml", scenario_idx, instance_idx);
    }
    if (strcmp(version, "static") == 0)
    {
        sprintf(path0, "xml/scenario%d_instance%d_S/solution.xml", scenario_idx, instance_idx);
    }   


    XMLDocument doc;
    doc.LoadFile(path0);
    XMLElement* info = doc.FirstChildElement( "info" );

    printf("scenario: %d; instance: %d\n", scenario_idx, instance_idx);

    XMLElement* depot_out = info->FirstChildElement( "depot" );
    XMLElement* depot_in = depot_out->NextSiblingElement();

    int node_to_node_r[MAX_NODE_TAG_LENGTH];
    memset(node_to_node_r, 0, sizeof(node_to_node_r));

    inst_tasks[0].head_node = 1;
    inst_tasks[0].tail_node = 1;
    inst_tasks[0].head_node_r = atoi(depot_out->Attribute("from_index"));
    inst_tasks[0].tail_node_r = inst_tasks[0].head_node_r;

    DEPOT = 1;
    DEPOT_R = inst_tasks[0].head_node_r;

    node_to_node_r[1] = DEPOT_R;

    Arc _arc;
    arc_id = depot_out->Attribute("edge");
    _arc.head_node_r = inst_tasks[0].head_node_r;
    _arc.tail_node_r = atoi(depot_out->Attribute("to_index"));
    Arcs[arc_id] = _arc;

    arc_id = depot_in->Attribute("edge");
    _arc.tail_node_r = inst_tasks[0].head_node_r;
    _arc.head_node_r = atoi(depot_out->Attribute("to_index"));
    Arcs[arc_id] = _arc;

    XMLElement* cap = doc.FirstChildElement( "info" )->FirstChildElement( "capacity" );
    capacity = atoi(cap->Attribute("value"));

    // CARPInd solution;
    memset(solution->Sequence, 0, sizeof(solution->Sequence));
    memset(solution->Loads, 0, sizeof(solution->Loads));
    solution->Sequence[0] = 0;
    int load = 0;
    solution->Loads[0] = -1;

    int idx, demand, IDindex, vt;
    int head_node_r, tail_node_r;
    XMLElement* slt = doc.FirstChildElement( "info" )->FirstChildElement( "solution" );
    XMLElement* curr_route = slt->FirstChildElement( "route" );

    int stop[101];
    int remain_capacity[101];
    memset(stop, 0, sizeof(stop));
    memset(remain_capacity, 0, sizeof(remain_capacity));

    req_edge_num = 0;
    memset(new_tasks_list, 0, sizeof(new_tasks_list));

    int node_index = 1;

    while (true)
    {
        XMLElement* curr_task = curr_route->FirstChildElement( "task" );
        XMLElement* next_task;
        solution->Sequence[0] ++;
        solution->Sequence[solution->Sequence[0]] = 0;
        solution->Loads[0] ++;
        solution->Loads[solution->Loads[0]] = load;
        load = 0;

        while (true)
        {   
            arc_id = curr_task->Attribute("edge");
            idx = atoi(curr_task->Attribute("id"));
            head_node_r = atoi(curr_task->Attribute("from_index"));
            tail_node_r = atoi(curr_task->Attribute("to_index"));
            demand = atoi(curr_task->Attribute("demand"));
            IDindex = atoi(curr_task->Attribute("id_index"));
            vt = atoi(curr_task->Attribute("vt"));

            if (vt)
            {
                inst_tasks[idx].head_node = DEPOT;
                inst_tasks[idx].head_node_r = DEPOT_R;
            } else {
                inst_tasks[idx].head_node = (++node_index);
                inst_tasks[idx].head_node_r = head_node_r;
            }

            inst_tasks[idx].tail_node = (++node_index);
            inst_tasks[idx].tail_node_r = tail_node_r;
            inst_tasks[idx].demand = demand;
            inst_tasks[idx].vt = vt;
            inst_tasks[idx].IDindex = IDindex;
            inst_tasks[idx].inverse = -1; // be careful for using the inverse to calculate the number of tasks
            inst_tasks[idx].serv_cost = 0; // ------------------------ 

            node_to_node_r[inst_tasks[idx].head_node] = inst_tasks[idx].head_node_r;
            node_to_node_r[inst_tasks[idx].tail_node] = inst_tasks[idx].tail_node_r;

            if (vt)
            {
                stop[0] ++;
                stop[stop[0]] = inst_tasks[idx].tail_node;
                remain_capacity[0] ++;
                remain_capacity[remain_capacity[0]] = capacity - demand;
            }

            solution->Sequence[0] ++;
            solution->Sequence[solution->Sequence[0]] = idx;
            load += demand;

            req_edge_num ++;
            
            Arc _arc;
            _arc.head_node_r = head_node_r;
            _arc.tail_node_r = tail_node_r;
            if (vt)
            {
                _arc.head_node = -1;  
            } else {
                _arc.head_node = inst_tasks[idx].head_node;  
            }
            _arc.tail_node = inst_tasks[idx].tail_node;
            Arcs[arc_id] = _arc;
            
            next_task = curr_task->NextSiblingElement();
            if (next_task == 0) break;
            curr_task = next_task;
        }

        // read new tasks
        XMLElement* next_route = curr_route->NextSiblingElement();
        if (next_route == 0) break;
        if (next_route->Attribute("property"))
        {
            curr_route = next_route;
            curr_task = curr_route->FirstChildElement( "task" );
            while (true)
            {   
                arc_id = curr_task->Attribute("edge");
                idx = atoi(curr_task->Attribute("id"));
                head_node_r = atoi(curr_task->Attribute("from_index"));
                tail_node_r = atoi(curr_task->Attribute("to_index"));
                demand = atoi(curr_task->Attribute("demand"));
                IDindex = atoi(curr_task->Attribute("id_index"));
                vt = atoi(curr_task->Attribute("vt"));


                inst_tasks[idx].tail_node_r = tail_node_r;
                inst_tasks[idx].head_node_r = head_node_r;
                inst_tasks[idx].head_node = ++node_index;
                inst_tasks[idx].tail_node = ++node_index;
                inst_tasks[idx].demand = demand;
                inst_tasks[idx].vt = vt;
                inst_tasks[idx].IDindex = IDindex;
                inst_tasks[idx].inverse = -1;
                inst_tasks[idx].serv_cost = 0; // ------------------------ 

                node_to_node_r[inst_tasks[idx].head_node] = inst_tasks[idx].head_node_r;
                node_to_node_r[inst_tasks[idx].tail_node] = inst_tasks[idx].tail_node_r;                

                new_tasks_list[0]++;
                new_tasks_list[new_tasks_list[0]] = idx;

                req_edge_num ++;
                Arc _arc;

                _arc.head_node_r = head_node_r;
                _arc.tail_node_r = tail_node_r;
                _arc.head_node = inst_tasks[idx].head_node;
                _arc.tail_node = inst_tasks[idx].tail_node;
                
                Arcs[arc_id] = _arc;

                next_task = curr_task->NextSiblingElement();
                if (next_task == 0) break;
                curr_task = next_task;
            }
            next_route = curr_route->NextSiblingElement();
        }
        if (next_route == 0) break;
        curr_route = next_route;
    }
    solution->Sequence[0] ++;
    solution->Sequence[solution->Sequence[0]] = 0;
    solution->Loads[0] ++;
    solution->Loads[solution->Loads[0]] = load;

    // make the inverse of a task is same with itself
    for (int i = 1; i <= req_edge_num; i++)
    {
        inst_tasks[i].inverse = i+req_edge_num;
        inst_tasks[i+req_edge_num].inverse = i;
        inst_tasks[i].serv_cost = 0;
        inst_tasks[i+req_edge_num].head_node = inst_tasks[i].head_node;
        inst_tasks[i+req_edge_num].tail_node = inst_tasks[i].tail_node;
        inst_tasks[i+req_edge_num].head_node_r = inst_tasks[i].head_node_r;
        inst_tasks[i+req_edge_num].tail_node_r = inst_tasks[i].tail_node_r;
        inst_tasks[i+req_edge_num].demand = inst_tasks[i].demand;
        inst_tasks[i+req_edge_num].vt = inst_tasks[i].vt;
        inst_tasks[i+req_edge_num].IDindex = inst_tasks[i].IDindex;
    }
    

    task_num = req_edge_num*2;


    // init min_cost_r
    construct_edge_node_map(Arcs);
    // convert min_cost_r to min_cost
    memset(min_cost, 0, sizeof(min_cost));
    for (int i = 1; i <= node_index; i++)
    {
        for (int j = 1; j <= node_index; j++)
        {
            min_cost[i][j] = min_cost_r[node_to_node_r[i]][node_to_node_r[j]];
        }
    }

    for (int i = 1; i <= req_edge_num; i++)
    {
        inst_tasks[i].serv_cost = min_cost[inst_tasks[i].head_node][inst_tasks[i].tail_node];
        inst_tasks[i+req_edge_num].serv_cost = inst_tasks[i].serv_cost;

        inst_tasks[i].dead_cost = inst_tasks[i].serv_cost;
        inst_tasks[i+req_edge_num].dead_cost = inst_tasks[i].dead_cost;
    }

    if (new_tasks_list[0] == 0)
    {
        solution->TotalCost = get_task_seq_total_cost(solution->Sequence, inst_tasks);
    }
    int a=1;
    int b=1;
}

void construct_edge_node_map(std::map<std::string, Arc> m)
{
    char path[50], path1[50];
    if (strcmp(version, "dynamic") == 0)
    {
        sprintf(path, "xml/scenario%d_instance%d_D/result.rou.alt.xml", scenario_idx, instance_idx);
        sprintf(path1, "xml/scenario%d_instance%d_D/trips.xml", scenario_idx, instance_idx);
    }

    if (strcmp(version, "static") == 0)
    {
        sprintf(path, "xml/scenario%d_instance%d_S/result.rou.alt.xml", scenario_idx, instance_idx);
        sprintf(path1, "xml/scenario%d_instance%d_S/trips.xml", scenario_idx, instance_idx);
    }

    XMLDocument doc, doc1;
    doc.LoadFile(path);
    doc1.LoadFile(path1);

    const char* rid;
    const char* cost;
    const char* rid1;
    XMLElement* curr = doc.FirstChildElement( "routes" )->FirstChildElement( "vehicle" );
    XMLElement* next;
    XMLElement* next1;

    XMLElement* curr1 = doc1.FirstChildElement( "routes" )->FirstChildElement( "trip" );
    const char* from_edge;
    const char* to_edge;
    std::map<std::string, Arc> ::iterator l_it, l_it_to;

    int head_r, tail_r, head_to_r, tail_to_r, tmp_cost;
    memset(min_cost_r, -1, sizeof(min_cost_r));
    for (int i = 0; i <= NODE_NUM; i++)
    {
        min_cost_r[i][i] = 0;
    }

    while (true)
    {
        rid = curr->Attribute( "id" );
        rid1 = curr1->Attribute("id");
        if (strcmp(rid, rid1) != 0)
        {
            printf("error xml\n");
            exit(-1);
        }
        cost = curr->FirstChildElement()->FirstChildElement()->Attribute("cost");
        from_edge = curr1->Attribute("from");
        to_edge = curr1->Attribute("to");
        l_it = m.find(from_edge);
        head_r = l_it->second.head_node_r;
        tail_r = l_it->second.tail_node_r;
        tmp_cost = round(atof(cost));
        if (strcmp(from_edge, to_edge) == 0)
        {
            /* code */
            min_cost_r[head_r][tail_r] = tmp_cost;
            // cout << head << "\t" << tail << "\t" << tmp_cost << endl;
        } else {
            /* code */
            l_it_to = m.find(to_edge);
            head_to_r = l_it_to->second.head_node_r;
            tail_to_r = l_it_to->second.tail_node_r;
            if (min_cost_r[head_r][head_to_r] < 0) min_cost_r[head_r][head_to_r] = tmp_cost - min_cost_r[head_to_r][tail_to_r];
            if (min_cost_r[head_r][tail_to_r] < 0) min_cost_r[head_r][tail_to_r] = tmp_cost;
            if (min_cost_r[tail_r][head_to_r] < 0) min_cost_r[tail_r][head_to_r] = tmp_cost - min_cost_r[head_r][tail_r] - min_cost_r[head_to_r][tail_to_r];
            if (min_cost_r[tail_r][tail_to_r] < 0) min_cost_r[tail_r][tail_to_r] = tmp_cost - min_cost_r[head_r][tail_r];
        }
        // routes[rid] = atof(cost);
        next = curr->NextSiblingElement();
        next1 = curr1->NextSiblingElement();
        if (next == 0 || next1 == 0) break;
        curr = next;
        curr1 = next1;
    }
}


void save_solution_xml(const CARPInd new_solution, const Task *inst_tasks)
{
    XMLDocument doc;
    XMLElement* root = doc.NewElement("solution");
    root->SetAttribute("num", new_solution.Loads[0]);
    doc.InsertFirstChild(root);
    
    XMLElement* route;
    XMLElement* task;
    int i, j, k;
    k = 0;
    for (i = 1; i < new_solution.Sequence[0]; i++)
    {   
        j = new_solution.Sequence[i];
        if ( j == 0)
        {
            k ++;
            route = root->InsertNewChildElement("route");
            route->SetAttribute("index", k);
            continue;
        }
        task = route->InsertNewChildElement("task");
        task->SetAttribute("IDindex", inst_tasks[j].IDindex);
        task->SetAttribute("vt", inst_tasks[j].vt);
        task->SetAttribute("demand", inst_tasks[j].demand);
    }
    char path[50];
    if (strcmp(version, "dynamic") == 0)
    {
        sprintf(path, "xml/scenario%d_instance%d_D/new_solution.xml", scenario_idx, instance_idx);
    }

    if (strcmp(version, "static") == 0)
    {
        sprintf(path, "xml/scenario%d_instance%d_S/new_solution.xml", scenario_idx, instance_idx);
    }
    doc.SaveFile(path);

}
