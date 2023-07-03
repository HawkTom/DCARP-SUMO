#include "src.h"
#include <cstring>
#include "../libs/tinyxml2.h"
using namespace tinyxml2;

void readMap(Task *inst_tasks, Arc *inst_arcs, const char *map1, CARPInd *sol_seq)
{
    FILE *fp;

    char dummy[101];

    char path[50];
    memset(path, 0, sizeof(path));
    sprintf(path, "%s.dat", map1);

    fp = fopen(path, "r");
    if (fp == NULL)
    {
        printf("The file <%s> can't be open\n", path);
        exit(0);
    }

    while (fscanf(fp, "%s", dummy) != EOF)
    {

        if (strcmp(dummy, "VERTICES")==0)
        {
            fscanf(fp, "%s", dummy);
            fscanf(fp, "%d", &vertex_num);
        }
        else if (strcmp(dummy, "ARISTAS_REQ") == 0)
        {
            fscanf(fp, "%s", dummy);
            fscanf(fp, "%d", &req_edge_num);
        }
        else if (strcmp(dummy, "ARISTAS_NOREQ")==0)
        {
            fscanf(fp, "%s", dummy);
            fscanf(fp, "%d", &nonreq_edge_num);
        }
        else if (strcmp(dummy, "VEHICULOS")==0)
        {
            fscanf(fp, "%s", dummy);
            fscanf(fp, "%d", &vehicle_num);
        }
        else if (strcmp(dummy, "CAPACIDAD")==0)
        {
            fscanf(fp, "%s", dummy);
            fscanf(fp, "%d", &capacity);
        }
        else if (strcmp(dummy, "LISTA_ARISTAS_REQ")==0) {

            fscanf(fp, "%s", dummy);
            task_num = 2 * req_edge_num + req_arc_num;
            total_arc_num = task_num + 2 * nonreq_edge_num + nonreq_arc_num;
            for (int i = 1; i <= req_edge_num; i++) {
                fscanf(fp, "%s", dummy);
                fscanf(fp, "%d,", &inst_tasks[i].head_node);
                fscanf(fp, "%d)", &inst_tasks[i].tail_node);
                fscanf(fp, "%s", dummy);
                fscanf(fp, "%d", &inst_tasks[i].serv_cost);
                fscanf(fp, "%s", dummy);
                fscanf(fp, "%d", &inst_tasks[i].demand);

                inst_tasks[i].dead_cost = inst_tasks[i].serv_cost;
                inst_tasks[i].inverse = i + req_edge_num;
                inst_tasks[i].vt = 0;
                inst_tasks[i].head_node_r = 2*i-1;
                inst_tasks[i].tail_node_r = 2*i;

                inst_tasks[i + req_edge_num].head_node = inst_tasks[i].tail_node;
                inst_tasks[i + req_edge_num].tail_node = inst_tasks[i].head_node;
                inst_tasks[i + req_edge_num].dead_cost = inst_tasks[i].dead_cost;
                inst_tasks[i + req_edge_num].serv_cost = inst_tasks[i].serv_cost;
                inst_tasks[i + req_edge_num].demand = inst_tasks[i].demand;
                inst_tasks[i + req_edge_num].inverse = i;
                inst_tasks[i + req_edge_num].vt = 0;
                inst_tasks[i + req_edge_num].head_node_r = inst_tasks[i].tail_node_r;
                inst_tasks[i + req_edge_num].tail_node_r = inst_tasks[i].head_node_r;


                inst_arcs[i].head_node = inst_tasks[i].head_node;
                inst_arcs[i].tail_node = inst_tasks[i].tail_node;
                inst_arcs[i].trav_cost = inst_tasks[i].dead_cost;
                inst_arcs[i].base_cost = inst_arcs[i].trav_cost;
                inst_arcs[i + req_edge_num].head_node = inst_arcs[i].tail_node;
                inst_arcs[i + req_edge_num].tail_node = inst_arcs[i].head_node;
                inst_arcs[i + req_edge_num].trav_cost = inst_arcs[i].trav_cost;
                inst_arcs[i + req_edge_num].base_cost = inst_arcs[i].base_cost;

                if (costlb > inst_tasks[i].dead_cost)
                    costlb = inst_tasks[i].dead_cost;

                if (costub < inst_tasks[i].dead_cost)
                    costub = inst_tasks[i].dead_cost;

                if (demandlb > inst_tasks[i].demand)
                    demandlb = inst_tasks[i].demand;

                if (demandub < inst_tasks[i].demand)
                    demandub = inst_tasks[i].demand;

            }
        }
        else if (strcmp(dummy, "LISTA_ARISTAS_NOREQ")==0)
        {
            fscanf(fp, "%s", dummy);
            for (int i=task_num+1; i<=task_num+nonreq_edge_num;i++)
            {
                fscanf(fp, "%s", dummy);
                fscanf(fp, "%d,", &inst_arcs[i].head_node);
                fscanf(fp, "%d)", &inst_arcs[i].tail_node);
                fscanf(fp, "%s", dummy);
                fscanf(fp, "%d", &inst_arcs[i].trav_cost);
                inst_arcs[i].base_cost = inst_arcs[i].trav_cost;

                inst_arcs[i + nonreq_edge_num].head_node = inst_arcs[i].tail_node;
                inst_arcs[i + nonreq_edge_num].tail_node = inst_arcs[i].head_node;
                inst_arcs[i + nonreq_edge_num].trav_cost = inst_arcs[i].trav_cost;
                inst_arcs[i + nonreq_edge_num].base_cost = inst_arcs[i].base_cost;

                if (costlb > inst_arcs[i].trav_cost)
                    costlb = inst_arcs[i].trav_cost;

                if (costub < inst_arcs[i].trav_cost)
                    costub = inst_arcs[i].trav_cost;

            }
        }
        else if (strcmp(dummy, "DEPOSITO")==0)
        {
            fscanf(fp, "%s", dummy);
            fscanf(fp, "%d", &DEPOT);
        }
        else if(strcmp(dummy, "MAENSBEST")==0)
        {
            fscanf(fp, "%s", dummy);
            fscanf(fp, "%d", &sol_seq->TotalCost);
        }
        else if(strcmp(dummy, "MAENSSOL")==0)
        {
            fscanf(fp, "%s", dummy);
            fscanf(fp, "%d", &sol_seq->Sequence[0]);
            for (int i=1; i<=sol_seq->Sequence[0];i++)
            {
                fscanf(fp, "%d", &sol_seq->Sequence[i]);
            }
            int load = 0;
            for(int i=2; i<=sol_seq->Sequence[0]; i++)
            {
                if (sol_seq->Sequence[i] == 0)
                {
                    sol_seq->Loads[0]++;
                    sol_seq->Loads[sol_seq->Loads[0]] = load;
                    load = 0;
                    continue;
                }
                load += inst_tasks[sol_seq->Sequence[i]].demand;
            }
        }
    }

    fclose(fp);

    inst_tasks[0].head_node = DEPOT;
    inst_tasks[0].tail_node = DEPOT;
    inst_tasks[0].head_node_r = 0;
    inst_tasks[0].tail_node_r = 0;
    inst_tasks[0].dead_cost = 0;
    inst_tasks[0].serv_cost = 0;
    inst_tasks[0].demand = 0;
    inst_tasks[0].inverse = 0;
    inst_arcs[0].head_node = DEPOT;
    inst_arcs[0].tail_node = DEPOT;
    inst_arcs[0].trav_cost = 0;
    inst_arcs[0].base_cost = 0;

    for (int i=1; i<=total_arc_num; i++)
    {
        cost_backup[inst_arcs[i].head_node][inst_arcs[i].tail_node] = inst_arcs[i].trav_cost;
    }

    memset(edge_index, 0, sizeof(edge_index));
    edge_index[0] = req_edge_num;
    edge_index[1] = nonreq_edge_num;
}

void read_dynamic_map(Task *inst_tasks, Arc *inst_arcs, const char *map1, CARPInd *sol_seq)
{
    FILE *fp;

    char dummy[101];

    char path[50];
    memset(path, 0, sizeof(path));
    sprintf(path, "degl/d-%s.dat", map1);

    fp = fopen(path, "r");
    if (fp == NULL)
    {
        printf("The file <%s> can't be open\n", path);
        exit(0);
    }
    while (fscanf(fp, "%s", dummy) != EOF)
    {

        if (strcmp(dummy, "VERTICES")==0)
        {
            fscanf(fp, "%s", dummy);
            fscanf(fp, "%d", &vertex_num);
        }
        else if (strcmp(dummy, "ARISTAS_REQ") == 0)
        {
            fscanf(fp, "%s", dummy);
            fscanf(fp, "%d", &req_edge_num);
        }
        else if (strcmp(dummy, "ARISTAS_NOREQ")==0)
        {
            fscanf(fp, "%s", dummy);
            fscanf(fp, "%d", &nonreq_edge_num);
        }
        else if (strcmp(dummy, "VEHICULOS")==0)
        {
            fscanf(fp, "%s", dummy);
            fscanf(fp, "%d", &vehicle_num);
        }
        else if (strcmp(dummy, "CAPACIDAD")==0)
        {
            fscanf(fp, "%s", dummy);
            fscanf(fp, "%d", &capacity);
        }
        else if (strcmp(dummy, "LISTA_ARISTAS_REQ")==0) {

            fscanf(fp, "%s", dummy);
            task_num = 2 * req_edge_num + req_arc_num;
            total_arc_num = task_num + 2 * nonreq_edge_num + nonreq_arc_num;
            for (int i = 1; i <= req_edge_num; i++) {
                fscanf(fp, "%s", dummy);
                fscanf(fp, "%d,", &inst_tasks[i].head_node);
                fscanf(fp, "%d)", &inst_tasks[i].tail_node);
                fscanf(fp, "%s", dummy);
                fscanf(fp, "%d", &inst_tasks[i].serv_cost);
                fscanf(fp, "%s", dummy);
                fscanf(fp, "%d", &inst_tasks[i].demand);

                inst_tasks[i].dead_cost = inst_tasks[i].serv_cost;
                inst_tasks[i].inverse = i + req_edge_num;
                inst_tasks[i].vt = 0;
                inst_tasks[i].head_node_r = 2*i-1;
                inst_tasks[i].tail_node_r = 2*i;

                inst_tasks[i + req_edge_num].head_node = inst_tasks[i].tail_node;
                inst_tasks[i + req_edge_num].tail_node = inst_tasks[i].head_node;
                inst_tasks[i + req_edge_num].dead_cost = inst_tasks[i].dead_cost;
                inst_tasks[i + req_edge_num].serv_cost = inst_tasks[i].serv_cost;
                inst_tasks[i + req_edge_num].demand = inst_tasks[i].demand;
                inst_tasks[i + req_edge_num].inverse = i;
                inst_tasks[i + req_edge_num].vt = 0;
                inst_tasks[i + req_edge_num].head_node_r = inst_tasks[i].tail_node_r;
                inst_tasks[i + req_edge_num].tail_node_r = inst_tasks[i].head_node_r;


                inst_arcs[i].head_node = inst_tasks[i].head_node;
                inst_arcs[i].tail_node = inst_tasks[i].tail_node;
                inst_arcs[i].trav_cost = inst_tasks[i].dead_cost;
                inst_arcs[i].base_cost = inst_arcs[i].trav_cost;
                inst_arcs[i].addt = 0;

                inst_arcs[i + req_edge_num].head_node = inst_arcs[i].tail_node;
                inst_arcs[i + req_edge_num].tail_node = inst_arcs[i].head_node;
                inst_arcs[i + req_edge_num].trav_cost = inst_arcs[i].trav_cost;
                inst_arcs[i + req_edge_num].base_cost = inst_arcs[i].base_cost;
                inst_arcs[i + req_edge_num].addt = 0;

                if (costlb > inst_tasks[i].dead_cost)
                    costlb = inst_tasks[i].dead_cost;

                if (costub < inst_tasks[i].dead_cost)
                    costub = inst_tasks[i].dead_cost;

                if (demandlb > inst_tasks[i].demand)
                    demandlb = inst_tasks[i].demand;

                if (demandub < inst_tasks[i].demand)
                    demandub = inst_tasks[i].demand;

            }
        }
        else if (strcmp(dummy, "LISTA_ARISTAS_NOREQ")==0)
        {
            fscanf(fp, "%s", dummy);
            for (int i=task_num+1; i<=task_num+nonreq_edge_num;i++)
            {
                fscanf(fp, "%s", dummy);
                fscanf(fp, "%d,", &inst_arcs[i].head_node);
                fscanf(fp, "%d)", &inst_arcs[i].tail_node);
                fscanf(fp, "%s", dummy);
                fscanf(fp, "%d", &inst_arcs[i].trav_cost);
                fscanf(fp, "%s", dummy);
                fscanf(fp, "%d", &inst_arcs[i].addt);
                inst_arcs[i].base_cost = inst_arcs[i].trav_cost;

                inst_arcs[i + nonreq_edge_num].head_node = inst_arcs[i].tail_node;
                inst_arcs[i + nonreq_edge_num].tail_node = inst_arcs[i].head_node;
                inst_arcs[i + nonreq_edge_num].trav_cost = inst_arcs[i].trav_cost;
                inst_arcs[i + nonreq_edge_num].base_cost = inst_arcs[i].base_cost;
                inst_arcs[i + nonreq_edge_num].addt = inst_arcs[i].addt;

                if (costlb > inst_arcs[i].trav_cost)
                    costlb = inst_arcs[i].trav_cost;

                if (costub < inst_arcs[i].trav_cost)
                    costub = inst_arcs[i].trav_cost;
            }
        }
        else if (strcmp(dummy, "DEPOSITO")==0)
        {
            fscanf(fp, "%s", dummy);
            fscanf(fp, "%d", &DEPOT);
        }
        else if(strcmp(dummy, "MAENSBEST")==0)
        {
            fscanf(fp, "%s", dummy);
            fscanf(fp, "%d", &sol_seq->TotalCost);
        }
        else if(strcmp(dummy, "MAENSSOL")==0)
        {
            fscanf(fp, "%s", dummy);
            fscanf(fp, "%d", &sol_seq->Sequence[0]);
            for (int i=1; i<=sol_seq->Sequence[0];i++)
            {
                fscanf(fp, "%d", &sol_seq->Sequence[i]);
            }
            int load = 0;
            for(int i=2; i<=sol_seq->Sequence[0]; i++)
            {
                if (sol_seq->Sequence[i] == 0)
                {
                    sol_seq->Loads[0]++;
                    sol_seq->Loads[sol_seq->Loads[0]] = load;
                    load = 0;
                    continue;
                }
                load += inst_tasks[sol_seq->Sequence[i]].demand;
            }
        }
    }

    fclose(fp);

    inst_tasks[0].head_node = DEPOT;
    inst_tasks[0].tail_node = DEPOT;
    inst_tasks[0].head_node_r = 0;
    inst_tasks[0].tail_node_r = 0;
    inst_tasks[0].dead_cost = 0;
    inst_tasks[0].serv_cost = 0;
    inst_tasks[0].demand = 0;
    inst_tasks[0].inverse = 0;
    inst_arcs[0].head_node = DEPOT;
    inst_arcs[0].tail_node = DEPOT;
    inst_arcs[0].trav_cost = 0;
    inst_arcs[0].base_cost = 0;
    inst_arcs[0].addt = 0;

    for (int i=1; i<=total_arc_num; i++)
    {
        cost_backup[inst_arcs[i].head_node][inst_arcs[i].tail_node] = inst_arcs[i].trav_cost;
    }

    memset(edge_index, 0, sizeof(edge_index));
    edge_index[0] = req_edge_num;
    edge_index[1] = nonreq_edge_num;
}

void read_instance_from_xml(Task *inst_tasks, Arc *inst_arcs, Vehicles *state)
{
    char path[100];
    memset(path, 0, sizeof(path));
    sprintf(path, "degl/%s/P%d-s%d-i%d.xml", map, file_name_sep, scn_num, dym_t);
//    sprintf(path, "degl/%s/%d.xml", map, dym_t);
//    sprintf(path, "cost-change-data/k=%d/%s/instance/instance%d.xml", (int)cost_change_ratio, map, instance_id);
//    std::cout << path << std::endl;

    XMLDocument doc;
    doc.LoadFile(path);
    XMLElement* root = doc.FirstChildElement( "Instance" );

    XMLElement* vehicles;
    vehicles = root->FirstChildElement("State");

    char rem_seq_str[2000];

    state->stop[0] = 0;
    state->remain_capacity[0] = 0;
    state->remain_seqs[0] = 1;
    state->remain_seqs[1] = 0;
    state->not_served_task_seq[0] = 1;
    state->not_served_task_seq[1] = 0;

    XMLElement* curr_veh;
    curr_veh = vehicles->FirstChildElement("vehicle");
    while (true)
    {
        if (curr_veh == 0) break;

        int stop_point = atoi(curr_veh->Attribute("stop"));

        if (stop_point == 1)
        {
            strcpy(rem_seq_str, curr_veh->Attribute("seq"));
            char *token = strtok(rem_seq_str, " ");
            while (token != NULL) {
                state->not_served_task_seq[0]++;
                state->not_served_task_seq[state->not_served_task_seq[0]] = atoi(token);
                token = strtok(NULL, " ");
            }
            state->not_served_task_seq[0]++;
            state->not_served_task_seq[state->not_served_task_seq[0]] = 0;
        } else{
            state->stop[0] ++;
            state->stop[state->stop[0]] = stop_point;

            state->remain_capacity[0] ++;
            state->remain_capacity[state->remain_capacity[0]] = atoi(curr_veh->Attribute("rcapacity"));

            strcpy(rem_seq_str, curr_veh->Attribute("seq"));
            char *token = strtok(rem_seq_str, " ");
            while (token != NULL) {
                state->remain_seqs[0]++;
                state->remain_seqs[state->remain_seqs[0]] = atoi(token);
                token = strtok(NULL, " ");
            }
            state->remain_seqs[0]++;
            state->remain_seqs[state->remain_seqs[0]] = 0;
        }
        
        XMLElement* next_veh = curr_veh->NextSiblingElement();
        curr_veh = next_veh;
    }

    XMLElement* tasks;
    tasks = root->FirstChildElement("Tasks");

    task_num = atoi(tasks->Attribute("num"));

    XMLElement* curr_task;
    curr_task = tasks->FirstChildElement("task");
    int k = -1;
    while(true)
    {
        if (curr_task == 0) break;
        k ++;
        inst_tasks[k].head_node = atoi(curr_task->Attribute("head_node"));
        inst_tasks[k].tail_node = atoi(curr_task->Attribute("tail_node"));
        inst_tasks[k].demand = atoi(curr_task->Attribute("demand"));
        inst_tasks[k].dead_cost = atoi(curr_task->Attribute("dead_cost"));
        inst_tasks[k].serv_cost = atoi(curr_task->Attribute("serv_cost"));
        inst_tasks[k].inverse = atoi(curr_task->Attribute("inverse"));
        // inst_tasks[k].head_node_r = 2*k-1;
        // inst_tasks[k].tail_node_r = 2*k;

        // inst_tasks[k].vt = atoi(curr_task->Attribute("vt"));

        XMLElement* next_task = curr_task->NextSiblingElement();
        curr_task = next_task;
    }

    XMLElement* arcs;
    arcs = root->FirstChildElement("Arcs");
    XMLElement* curr_arc;
    curr_arc = arcs->FirstChildElement("arc");
    k = -1;
    while(true)
    {
        if (curr_arc == 0) break;
        k ++;
        inst_arcs[k].head_node = atoi(curr_arc->Attribute("head_node"));
        inst_arcs[k].tail_node = atoi(curr_arc->Attribute("tail_node"));
        inst_arcs[k].trav_cost = atoi(curr_arc->Attribute("trav_cost"));
        // inst_arcs[k].change = atoi(curr_arc->Attribute("change"));
        // inst_arcs[k].link = atoi(curr_arc->Attribute("link"));

        XMLElement* next_arc = curr_arc->NextSiblingElement();
        curr_arc = next_arc;
    }

    XMLElement* info;
    info = root->FirstChildElement("Info");
    XMLElement* tmp;
    tmp = info->FirstChildElement("req_edge");
    req_edge_num = atoi(tmp->Attribute("num"));
    // std::cout << req_edge_num << std::endl;

    tmp = info->FirstChildElement("non_req_edge");
    nonreq_edge_num = atoi(tmp->Attribute("num"));

    tmp = info->FirstChildElement("vertex");
    vertex_num = atoi(tmp->Attribute("num"));

    tmp = info->FirstChildElement("total_arc");
    total_arc_num = atoi(tmp->Attribute("num"));

    tmp = info->FirstChildElement("capacity");
    capacity = atoi(tmp->Attribute("num"));

    tmp = info->FirstChildElement("vehicle_num");
    vehicle_num = atoi(tmp->Attribute("num"));

//    edge_index[0] = req_edge_num;
//    edge_index[1] = nonreq_edge_num;

    DEPOT = 1;


    XMLElement* dynamic;
    info = root->FirstChildElement("Dynamic");
    XMLElement* cost_change;
    cost_change = info->FirstChildElement("cost");

    XMLElement* curr_change_edge;
    curr_change_edge = cost_change->FirstChildElement("change");

    k = 0;
    std::string arc;
    while (true)
    {
        if (curr_change_edge == 0) break;
        int head_node = atoi(curr_change_edge->Attribute("head_node"));
        int tail_node = atoi(curr_change_edge->Attribute("tail_node"));
        k ++;
        arc = std::to_string(head_node) + "," + std::to_string(tail_node);
        cost_change_edges.emplace(arc);
        arc = std::to_string(tail_node) + "," + std::to_string(head_node);
        cost_change_edges.emplace(arc);
        XMLElement* next_change_edge = curr_change_edge->NextSiblingElement();
        curr_change_edge = next_change_edge;
    }

    XMLElement* task_change;
    task_change = info->FirstChildElement("task");
    if (task_change != nullptr) {
        XMLElement *curr_change_task;
        curr_change_task = task_change->FirstChildElement("change");

        while (true) {
            if (curr_change_task == 0) break;
            int head_node = atoi(curr_change_task->Attribute("head_node"));
            int tail_node = atoi(curr_change_task->Attribute("tail_node"));

            XMLElement *next_change_task = curr_change_task->NextSiblingElement();
            curr_change_task = next_change_task;

            if (head_node < 0 && tail_node < 0) continue;
            for (int i = 1; i <= task_num; i++) {
                if (inst_tasks[i].head_node == head_node && inst_tasks[i].tail_node == tail_node) {
                    new_tasks_list[0]++;
                    new_tasks_list[new_tasks_list[0]] = i;
                    break;
                }
            }
        }
    }
}

void save_dcarp_instance(const Task *inst_tasks, const Arc *inst_arcs, Vehicles state, int instance_idx)
{
    int i;

    XMLDocument doc;
    XMLElement* root = doc.NewElement("Instance");
    root->SetAttribute("map", map);
    doc.InsertFirstChild(root);
    
    XMLElement* vehicles;
    vehicles = root->InsertNewChildElement("State");

    int k=2;
    char s[1000];
    char tmps[10];
    for (i = 1; i <= state.stop[0]; i++)
    {
        XMLElement* veh = vehicles->InsertNewChildElement("vehicle");
        veh->SetAttribute("stop", state.stop[i]);
        veh->SetAttribute("rcapacity", state.remain_capacity[i]);
        memset(s, 0, sizeof(s));
        while (true)
        {    
            memset(tmps, 0, sizeof(tmps));
            sprintf(tmps, "%d ", state.remain_seqs[k]);
            strcat(s, tmps);
            k++;
            if(state.remain_seqs[k] == 0)
            {
                k++;
                break;
            }
        }
        veh->SetAttribute("seq", s);
    }
    k=2;
    while (true)
    {
        if (k >= state.not_served_task_seq[0])
        {
            break;
        }

        XMLElement* veh = vehicles->InsertNewChildElement("vehicle");
        veh->SetAttribute("stop", 1);
        veh->SetAttribute("rcapacity", capacity);
        memset(s, 0, sizeof(s));
        while (true)
        {    
            memset(tmps, 0, sizeof(tmps));
            sprintf(tmps, "%d ", state.not_served_task_seq[k]);
            strcat(s, tmps);
            k++;
            if(state.not_served_task_seq[k] == 0)
            {
                k++;
                break;
            }
        }
        veh->SetAttribute("seq", s);
        
    }
    

    XMLElement* tasks;
    tasks = root->InsertNewChildElement("Tasks");
    tasks->SetAttribute("num", task_num);
    for (i = 0; i <= task_num; i++)
    {   
        XMLElement* task;
        task = tasks->InsertNewChildElement("task");
        task->SetAttribute("index", i);
        task->SetAttribute("head_node", inst_tasks[i].head_node);
        task->SetAttribute("tail_node", inst_tasks[i].tail_node);
        task->SetAttribute("demand", inst_tasks[i].demand);
        task->SetAttribute("dead_cost", inst_tasks[i].dead_cost);
        task->SetAttribute("serv_cost", inst_tasks[i].serv_cost);
        task->SetAttribute("inverse", inst_tasks[i].inverse);
    }

    XMLElement* arcs;
    arcs = root->InsertNewChildElement("Arcs");
    for (i = 0; i <= total_arc_num; i++)
    {   
        XMLElement* arc;
        arc = arcs->InsertNewChildElement("arc");
        arc->SetAttribute("head_node", inst_arcs[i].head_node);
        arc->SetAttribute("tail_node", inst_arcs[i].tail_node);
        arc->SetAttribute("base_cost", inst_arcs[i].base_cost);
        arc->SetAttribute("trav_cost", inst_arcs[i].trav_cost);
    }

    // *************************
    XMLElement* dynamic;
    dynamic = root->InsertNewChildElement("Dynamic");
    
    XMLElement* cost;
    cost = dynamic->InsertNewChildElement("cost");
    for (i=1; i<= info_cost_change[0][0]; i++)
    {
        XMLElement* increase;
        increase = cost->InsertNewChildElement("change");
        increase->SetAttribute("head_node", info_cost_change[i][0]);
        increase->SetAttribute("tail_node", info_cost_change[i][1]);
        increase->SetAttribute("cost", info_cost_change[i][2]);
    }

    XMLElement* demand;
    demand = dynamic->InsertNewChildElement("demand");
    for (i=1; i<= info_demand_change[0][0]; i++)
    {
        XMLElement* task;
        task = demand->InsertNewChildElement("task");
        task->SetAttribute("head_node", info_demand_change[i][0]);
        task->SetAttribute("tail_node", info_demand_change[i][1]);
        task->SetAttribute("demand", info_demand_change[i][2]);
    }

    XMLElement* newTask;
    newTask = dynamic->InsertNewChildElement("task");
    for (i=1; i<= info_task_change[0][0]; i++)
    {
        XMLElement* task;
        task = newTask->InsertNewChildElement("change");
        task->SetAttribute("head_node", info_task_change[i][0]);
        task->SetAttribute("tail_node", info_task_change[i][1]);
        task->SetAttribute("demand", info_task_change[i][2]);
    }

    // *************************
    XMLElement* info;
    info = root->InsertNewChildElement("Info");

    XMLElement* tmp;

    tmp = info->InsertNewChildElement("capacity");
    tmp->SetAttribute("num", capacity);

    tmp = info->InsertNewChildElement("req_edge");
    tmp->SetAttribute("num", req_edge_num);

    tmp = info->InsertNewChildElement("non_req_edge");
    tmp->SetAttribute("num", nonreq_edge_num);

    tmp = info->InsertNewChildElement("vertex");
    tmp->SetAttribute("num", vertex_num);

    tmp = info->InsertNewChildElement("total_arc");
    tmp->SetAttribute("num", total_arc_num);

    tmp = info->InsertNewChildElement("vehicle_num");
    tmp->SetAttribute("num", vehicle_num);

    char xml_path[50];
    sprintf(xml_path, "degl/%s/P%d-s%d-i%d.xml", map, (int)file_name_sep, scn_num, dym_t+1);
    printf("%s\n", xml_path);
    doc.SaveFile(xml_path);
}

void save_dcarp_best(const CARPInd best_ind, const int add_cost, const int instance_idx)
{
    FILE *fp;
    char path[50];
    sprintf(path, "best/%s/instance%d.txt", map, instance_idx);
    fp = fopen(path, "w");
    if (fp == NULL)
    {
        printf("The file <%s> can't be open\n", path);
        exit(0);
    }

    fprintf(fp, "%d\n", best_ind.TotalCost-add_cost);
    int i;
    for (i=0; i<best_ind.Sequence[0]; i++)
    {
        fprintf(fp, "%d,", best_ind.Sequence[i]);
    }
    fprintf(fp, "%d\nadd:%d\n", best_ind.Sequence[i], add_cost);
    fclose(fp);
}