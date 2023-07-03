
// module for processing virtual task related solution/individual

#include "src.h"

void indi_route_converter(CARPInd *dst, CARPInd *src, const Task *inst_tasks)
{
    int i, load;
    load = 0;
    memset(dst->Sequence, 0, sizeof(int) * MAX_TASK_SEQ_LENGTH);
    memset(dst->Loads, 0, sizeof(int) * 50);
    dst->Sequence[0] = 1;
    dst->Sequence[1] = 0;
    for (i = 2; i <= src->Sequence[0]; i++)
    {
        if (src->Sequence[i] == 0)
        {
            dst->Sequence[0] ++;
            dst->Sequence[dst->Sequence[0]] = 0;
            dst->Loads[0] ++;
            dst->Loads[dst->Loads[0]] = load;
            load = 0;
            continue;
        }
        if (inst_tasks[src->Sequence[i]].vt > 0 && src->Sequence[i-1] != 0)
        {
            dst->Sequence[0] ++;
            dst->Sequence[dst->Sequence[0]] = 0;
            dst->Loads[0] ++;
            dst->Loads[dst->Loads[0]] = load;
            load = 0;
        }

        load += inst_tasks[src->Sequence[i]].demand;
        dst->Sequence[0] ++;
        dst->Sequence[dst->Sequence[0]] = src->Sequence[i];
    }
}

void construct_virtual_task(const Task *inst_tasks, Task *tasks_vt, const int *stop, const int *remain_capacity)
{

    int i;

    actual_req_edge_num = req_edge_num;
    req_edge_num += stop[0];
    task_num = 2 * req_edge_num;

    
    for (i = 1; i <= actual_req_edge_num; i++)
    {
        tasks_vt[i].head_node = inst_tasks[i].head_node;
        tasks_vt[i].tail_node = inst_tasks[i].tail_node;
        tasks_vt[i].dead_cost = inst_tasks[i].dead_cost;
        tasks_vt[i].serv_cost = inst_tasks[i].serv_cost;
        tasks_vt[i].demand = inst_tasks[i].demand;
        tasks_vt[i].inverse = i + req_edge_num;
        tasks_vt[i].vt = 0;
        tasks_vt[i].head_node_r = 2*i-1;
        tasks_vt[i].tail_node_r = 2*i;

        tasks_vt[i + req_edge_num].head_node = tasks_vt[i].tail_node;
        tasks_vt[i + req_edge_num].tail_node = tasks_vt[i].head_node;
        tasks_vt[i + req_edge_num].head_node_r = tasks_vt[i].tail_node_r;
        tasks_vt[i + req_edge_num].tail_node_r = tasks_vt[i].head_node_r;

        tasks_vt[i + req_edge_num].dead_cost = tasks_vt[i].dead_cost;
        tasks_vt[i + req_edge_num].serv_cost = tasks_vt[i].serv_cost;
        tasks_vt[i + req_edge_num].demand = tasks_vt[i].demand;
        tasks_vt[i + req_edge_num].inverse = i;
        tasks_vt[i + req_edge_num].vt = 0;

    }
    if (stop[0] == 0)
    {
        return;
    }
    for (i = actual_req_edge_num+1; i <= req_edge_num; i++)
    {
        tasks_vt[i].head_node = DEPOT;
        tasks_vt[i].tail_node = stop[i-actual_req_edge_num];
        tasks_vt[i].dead_cost = 0;
        tasks_vt[i].serv_cost = min_cost[tasks_vt[i].head_node][tasks_vt[i].tail_node];
        tasks_vt[i].demand = capacity - remain_capacity[i-actual_req_edge_num];
        if (tasks_vt[i].demand == 0)
        {
            tasks_vt[i].demand = 0;
        }
        tasks_vt[i].inverse = i + req_edge_num;
        tasks_vt[i].vt = 1;
        tasks_vt[i].head_node_r = 2*i-1;
        tasks_vt[i].tail_node_r = 2*i;

        tasks_vt[i + req_edge_num].head_node = tasks_vt[i].head_node;
        tasks_vt[i + req_edge_num].tail_node = tasks_vt[i].tail_node;
        tasks_vt[i + req_edge_num].head_node_r = tasks_vt[i].tail_node_r;
        tasks_vt[i + req_edge_num].tail_node_r = tasks_vt[i].head_node_r;
        tasks_vt[i + req_edge_num].dead_cost = tasks_vt[i].dead_cost;
        tasks_vt[i + req_edge_num].serv_cost = tasks_vt[i].serv_cost;
        tasks_vt[i + req_edge_num].demand = tasks_vt[i].demand;
        tasks_vt[i + req_edge_num].inverse = i;
        tasks_vt[i + req_edge_num].vt = 1;
    }
    tasks_vt[0].head_node = DEPOT;
    tasks_vt[0].tail_node = DEPOT;
    tasks_vt[0].dead_cost = 0;
    tasks_vt[0].serv_cost = 0;
    tasks_vt[0].demand = 0;
    tasks_vt[0].inverse = 0;
    tasks_vt[0].head_node_r = 0;
    tasks_vt[0].tail_node_r = 0;
}

void vt_demand_change(Task *inst_tasks, int flag)
{
    int i, j;
    i = inst_tasks[1].inverse-1;
    while (true)
    {
        if (inst_tasks[i].vt > 0 && inst_tasks[i].demand == (1-flag))
        {   
            inst_tasks[i].demand = flag;
            inst_tasks[inst_tasks[i].inverse].demand = flag;
        } 
        
        if(inst_tasks[i].vt == 0){
            break;
        }
        i--;
    }
}