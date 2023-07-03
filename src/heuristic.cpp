#include "src.h"


void route_process(int *split_task_seq, const int *process_split, const Task *inst_tasks);
int split(int *split_task_seq, int *one_task_seq, int *split_route_loads, const Task *inst_tasks)
{
    int V[MAX_TASK_TAG_LENGTH], P[MAX_TASK_TAG_LENGTH];
    V[0] = 0;
    P[0] = 0;
    int i, j;

    for(i = 1; i <= one_task_seq[0]; i++)
    {
        V[i] = INF;
    }

    for(i = 1; i <= one_task_seq[0]; i++)
    {
        int load = 0, cost = 0;
        j = i;
        while (j <= one_task_seq[0] && load <= capacity)
        {
            // for dynamic CARP, once a virtual task being added, there is a new route.
            // the load reset to 0
            if (inst_tasks[one_task_seq[j]].vt > 0)
            {
                load = 0;
            }
            // for static CARP, each split represented one route
            load += inst_tasks[one_task_seq[j]].demand;
            if (j == i)
            {
                cost = min_cost[DEPOT][inst_tasks[one_task_seq[j]].head_node] \
                        + inst_tasks[one_task_seq[j]].serv_cost \
                        + min_cost[inst_tasks[one_task_seq[j]].tail_node][DEPOT];
            } else {
                cost += min_cost[inst_tasks[one_task_seq[j-1]].tail_node][inst_tasks[one_task_seq[j]].head_node] \
                        + inst_tasks[one_task_seq[j]].serv_cost \
                        + min_cost[inst_tasks[one_task_seq[j]].tail_node][DEPOT] \
                        - min_cost[inst_tasks[one_task_seq[j-1]].tail_node][DEPOT];
            }
            if (load <= capacity)
            {
                int V_new = V[i-1] + cost;
                if (V_new < V[j])
                {
                    V[j] = V_new;
                    P[j] = i-1;
                }
                j ++;
            }
        }
    }

    int process_split[MAX_TASK_SEQ_LENGTH];
    memset(process_split, 0, sizeof(process_split));

    process_split[0] = 1;
    process_split[1] = 0;

    j = one_task_seq[0];
    int ptr = P[j];
    while (ptr > 0)
    {
        for (int k=ptr+1; k<=j; k++)
        {
            process_split[0] ++;
            process_split[process_split[0]] = one_task_seq[k];
        }

        process_split[0] ++;
        process_split[process_split[0]] = 0;

        j = ptr;
        ptr = P[j];
    }

    for (int k=1; k <= j; k++)
    {
        process_split[0] ++;
        process_split[process_split[0]] = one_task_seq[k];
    }
    process_split[0] ++;
    process_split[process_split[0]] = 0;

    route_process(split_task_seq, process_split, inst_tasks);

    int load = 0;
    for (i=2; i <= split_task_seq[0]; i++)
    {
        load += inst_tasks[split_task_seq[i]].demand;
        if (split_task_seq[i] == 0)
        {
            split_route_loads[0] ++;
            split_route_loads[split_route_loads[0]] = load;
            load = 0;
        }
    }

    int opt_cost = V[one_task_seq[0]];
    return opt_cost;

}

void route_process(int *split_task_seq, const int *process_split, const Task *inst_tasks)
{
    int i;
    split_task_seq[0] = 0;
     
    for (i = 1; i <= process_split[0]; i++)
    {
        if (inst_tasks[process_split[i]].vt > 0 && process_split[i-1] != 0)
        {
            split_task_seq[0] ++;
            split_task_seq[split_task_seq[0]] = 0;
        }

        split_task_seq[0] ++;
        split_task_seq[split_task_seq[0]] = process_split[i];        
    }
}

void path_scanning(CARPInd *ps_indi, const Task *inst_tasks, const int *serve_mark)
{
    // min_cost, NRE, NRA, NVeh, capacity, is the extern variables.
    int i, j, k;
    int serve_task_num=0;
    for (i=req_edge_num+1; i<=task_num; i++)
    {
        if (serve_mark[i])
        {
            serve_task_num ++;
        }
    }
    int load, trial, mindist;
    int unserved_task[MAX_TASK_TAG_LENGTH], candi_task[MAX_TASK_TAG_LENGTH], nearest_task[MAX_TASK_TAG_LENGTH];
    int nearest_isol_task[MAX_TASK_TAG_LENGTH], nearest_inci_task[MAX_TASK_TAG_LENGTH], sel_task[MAX_TASK_TAG_LENGTH];
    int current_task, next_task;

    int positions[MAX_TASK_SEQ_LENGTH];

    ps_indi->TotalCost = INF;
    CARPInd tmp_indi1, tmp_indi2, tmp_indi3, tmp_indi4, tmp_indi5;

    int dep_dist[MAX_TASK_TAG_LENGTH], max_dep_dist, min_dep_dist;
    double yield[MAX_TASK_TAG_LENGTH], max_yield, min_yield;

    for (i=1; i<=task_num; i++)
    {
        if (!serve_mark[i])
            continue;
        if(inst_tasks[i].demand > capacity)
        {
            printf("error. \n");
            // longjmp(buf, 2);
            exit(0);
        }

        dep_dist[i] = min_cost[inst_tasks[i].tail_node][DEPOT];
        yield[i] = 1.0*inst_tasks[i].demand/inst_tasks[i].serv_cost;
    }

    /*
     * / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
     * Use Rule 1 to obtain a solution
     * / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
     * */
    tmp_indi1.Sequence[0] = 1;
    tmp_indi1.Sequence[1] = 0;
    tmp_indi1.Loads[0] = 0;

    unserved_task[0] = 0;
    for (i=1; i<=task_num; i++)
    {
        if (!serve_mark[i])
            continue;
        unserved_task[0] ++;
        unserved_task[unserved_task[0]] = i;
    }

    load = 0;
    trial = 0;
    while(trial < serve_task_num)
    {
        current_task = tmp_indi1.Sequence[tmp_indi1.Sequence[0]]; // get the current task's id
        candi_task[0] = 0;

        for (i = 1; i <= unserved_task[0]; i++)
        {
            if (inst_tasks[unserved_task[i]].demand <= capacity-load)
            {
                candi_task[0] ++;
                candi_task[candi_task[0]] = unserved_task[i];
            }
        }

        if (candi_task[0] == 0)
        {
            tmp_indi1.Sequence[0] ++;
            tmp_indi1.Sequence[tmp_indi1.Sequence[0]] = 0;
            tmp_indi1.Loads[0] ++;
            tmp_indi1.Loads[tmp_indi1.Loads[0]] = load;
            load = 0;
            continue;
        }

        mindist = INF;
        nearest_task[0] = 0;
        for (i = 1; i<=candi_task[0]; i++)
        {
            if (min_cost[inst_tasks[current_task].tail_node][inst_tasks[candi_task[i]].head_node] < mindist)
            {
                mindist = min_cost[inst_tasks[current_task].tail_node][inst_tasks[candi_task[i]].head_node];
                nearest_task[0] = 1;
                nearest_task[nearest_task[0]] = candi_task[i];
            }else if(min_cost[inst_tasks[current_task].tail_node][inst_tasks[candi_task[i]].head_node] == mindist)
            {
                nearest_task[0] ++;
                nearest_task[nearest_task[0]] = candi_task[i];
            }
        }

        nearest_inci_task[0] = 0;
        nearest_isol_task[0] = 0;
        for(i=1; i<=nearest_task[0]; i++)
        {
            if (inst_tasks[nearest_task[i]].tail_node == 1)
            {
                nearest_inci_task[0] ++;
                nearest_inci_task[nearest_inci_task[0]] = nearest_task[i];
            }
            else
            {
                nearest_isol_task[0] ++;
                nearest_isol_task[nearest_isol_task[0]] = nearest_task[i];
            }
        }
        if (nearest_isol_task[0] == 0)
        {
            memcpy(nearest_isol_task, nearest_inci_task, (nearest_inci_task[0]+1)*sizeof(int));
        }

        // for 5 five phase, the above part is the same
        max_dep_dist = -1;
        sel_task[0] = 0;
        for (i = 1; i <= nearest_isol_task[0]; i++)
        {
            if (dep_dist[nearest_isol_task[i]] > max_dep_dist)
            {
                max_dep_dist = dep_dist[nearest_isol_task[i]];
                sel_task[0] = 1;
                sel_task[sel_task[0]] = nearest_isol_task[i];
            }
            else if (dep_dist[nearest_isol_task[i]] == max_dep_dist)
            {
                sel_task[0] ++;
                sel_task[sel_task[0]] = nearest_isol_task[i];
            }
        }
        k = 1;
        next_task = sel_task[k];

        trial ++;
        tmp_indi1.Sequence[0]++;
        tmp_indi1.Sequence[tmp_indi1.Sequence[0]] = next_task;
        if (inst_tasks[next_task].vt > 0)
        {
            load = 0;
        }
        load += inst_tasks[next_task].demand;

        // delete the served task in unserved_task array
        find_ele_positions(positions, unserved_task, next_task);
        delete_element(unserved_task,positions[1]);

        if (inst_tasks[next_task].inverse > 0)
        {
            find_ele_positions(positions, unserved_task, inst_tasks[next_task].inverse);
            delete_element(unserved_task, positions[1]);
        }
//        printf("serve_task_num: %d\n", serve_task_num);
    }

//    printf("serve_task_num: %d\n", serve_task_num);
    tmp_indi1.Sequence[0] ++    ;
    tmp_indi1.Sequence[tmp_indi1.Sequence[0]] = 0;
    tmp_indi1.Loads[0] ++;
    tmp_indi1.Loads[tmp_indi1.Loads[0]] = load;

    tmp_indi1.TotalCost = get_task_seq_total_cost(tmp_indi1.Sequence, inst_tasks);
//    tmp_indi1.TotalVioLoad = get_total_vio_load(tmp_indi1.Loads);

    if (tmp_indi1.TotalCost < ps_indi->TotalCost)
    {
//        memcpy(ps_indi->Sequence, tmp_indi1.Sequence, (tmp_indi1.Sequence[0]+1)*sizeof(int));
//        memcpy(ps_indi->Loads, tmp_indi1.Loads, (tmp_indi1.Loads[0]+1)*sizeof(int));
        indi_route_converter(ps_indi, &tmp_indi1, inst_tasks);
        ps_indi->TotalCost = tmp_indi1.TotalCost;
    }

    /*
     * / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
     * Use Rule 2 to obtain a solution
     * / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
     * */
    tmp_indi2.Sequence[0] = 1;
    tmp_indi2.Sequence[1] = 0;
    tmp_indi2.Loads[0] = 0;

    unserved_task[0] = 0;
    for (i=1; i<=task_num; i++)
    {
        if (!serve_mark[i])
            continue;
        unserved_task[0] ++;
        unserved_task[unserved_task[0]] = i;
    }

    load = 0;
    trial = 0;
    while(trial < serve_task_num)
    {
        current_task = tmp_indi2.Sequence[tmp_indi2.Sequence[0]]; // get the current task's id
        candi_task[0] = 0;

        for (i = 1; i <= unserved_task[0]; i++)
        {
            if (inst_tasks[unserved_task[i]].demand <= capacity-load)
            {
                candi_task[0] ++;
                candi_task[candi_task[0]] = unserved_task[i];
            }
        }

        if (candi_task[0] == 0)
        {
            tmp_indi2.Sequence[0] ++;
            tmp_indi2.Sequence[tmp_indi2.Sequence[0]] = 0;
            tmp_indi2.Loads[0] ++;
            tmp_indi2.Loads[tmp_indi2.Loads[0]] = load;
            load = 0;
            continue;
        }

        mindist = INF;
        nearest_task[0] = 0;
        for (i = 1; i<=candi_task[0]; i++)
        {
            if (min_cost[inst_tasks[current_task].tail_node][inst_tasks[candi_task[i]].head_node] < mindist)
            {
                mindist = min_cost[inst_tasks[current_task].tail_node][inst_tasks[candi_task[i]].head_node];
                nearest_task[0] = 1;
                nearest_task[nearest_task[0]] = candi_task[i];
            }else if(min_cost[inst_tasks[current_task].tail_node][inst_tasks[candi_task[i]].head_node] == mindist)
            {
                nearest_task[0] ++;
                nearest_task[nearest_task[0]] = candi_task[i];
            }
        }

        nearest_inci_task[0] = 0;
        nearest_isol_task[0] = 0;
        for(i=1; i<=nearest_task[0]; i++)
        {
            if (inst_tasks[nearest_task[i]].tail_node == 1)
            {
                nearest_inci_task[0] ++;
                nearest_inci_task[nearest_inci_task[0]] = nearest_task[i];
            }
            else
            {
                nearest_isol_task[0] ++;
                nearest_isol_task[nearest_isol_task[0]] = nearest_task[i];
            }
        }
        if (nearest_isol_task[0] == 0)
        {
            memcpy(nearest_isol_task, nearest_inci_task, (nearest_inci_task[0]+1)*sizeof(int));
        }

        // for 5 five phase, the above part is the same
        min_dep_dist = INF;
        sel_task[0] = 0;
        for (i = 1; i <= nearest_isol_task[0]; i++)
        {
            if (dep_dist[nearest_isol_task[i]] < min_dep_dist)
            {
                min_dep_dist = dep_dist[nearest_isol_task[i]];
                sel_task[0] = 1;
                sel_task[sel_task[0]] = nearest_isol_task[i];
            }
            else if (dep_dist[nearest_isol_task[i]] == min_dep_dist)
            {
                sel_task[0] ++;
                sel_task[sel_task[0]] = nearest_isol_task[i];
            }
        }


        k = 1;
        next_task = sel_task[k];

        trial ++;
        tmp_indi2.Sequence[0]++;
        tmp_indi2.Sequence[tmp_indi2.Sequence[0]] = next_task;
        if (inst_tasks[next_task].vt > 0)
        {
            load = 0;
        }
        load += inst_tasks[next_task].demand;

        // delete the served task in unserved_task array
        find_ele_positions(positions, unserved_task, next_task);
        delete_element(unserved_task,positions[1]);

        if (inst_tasks[next_task].inverse > 0)
        {
            find_ele_positions(positions, unserved_task, inst_tasks[next_task].inverse);
            delete_element(unserved_task, positions[1]);
        }
    }

    tmp_indi2.Sequence[0] ++ ;
    tmp_indi2.Sequence[tmp_indi2.Sequence[0]] = 0;
    tmp_indi2.Loads[0] ++;
    tmp_indi2.Loads[tmp_indi2.Loads[0]] = load;

    tmp_indi2.TotalCost = get_task_seq_total_cost(tmp_indi2.Sequence, inst_tasks);

    if (tmp_indi2.TotalCost < ps_indi->TotalCost)
    {
        indi_route_converter(ps_indi, &tmp_indi2, inst_tasks);
        ps_indi->TotalCost = tmp_indi2.TotalCost;
    }

    /*
     * / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
     * Use Rule 3 to obtain a solution
     * / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
     * */
    tmp_indi3.Sequence[0] = 1;
    tmp_indi3.Sequence[1] = 0;
    tmp_indi3.Loads[0] = 0;

    unserved_task[0] = 0;
    for (i=1; i<=task_num; i++)
    {
        if (!serve_mark[i])
            continue;
        unserved_task[0] ++;
        unserved_task[unserved_task[0]] = i;
    }

    load = 0;
    trial = 0;
    while(trial < serve_task_num)
    {
        current_task = tmp_indi3.Sequence[tmp_indi3.Sequence[0]]; // get the current task's id
        candi_task[0] = 0;

        for (i = 1; i <= unserved_task[0]; i++)
        {
            if (inst_tasks[unserved_task[i]].demand <= capacity-load)
            {
                candi_task[0] ++;
                candi_task[candi_task[0]] = unserved_task[i];
            }
        }

        if (candi_task[0] == 0)
        {
            tmp_indi3.Sequence[0] ++;
            tmp_indi3.Sequence[tmp_indi3.Sequence[0]] = 0;
            tmp_indi3.Loads[0] ++;
            tmp_indi3.Loads[tmp_indi3.Loads[0]] = load;
            load = 0;
            continue;
        }

        mindist = INF;
        nearest_task[0] = 0;
        for (i = 1; i<=candi_task[0]; i++)
        {
            if (min_cost[inst_tasks[current_task].tail_node][inst_tasks[candi_task[i]].head_node] < mindist)
            {
                mindist = min_cost[inst_tasks[current_task].tail_node][inst_tasks[candi_task[i]].head_node];
                nearest_task[0] = 1;
                nearest_task[nearest_task[0]] = candi_task[i];
            }else if(min_cost[inst_tasks[current_task].tail_node][inst_tasks[candi_task[i]].head_node] == mindist)
            {
                nearest_task[0] ++;
                nearest_task[nearest_task[0]] = candi_task[i];
            }
        }

        nearest_inci_task[0] = 0;
        nearest_isol_task[0] = 0;
        for(i=1; i<=nearest_task[0]; i++)
        {
            if (inst_tasks[nearest_task[i]].tail_node == 1)
            {
                nearest_inci_task[0] ++;
                nearest_inci_task[nearest_inci_task[0]] = nearest_task[i];
            }
            else
            {
                nearest_isol_task[0] ++;
                nearest_isol_task[nearest_isol_task[0]] = nearest_task[i];
            }
        }
        if (nearest_isol_task[0] == 0)
        {
            memcpy(nearest_isol_task, nearest_inci_task, (nearest_inci_task[0]+1)*sizeof(int));
        }

        // for 5 five phase, the above part is the same
        max_yield = -1;
        sel_task[0] = 0;
        for (i = 1; i <= nearest_isol_task[0]; i++)
        {
            if (yield[nearest_isol_task[i]] > max_yield)
            {
                max_yield = yield[nearest_isol_task[i]];
                sel_task[0] = 1;
                sel_task[sel_task[0]] = nearest_isol_task[i];
            }
            else if (yield[nearest_isol_task[i]] == max_yield)
            {
                sel_task[0] ++;
                sel_task[sel_task[0]] = nearest_isol_task[i];
            }
        }


        k = 1;
        next_task = sel_task[k];

        trial ++;
        tmp_indi3.Sequence[0]++;
        tmp_indi3.Sequence[tmp_indi3.Sequence[0]] = next_task;
        if (inst_tasks[next_task].vt > 0)
        {
            load = 0;
        }
        load += inst_tasks[next_task].demand;
        // delete the served task in unserved_task array
        find_ele_positions(positions, unserved_task, next_task);
        delete_element(unserved_task,positions[1]);

        if (inst_tasks[next_task].inverse > 0)
        {
            find_ele_positions(positions, unserved_task, inst_tasks[next_task].inverse);
            delete_element(unserved_task, positions[1]);
        }
    }

    tmp_indi3.Sequence[0] ++ ;
    tmp_indi3.Sequence[tmp_indi3.Sequence[0]] = 0;
    tmp_indi3.Loads[0] ++;
    tmp_indi3.Loads[tmp_indi3.Loads[0]] = load;

    tmp_indi3.TotalCost = get_task_seq_total_cost(tmp_indi3.Sequence, inst_tasks);

    if (tmp_indi3.TotalCost < ps_indi->TotalCost)
    {
        indi_route_converter(ps_indi, &tmp_indi3, inst_tasks);
        ps_indi->TotalCost = tmp_indi3.TotalCost;
    }

    /*
     * / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
     * Use Rule 4 to obtain a solution
     * / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
     * */
    tmp_indi4.Sequence[0] = 1;
    tmp_indi4.Sequence[1] = 0;
    tmp_indi4.Loads[0] = 0;

    unserved_task[0] = 0;
    for (i=1; i<=task_num; i++)
    {
        if (!serve_mark[i])
            continue;
        unserved_task[0] ++;
        unserved_task[unserved_task[0]] = i;
    }

    load = 0;
    trial = 0;
    while(trial < serve_task_num)
    {
        current_task = tmp_indi4.Sequence[tmp_indi4.Sequence[0]]; // get the current task's id
        candi_task[0] = 0;

        for (i = 1; i <= unserved_task[0]; i++)
        {
            if (inst_tasks[unserved_task[i]].demand <= capacity-load)
            {
                candi_task[0] ++;
                candi_task[candi_task[0]] = unserved_task[i];
            }
        }

        if (candi_task[0] == 0)
        {
            tmp_indi4.Sequence[0] ++;
            tmp_indi4.Sequence[tmp_indi4.Sequence[0]] = 0;
            tmp_indi4.Loads[0] ++;
            tmp_indi4.Loads[tmp_indi4.Loads[0]] = load;
            load = 0;
            continue;
        }

        mindist = INF;
        nearest_task[0] = 0;
        for (i = 1; i<=candi_task[0]; i++)
        {
            if (min_cost[inst_tasks[current_task].tail_node][inst_tasks[candi_task[i]].head_node] < mindist)
            {
                mindist = min_cost[inst_tasks[current_task].tail_node][inst_tasks[candi_task[i]].head_node];
                nearest_task[0] = 1;
                nearest_task[nearest_task[0]] = candi_task[i];
            }else if(min_cost[inst_tasks[current_task].tail_node][inst_tasks[candi_task[i]].head_node] == mindist)
            {
                nearest_task[0] ++;
                nearest_task[nearest_task[0]] = candi_task[i];
            }
        }

        nearest_inci_task[0] = 0;
        nearest_isol_task[0] = 0;
        for(i=1; i<=nearest_task[0]; i++)
        {
            if (inst_tasks[nearest_task[i]].tail_node == 1)
            {
                nearest_inci_task[0] ++;
                nearest_inci_task[nearest_inci_task[0]] = nearest_task[i];
            }
            else
            {
                nearest_isol_task[0] ++;
                nearest_isol_task[nearest_isol_task[0]] = nearest_task[i];
            }
        }
        if (nearest_isol_task[0] == 0)
        {
            memcpy(nearest_isol_task, nearest_inci_task, (nearest_inci_task[0]+1)*sizeof(int));
        }

        // for 5 five phase, the above part is the same
        min_yield = INF;
        sel_task[0] = 0;
        for (i = 1; i <= nearest_isol_task[0]; i++)
        {
            if (yield[nearest_isol_task[i]] < min_yield)
            {
                min_yield = yield[nearest_isol_task[i]];
                sel_task[0] = 1;
                sel_task[sel_task[0]] = nearest_isol_task[i];
            }
            else if (yield[nearest_isol_task[i]] == min_yield)
            {
                sel_task[0] ++;
                sel_task[sel_task[0]] = nearest_isol_task[i];
            }
        }


        k = 1;
        next_task = sel_task[k];

        trial ++;
        tmp_indi4.Sequence[0]++;
        tmp_indi4.Sequence[tmp_indi4.Sequence[0]] = next_task;
        if (inst_tasks[next_task].vt > 0)
        {
            load = 0;
        }
        load += inst_tasks[next_task].demand;
        // delete the served task in unserved_task array
        find_ele_positions(positions, unserved_task, next_task);
        delete_element(unserved_task,positions[1]);

        if (inst_tasks[next_task].inverse > 0)
        {
            find_ele_positions(positions, unserved_task, inst_tasks[next_task].inverse);
            delete_element(unserved_task, positions[1]);
        }
    }

    tmp_indi4.Sequence[0] ++ ;
    tmp_indi4.Sequence[tmp_indi4.Sequence[0]] = 0;
    tmp_indi4.Loads[0] ++;
    tmp_indi4.Loads[tmp_indi4.Loads[0]] = load;

    tmp_indi4.TotalCost = get_task_seq_total_cost(tmp_indi4.Sequence, inst_tasks);

    if (tmp_indi4.TotalCost < ps_indi->TotalCost)
    {
        indi_route_converter(ps_indi, &tmp_indi4, inst_tasks);
        ps_indi->TotalCost = tmp_indi4.TotalCost;
    }

    /*
     * / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
     * Use Rule 5 to obtain a solution
     * / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
     * */
    tmp_indi5.Sequence[0] = 1;
    tmp_indi5.Sequence[1] = 0;
    tmp_indi5.Loads[0] = 0;

    unserved_task[0] = 0;
    for (i=1; i<=task_num; i++)
    {
        if (!serve_mark[i])
            continue;
        unserved_task[0] ++;
        unserved_task[unserved_task[0]] = i;
    }

    load = 0;
    trial = 0;
    while(trial < serve_task_num)
    {
        current_task = tmp_indi5.Sequence[tmp_indi5.Sequence[0]]; // get the current task's id
        candi_task[0] = 0;

        for (i = 1; i <= unserved_task[0]; i++)
        {
            if (inst_tasks[unserved_task[i]].demand <= capacity-load)
            {
                candi_task[0] ++;
                candi_task[candi_task[0]] = unserved_task[i];
            }
        }

        if (candi_task[0] == 0)
        {
            tmp_indi5.Sequence[0] ++;
            tmp_indi5.Sequence[tmp_indi5.Sequence[0]] = 0;
            tmp_indi5.Loads[0] ++;
            tmp_indi5.Loads[tmp_indi5.Loads[0]] = load;
            load = 0;
            continue;
        }

        mindist = INF;
        nearest_task[0] = 0;
        for (i = 1; i<=candi_task[0]; i++)
        {
            if (min_cost[inst_tasks[current_task].tail_node][inst_tasks[candi_task[i]].head_node] < mindist)
            {
                mindist = min_cost[inst_tasks[current_task].tail_node][inst_tasks[candi_task[i]].head_node];
                nearest_task[0] = 1;
                nearest_task[nearest_task[0]] = candi_task[i];
            }else if(min_cost[inst_tasks[current_task].tail_node][inst_tasks[candi_task[i]].head_node] == mindist)
            {
                nearest_task[0] ++;
                nearest_task[nearest_task[0]] = candi_task[i];
            }
        }

        nearest_inci_task[0] = 0;
        nearest_isol_task[0] = 0;
        for(i=1; i<=nearest_task[0]; i++)
        {
            if (inst_tasks[nearest_task[i]].tail_node == 1)
            {
                nearest_inci_task[0] ++;
                nearest_inci_task[nearest_inci_task[0]] = nearest_task[i];
            }
            else
            {
                nearest_isol_task[0] ++;
                nearest_isol_task[nearest_isol_task[0]] = nearest_task[i];
            }
        }
        if (nearest_isol_task[0] == 0)
        {
            memcpy(nearest_isol_task, nearest_inci_task, (nearest_inci_task[0]+1)*sizeof(int));
        }

        // for 5 five phase, the above part is the same
        if (load < capacity/2)
        {
            max_dep_dist = -1;
            sel_task[0] = 0;
            for (i = 1; i <= nearest_isol_task[0]; i++)
            {
                if (dep_dist[nearest_isol_task[i]] > max_dep_dist)
                {
                    max_dep_dist = dep_dist[nearest_isol_task[i]];
                    sel_task[0] = 1;
                    sel_task[sel_task[0]] = nearest_isol_task[i];
                }
                else if (dep_dist[nearest_isol_task[i]] == max_dep_dist)
                {
                    sel_task[0] ++;
                    sel_task[sel_task[0]] = nearest_isol_task[i];
                }
            }
        }
        else
        {
            min_dep_dist = INF;
            sel_task[0] = 0;
            for (i = 1; i <= nearest_isol_task[0]; i++)
            {
                if (dep_dist[nearest_isol_task[i]] < min_dep_dist)
                {
                    min_dep_dist = dep_dist[nearest_isol_task[i]];
                    sel_task[0] = 1;
                    sel_task[sel_task[0]] = nearest_isol_task[i];
                }
                else if (dep_dist[nearest_isol_task[i]] == min_dep_dist)
                {
                    sel_task[0] ++;
                    sel_task[sel_task[0]] = nearest_isol_task[i];
                }
            }
        }


        k = 1;
        next_task = sel_task[k];

        trial ++;
        tmp_indi5.Sequence[0]++;
        tmp_indi5.Sequence[tmp_indi5.Sequence[0]] = next_task;
        if (inst_tasks[next_task].vt > 0)
        {
            load = 0;
        }
        load += inst_tasks[next_task].demand;

        // delete the served task in unserved_task array
        find_ele_positions(positions, unserved_task, next_task);
        delete_element(unserved_task,positions[1]);

        if (inst_tasks[next_task].inverse > 0)
        {
            find_ele_positions(positions, unserved_task, inst_tasks[next_task].inverse);
            delete_element(unserved_task, positions[1]);
        }
    }

    tmp_indi5.Sequence[0] ++ ;
    tmp_indi5.Sequence[tmp_indi5.Sequence[0]] = 0;
    tmp_indi5.Loads[0] ++;
    tmp_indi5.Loads[tmp_indi5.Loads[0]] = load;

    tmp_indi5.TotalCost = get_task_seq_total_cost(tmp_indi5.Sequence, inst_tasks);

    if (tmp_indi5.TotalCost < ps_indi->TotalCost)
    {
        indi_route_converter(ps_indi, &tmp_indi5, inst_tasks);
        ps_indi->TotalCost = tmp_indi5.TotalCost;
    }

    ps_indi->TotalVioLoad = 0;

}


void rand_scanning(CARPInd *rs_indi, const Task *inst_tasks)
{

    CARPInd indi;

    int i, k;

    int serve_task_num =  req_edge_num;
    

    int load, trial, mindist;

    int unserved_task[MAX_TASK_TAG_LENGTH], candi_task[MAX_TASK_TAG_LENGTH], nearest_task[MAX_TASK_TAG_LENGTH];
    int current_task, next_task;

    int positions[MAX_TASK_SEG_LENGTH];

    indi.Sequence[0] = 1;
    indi.Sequence[1] = 0;
    indi.Loads[0] = 0;

    unserved_task[0] = 0;

    for (i = 1; i <= task_num; i++)
    {
        unserved_task[0] ++;
        unserved_task[unserved_task[0]] = i;
    }

    load = 0;
    trial = 0;
    while (trial < serve_task_num)
    {
        current_task = indi.Sequence[indi.Sequence[0]];

        candi_task[0] = 0;
        for( i = 1; i <= unserved_task[0]; i++)
        {
            if( inst_tasks[unserved_task[i]].demand <= capacity - load )
            {
                candi_task[0] ++;
                candi_task[candi_task[0]] = unserved_task[i];
            }
        }

        if (candi_task[0] == 0)
        {
            indi.Sequence[0] ++;
            indi.Sequence[indi.Sequence[0]] = 0;
            indi.Loads[0] ++;
            indi.Loads[indi.Loads[0]] = load;
            load = 0;
            continue;
        }

        mindist = INF;
        nearest_task[0] = 0;

        for (i = 1; i <= candi_task[0]; i++)
        {
            if (min_cost[inst_tasks[current_task].tail_node][inst_tasks[candi_task[i]].head_node] < mindist)
            {
                mindist = min_cost[inst_tasks[current_task].tail_node][inst_tasks[candi_task[i]].head_node];
                nearest_task[0] = 1;
                nearest_task[nearest_task[0]] = candi_task[i];
            }else if(min_cost[inst_tasks[current_task].tail_node][inst_tasks[candi_task[i]].head_node] == mindist)
            {
                nearest_task[0] ++;
                nearest_task[nearest_task[0]] = candi_task[i];
            }
        }

        k = rand_choose(nearest_task[0]);
        next_task = nearest_task[k];
        // printf("next task: %d\n", next_task);

        trial ++;
        indi.Sequence[0] ++;
        indi.Sequence[indi.Sequence[0]] = next_task;
        if (inst_tasks[next_task].vt > 0)
        {
            load = 0;
        }
        load += inst_tasks[next_task].demand;
        find_ele_positions(positions, unserved_task, next_task);
        delete_element(unserved_task, positions[1]);

        if (inst_tasks[next_task].inverse > 0)
        {
            find_ele_positions(positions, unserved_task, inst_tasks[next_task].inverse);
            delete_element(unserved_task, positions[1]);
        }
    }

    indi.Sequence[0] ++;
    indi.Sequence[indi.Sequence[0]] = 0;
    indi.Loads[0] ++;
    indi.Loads[indi.Loads[0]] = load;

    indi.TotalCost = get_task_seq_total_cost(indi.Sequence, inst_tasks);
    indi.TotalVioLoad = get_total_vio_load(indi.Loads);

    indi_route_converter(rs_indi, &indi, inst_tasks);
    rs_indi->TotalCost = get_task_seq_total_cost(indi.Sequence, inst_tasks);
    rs_indi->TotalVioLoad = get_total_vio_load(indi.Loads);
}


// void RepairInfeasibility (CARPInd *Indi, const Task *inst_tasks)
// {
//     int NRE = req_edge_num, NRA = req_arc_num;
//     int i, j, k, RoutesNum;
//     int NodeRoutes[101][MAX_TASK_SEQ_LENGTH], TaskRoutes[101][MAX_TASK_SEQ_LENGTH];
//     int InRoutes[NRE+NRA + 1][101], CheckMark[NRE+NRA+1];
//     int NO_Task = 2*NRE + NRA;

//     // TaskRoutes: each row is the task sequence of each sub-route
//     // RoutesNum: the number of routes
//     RoutesNum = 0;
//     for (i = 1; i < Indi->Sequence[0]; i++)
//     {
//         if (Indi->Sequence[i] == 0)
//         {
//             TaskRoutes[RoutesNum][0]++;
//             TaskRoutes[RoutesNum][TaskRoutes[RoutesNum][0]] = 0;
//             RoutesNum ++;
//             TaskRoutes[RoutesNum][0] = 1;
//             TaskRoutes[RoutesNum][1] = 0;
//         } else{
//             TaskRoutes[RoutesNum][0] ++;
//             TaskRoutes[RoutesNum][TaskRoutes[RoutesNum][0]] = Indi->Sequence[i];
//         }
//     }
//     TaskRoutes[RoutesNum][0]++;
//     TaskRoutes[RoutesNum][TaskRoutes[RoutesNum][0]] = 0;

//     // NodeRoutes: each row is the actual node sequence of each sub-route
//     for (i = 1; i <= RoutesNum; i++)
//     {
//         NodeRoutes[i][0] = 0;
//         for (j = 1; j < TaskRoutes[i][0]; j++)
//         {
//             for (k = 1; k <= shortest_path[inst_tasks[TaskRoutes[i][j]].tail_node][inst_tasks[TaskRoutes[i][j+1]].head_node][0]; k++)
//             {
//                 NodeRoutes[i][0] ++;
//                 NodeRoutes[i][NodeRoutes[i][0]] = shortest_path[inst_tasks[TaskRoutes[i][j]].tail_node][inst_tasks[TaskRoutes[i][j+1]].head_node][k];
//             }
//         }
//     }

//     // InRoutes: the row TID save all routes ID where task TID located
//     // e.g. InRoutes[1] = {1, 2, 3}, task 1 exists in route 1, 2, 3.
//     for (i = 1; i <= NRE+NRA; i++)
//     {
//         InRoutes[i][0] = 0;
//     }
//     for (i=1; i<=RoutesNum; i++)
//     {
//         memset(CheckMark, 0, sizeof(CheckMark));
//         for (j=1; j < NodeRoutes[i][0]; j++)
//         {
//             int TID = FindTask(NodeRoutes[i][j], NodeRoutes[i][j+1], inst_tasks, NO_Task);
//             if (TID > NRE)
//                 TID -= NRE;
//             if (TID == 0) // if TID==0, it is not a task
//                 continue;

//             if (!CheckMark[TID])
//             {
//                 CheckMark[TID] = 1;
//                 InRoutes[TID][0] ++;
//                 InRoutes[TID][InRoutes[TID][0]] = i;
//             }
//         }
//     }

//     // FreeTasks: tasks that exists in over one routes
//     // tasks only exist in one route is ignored because it can't be assigned into different routes.
//     int FreeTasks[NRE + NRA + 1];
//     FreeTasks[0] = 0;
//     for (i = 1; i <= NRE + NRA; i++)
//     {
//         if (InRoutes[i][0] == 1) // tasks only exist in one route;
//             continue;
//         FreeTasks[0] ++;
//         FreeTasks[FreeTasks[0]] = i;
//     }


//     struct TaskAssignment
//     {
//         int Assignment[300]; // which route is assigned for each task.
//         int Loads[101];
//         int TotalVioLoad;
//     };

//     struct TaskAssignment CurrTA, NeighTA, NextTA, BestTA;

//     memset(CurrTA.Assignment, 0, sizeof(CurrTA.Assignment));
//     memset(CurrTA.Loads, 0, sizeof(CurrTA.Loads));
//     CurrTA.Loads[0] = RoutesNum;

//     for (i = 1; i <= NRE + NRA; i++)
//     {
//         if (InRoutes[i][0] == 1)
//         {
//             CurrTA.Assignment[i] = InRoutes[i][1];
//             CurrTA.Loads[InRoutes[i][1]] += inst_tasks[i].demand;
//         }
//     }

//     int LeftTasks[NRE+NRA+1], RouteCandDemands[RoutesNum+1], CandRoutes[NRE+NRA+1][101];
//     AssignArray(FreeTasks, LeftTasks);
//     memset(RouteCandDemands, 0, sizeof(RouteCandDemands));
//     for(i = 1; i <= NRE+NRA; i++)
//     {
//         CandRoutes[i][0] = 0; // The routes which task i can be assigned.
//         for (j = 1; j <= InRoutes[i][0]; j++)
//         {
//             if (CurrTA.Loads[InRoutes[i][j]] >= capacity)
//                 continue;

//             CandRoutes[i][0] ++;
//             CandRoutes[i][CandRoutes[i][0]] = InRoutes[i][j];
//         }
//     }

//     // RouteCandDemands[i] denotes Route[i].
//     // The value of RouteCandDemands[i] represents the total demand of all possible assigned tasks.
//     for (i = 1; i <= LeftTasks[0]; i++)
//     {
//         for (j = 1; j <= InRoutes[LeftTasks[i]][0]; j++)
//         {
//             RouteCandDemands[InRoutes[LeftTasks[i]][j]] += inst_tasks[LeftTasks[i]].demand;
//         }
//     }

//     int SelID, AssignedRouteID;
//     int n;
//     for (n = 1; n <= FreeTasks[0]; n++)
//     {
//         SelID = 0; // select item
//         for (i = 1; i <= LeftTasks[0]; i++)
//         {
//             if (SelID == 0)
//             {
//                 SelID = i;
//             } else if (CandRoutes[LeftTasks[i]][0] < CandRoutes[LeftTasks[SelID]][0]) // select smallest _|^|_
//             {
//                 SelID = i;
//             } else if (CandRoutes[LeftTasks[i]][0] == CandRoutes[LeftTasks[SelID]][0])
//             {
//                 if (inst_tasks[LeftTasks[i]].demand > inst_tasks[LeftTasks[SelID]].demand) // select the max demand
//                 {
//                     SelID = i;
//                 }
//             }
//         }

//         AssignedRouteID = 0; // select bin
//         for (i = 1; i <= InRoutes[LeftTasks[SelID]][0]; i++)
//         {
//             if (AssignedRouteID == 0)
//             {
//                 AssignedRouteID = i;
//             }
//             else if (CurrTA.Loads[InRoutes[LeftTasks[SelID]][i]]+inst_tasks[LeftTasks[SelID]].demand <
//                      CurrTA.Loads[InRoutes[LeftTasks[SelID]][AssignedRouteID]]+inst_tasks[LeftTasks[SelID]].demand)
//             {
//                 AssignedRouteID = i;
//             }
//             else if (CurrTA.Loads[InRoutes[LeftTasks[SelID]][i]]+inst_tasks[LeftTasks[SelID]].demand ==
//                      CurrTA.Loads[InRoutes[LeftTasks[SelID]][AssignedRouteID]]+inst_tasks[LeftTasks[SelID]].demand)
//             {
//                 if (RouteCandDemands[InRoutes[LeftTasks[SelID]][i]]+CurrTA.Loads[InRoutes[LeftTasks[SelID]][i]] >
//                     RouteCandDemands[InRoutes[LeftTasks[SelID]][AssignedRouteID]]+
//                     CurrTA.Loads[InRoutes[LeftTasks[SelID]][AssignedRouteID]])
//                 {
//                     AssignedRouteID = i;
//                 }
//             }
//         }

//         CurrTA.Assignment[LeftTasks[SelID]] = InRoutes[LeftTasks[SelID]][AssignedRouteID];
//         CurrTA.Loads[CurrTA.Assignment[LeftTasks[SelID]]] += inst_tasks[LeftTasks[SelID]].demand;
//         for (i = 1; i <= InRoutes[LeftTasks[SelID]][0]; i++)
//         {
//             RouteCandDemands[InRoutes[LeftTasks[SelID]][i]] -= inst_tasks[LeftTasks[SelID]].demand;
//         }

//         if (CurrTA.Loads[CurrTA.Assignment[LeftTasks[SelID]]] >= capacity)
//         {
//             for (i = 1; i <= NRE+NRA; i++)
//             {
//                 for (j = 1; j <= CandRoutes[i][0]; j++)
//                 {
//                     if (CandRoutes[i][j] == CurrTA.Assignment[LeftTasks[SelID]])
//                     {
//                         delete_element(CandRoutes[i], j);
//                         break;
//                     }
//                 }
//             }
//         }
//         delete_element(LeftTasks, SelID);
//     }
//     CurrTA.TotalVioLoad = 0;
//     for (i = 1; i <= RoutesNum; i++)
//     {
//         if (CurrTA.Loads[i] > capacity)
//             CurrTA.TotalVioLoad += CurrTA.Loads[i]-capacity;
//     }

//     BestTA = CurrTA;

//     // tabu search for repairing
//     int TabuList[NRE+NRA+1];
//     int TabuTenure = FreeTasks[0]/2;
//     int TabuTask, Tabu;
//     memset(TabuList, 0, sizeof(TabuList));

//     int count = 0;
//     int stlcount = 0;
//     while (BestTA.TotalVioLoad > 0)
//     {
//         count ++;
//         stlcount ++;
//         NextTA.TotalVioLoad = INF;
//         for (i = 1; i <= FreeTasks[0]; i++)
//         {
//             for (j = 1; j <= InRoutes[FreeTasks[i]][0]; j++)
//             {
//                 if (InRoutes[FreeTasks[i]][j] == CurrTA.Assignment[FreeTasks[i]])
//                     continue;

//                 NeighTA = CurrTA;
//                 NeighTA.Assignment[FreeTasks[i]] = InRoutes[FreeTasks[i]][j];
//                 NeighTA.Loads[CurrTA.Assignment[FreeTasks[i]]] -= inst_tasks[FreeTasks[i]].demand;
//                 NeighTA.Loads[NeighTA.Assignment[FreeTasks[i]]] += inst_tasks[FreeTasks[i]].demand;

//                 if (CurrTA.Loads[NeighTA.Assignment[FreeTasks[i]]] >= capacity)
//                 {
//                     NeighTA.TotalVioLoad += inst_tasks[FreeTasks[i]].demand;
//                 }
//                 else if (NeighTA.Loads[NeighTA.Assignment[FreeTasks[i]]] > capacity)
//                 {
//                     NeighTA.TotalVioLoad += NeighTA.Loads[NeighTA.Assignment[FreeTasks[i]]]-capacity;
//                 }

//                 if (NeighTA.Loads[CurrTA.Assignment[FreeTasks[i]]] >= capacity)
//                 {
//                     NeighTA.TotalVioLoad -= inst_tasks[FreeTasks[i]].demand;
//                 }
//                 else if (CurrTA.Loads[CurrTA.Assignment[FreeTasks[i]]] > capacity)
//                 {
//                     NeighTA.TotalVioLoad -= CurrTA.Loads[CurrTA.Assignment[FreeTasks[i]]]-capacity;
//                 }

//                 Tabu = 0;
//                 if (TabuList[FreeTasks[i]] > 0 && NeighTA.TotalVioLoad >= BestTA.TotalVioLoad)
//                     Tabu = 1;

//                 if (Tabu)
//                     continue;

//                 if (NeighTA.TotalVioLoad < NextTA.TotalVioLoad)
//                 {
//                     NextTA = NeighTA;
//                     TabuTask = FreeTasks[i];
//                 }
//             }
//         }
//         for (i = 1; i <= NRE + NRA; i++)
//         {
//             if (TabuList[i] > 0)
//                 TabuList[i] --;
//         }

//         TabuList[TabuTask] = TabuTenure;
//         CurrTA = NextTA;
//         if (CurrTA.TotalVioLoad < BestTA.TotalVioLoad)
//         {
//             stlcount = 0;
//             BestTA = CurrTA;
//         }

//         if (count == 2*(NRA+NRA) || stlcount == (NRE + NRA) / 2)
//             break;
//     }
//     if (BestTA.TotalVioLoad > Indi->TotalVioLoad)
//         return;

//     int Served[NRA + NRE + 1];
//     memset(Served, 0, sizeof(Served));

//     Indi->Sequence[0] = 1;
//     Indi->Sequence[1] = 0;

//     for (i=1; i <= RoutesNum; i++)
//     {
//         for (j = 1; j < NodeRoutes[i][0]; j++)
//         {
//             int TID = FindTask(NodeRoutes[i][j], NodeRoutes[i][j+1], inst_tasks, NO_Task);
//             int TmpTID = TID;

//             if (TmpTID > NRE)
//                 TmpTID -= NRE;

//             if (TmpTID == 0)
//                 continue;

//             if (BestTA.Assignment[TmpTID] == i && !Served[TmpTID])
//             {
//                 Served[TmpTID] = 1;
//                 Indi->Sequence[0] ++;
//                 Indi->Sequence[Indi->Sequence[0]] = TID;
//             }

//         }
//         if (Indi->Sequence[Indi->Sequence[0]] != 0)
//         {
//             Indi->Sequence[0] ++;
//             Indi->Sequence[Indi->Sequence[0]] = 0;
//         }
//     }

//     for(i = BestTA.Loads[0]; i > 0; i--)
//     {
//         if (BestTA.Loads[i] == 0)
//             delete_element(BestTA.Loads, i);
//     }

//     Indi->TotalCost = get_task_seq_total_cost(Indi->Sequence, inst_tasks);
//     AssignArray(BestTA.Loads, Indi->Loads);
//     Indi->TotalVioLoad = BestTA.TotalVioLoad;

// }