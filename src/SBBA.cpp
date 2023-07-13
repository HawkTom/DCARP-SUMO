//
// Created by 57725 on 2022/10/21.
//
#include "src.h"
// #include "../utils.h"

void construct_old_solution(CARPInd *old_sol, Vehicles state, const Task *inst_tasks);


#define FULL 1
#define PARTIAL 0
experience::experience()
{
    this->costScale = 0;
    this->LB = 0;
}

void experience::save_best_so_far_queue(CARPInd sol)
{
    this->queue_index ++;
    int curr_idx = this->next;
    if (curr_idx > 29)
    {
        this->next = 0;
        curr_idx = this->next;
    }
    for (int i=0; i<=sol.Sequence[0]; i++)
    {
        this->best_so_far[curr_idx][i] = sol.Sequence[i];
    }
    this->next ++;
}
void experience::move_best_to_first()
{
    int data_num;
    if (this->queue_index > 30)
    {
        data_num = 30;
    } else {
        data_num = this->queue_index;
    }

    int min_idx, min_cost = INF;
    for (int i = 0; i<data_num; i++)
    {
        if (this->save_cost[i] < min_cost)
        {
            min_cost = this->save_cost[i];
            min_idx = i;
        }
    }
    if (min_idx != 0)
    {
        int tmp;
        for (int i=0; i<400; i++)
        {
            tmp = this->best_so_far[min_idx][i];
            this->best_so_far[min_idx][i] = this->best_so_far[0][i];
            this->best_so_far[0][i] = this->best_so_far[min_idx][i];;
        }
    }
}

int experience::save_best_so_far(CARPInd sol)
{
    this->queue_index ++;
    int index = this->queue_index-1;
//    int index = (this->queue_index-1)%30;
    for (int i=0; i<=sol.Sequence[0]; i++)
    {
        this->best_so_far[index][i] = sol.Sequence[i];
        this->save_cost[index] = sol.TotalCost;
    }
    return this->queue_index;
}

void experience::save_solutions(const Task *inst_tasks)
{
    char path[100];
    memset(path, 0, sizeof(path));
    sprintf(path, "data/s%d-sols%d.txt", scenario_idx, instance_idx);
    FILE *fp;
    fp = fopen(path, "w");
    int data_num;
    if (this->queue_index > 30)
    {
        data_num = 30;
    } else {
        data_num = this->queue_index;
    }
    fprintf(fp, "num,%d\n", data_num);
    int i,j, head, tail;
    for (i=0; i<data_num; i++)
    {
        for (j=1; j<this->best_so_far[i][0]; j++)
        {
            if (inst_tasks[this->best_so_far[i][j]].vt > 0)
            {
                continue;
            }
            if (this->best_so_far[i][j] == 0)
            {
                fprintf(fp, "0,");
            } else {
                head = inst_tasks[this->best_so_far[i][j]].head_node_r;
                tail = inst_tasks[this->best_so_far[i][j]].tail_node_r;
                fprintf(fp, "%d,%d,", head, tail);
            }
        }
        fprintf(fp, "0,-1\n");
    }
    fclose(fp);
}

repair_old_solutions::repair_old_solutions(const Task *inst_tasks, CARPInd old_sol) {

    this->sols = new CARPInd[35];

    this->get_inherited_solution(old_sol);
//    this->get_cps_solution(inst_tasks);

    copy_individual(&this->sols[0], &this->inherited_sol);
    this->sol_num = 1;
//    copy_individual(&this->sols[1], &this->inherited_sol);
//    this->sol_num = 2;

    int NO_task = inst_tasks[inst_tasks[1].inverse - 1].inverse;
    char path[100];
    memset(path, 0, sizeof(path));
    if (instance_idx == 1){
        sprintf(path, "data/s%d-sols%d.txt", scenario_idx, instance_idx-1);
    } else {
        sprintf(path, "data/s%d-sols%d.txt", scenario_idx, instance_idx-1);
    }
//    sprintf(path, "data/%s/s%d-sols%d.txt", map, scn_num, dym_t-1);
//    sprintf(path, "data/%s/sols%d.txt", map, dym_t-1);
    FILE *fp;
    fp = fopen(path, "r");

    int i, j, task_id;
    int node1, node2;
    int data_num;
    fscanf(fp, "num,%d\n", &data_num);
    int tmp_sol_seq[MAX_TASK_SEG_LENGTH];
    int task_num_cal;
    CARPInd tmp_sol;
    for (i=0;i<data_num;i++)
    {
        memset(tmp_sol_seq, 0, sizeof(tmp_sol_seq));
        task_num_cal = 0;
        tmp_sol_seq[0] = 0;
        while (fscanf(fp, "%d,", &node1))
        {
            if (node1 == -1)
            {
                break;
            }
            if (node1 == 0)
            {
                tmp_sol_seq[0]++;
                tmp_sol_seq[tmp_sol_seq[0]] = 0;
                continue;
            }
            fscanf(fp, "%d,", &node2);
            task_id = FindTask(node1, node2, inst_tasks, NO_task);
            if (task_id == 0)
            {
                task_id = -1;
            }else{
                task_num_cal++;
            }
            tmp_sol_seq[0]++;
            tmp_sol_seq[tmp_sol_seq[0]] = task_id;
        }
        // repair tmp_sol_seq;

        int loads[100];
        memset(loads, 0, sizeof(loads));
        get_task_seq_loads(loads, tmp_sol_seq, inst_tasks);

        this->repair(&tmp_sol, tmp_sol_seq, inst_tasks);
        int used = 0;
        for (j=0; j<this->sol_num; j++)
        {
            if (tmp_sol.TotalCost == this->sols[j].TotalCost && tmp_sol.TotalVioLoad == this->sols[j].TotalVioLoad)
            {
                used = 1;
                break;
            }
        }
        if (used) continue;

        copy_individual(&this->sols[this->sol_num], &tmp_sol);
        this->sol_num ++;
    }
    fclose(fp);
}

// void repair_old_solutions::get_inherited_solution(Vehicles state, const Task *inst_tasks) {
//     construct_old_solution(&this->inherited_sol, state, inst_tasks);
// }


void repair_old_solutions::get_inherited_solution(CARPInd old_sol) {
    copy_individual(&this->inherited_sol, &old_sol);
}

// void repair_old_solutions::get_cps_solution(const Task *inst_tasks)
// {
//     int split_seqs[MAX_TASK_SEQ_LENGTH];
//     detect_influenced_path(this->inherited_sol.Sequence, inst_tasks, split_seqs);

//     CARPInd ex_ps_solution;
//     building_block seg[400];
//     memset(seg->seqs, 0, sizeof(int)*100);

//     int seg_num = inherited_cluster(seg, split_seqs, inst_tasks);
//     building_block_path_scanning(&this->cps_sol, seg, seg_num, FULL, inst_tasks);
// //    printf("cps cost:%d\n", this->cps_sol.TotalCost);
// }

void repair_old_solutions::repair(CARPInd *tmp_sol, int *sol_seq, const Task *inst_tasks)
{
    building_block seg[400];
    memset(seg->seqs, 0, sizeof(int)*100);
    int i,j;
    int seg_num = old_solution_block(sol_seq, inst_tasks, FULL, seg);

    building_block_path_scanning(tmp_sol, seg, seg_num, FULL, inst_tasks);
//    printf("repair cost:%d\n", this->sols[sol_idx].TotalCost);
}

int inherited_cluster(building_block *seg, const int *split_seqs, const Task *inst_tasks)
{
    int i, j;
    int is_influenced = 0;
    for (i=1; i <= split_seqs[0]; i++)
    {
        if (split_seqs[i] == -1)
        {
            is_influenced = 1;
            break;
        }
    }

    if (is_influenced == 0)
    {
        return 0;
    }

    // split the route into several segment according to if the link between tasks are affected by dynamic events or not.
    int seg_num = 0;
    for (i=1; i<=split_seqs[0]; i++)
    {
        if ((split_seqs[i] == 0 || split_seqs[i] == -1) && split_seqs[i-1] > 0)
        {
            seg[seg_num].head = inst_tasks[seg[seg_num].seqs[1]].head_node;
            seg[seg_num].tail = inst_tasks[seg[seg_num].seqs[seg[seg_num].seqs[0]]].tail_node;
            seg_num++;
            seg[seg_num].seqs[0] = 0;
            continue;
        }
        if (split_seqs[i] == 0)
            continue;
        if (split_seqs[i] == -1)
            continue;
        seg[seg_num].seqs[0] ++;
        seg[seg_num].seqs[seg[seg_num].seqs[0]] = split_seqs[i];
    }
    seg_num --;


    int curr_loads, serv_costs;
    for (i=1; i<=seg_num;i++)
    {
        curr_loads = 0;
        serv_costs = 0;
        for (j=1; j<=seg[i].seqs[0]; j++)
        {
            curr_loads += inst_tasks[seg[i].seqs[j]].demand;
        }
        int a, b;
        for (j=1; j<seg[i].seqs[0]; j++)
        {
            serv_costs += inst_tasks[seg[i].seqs[j]].serv_cost;
            a = inst_tasks[seg[i].seqs[j]].tail_node;
            b = inst_tasks[seg[i].seqs[j+1]].head_node;
            serv_costs += min_cost[a][b];
        }
        serv_costs += inst_tasks[seg[i].seqs[j]].serv_cost;

        seg[i].loads = curr_loads;
        seg[i].dep_dist = min_cost[seg[i].tail][DEPOT];
        seg[i].serv_costs = serv_costs;
        seg[i].yield = 1.0*seg[i].loads / serv_costs;
        seg[i].inverse = i+seg_num;

        if (inst_tasks[seg[i].seqs[1]].vt > 0)
        {
            seg[i+seg_num].head = seg[i].head;
            seg[i+seg_num].tail = seg[i].tail;
            seg[i+seg_num].seqs[0] = 0;
            for (j=1; j<=seg[i].seqs[0]; j++)
            {
                seg[i+seg_num].seqs[0]++;
                seg[i+seg_num].seqs[seg[i+seg_num].seqs[0]] = seg[i].seqs[j];
            }
        } else {
            seg[i+seg_num].head = seg[i].tail;
            seg[i+seg_num].tail = seg[i].head;
            seg[i+seg_num].seqs[0] = 0;
            for (j=seg[i].seqs[0]; j>=1; j--)
            {
                seg[i+seg_num].seqs[0]++;
                seg[i+seg_num].seqs[seg[i+seg_num].seqs[0]] = inst_tasks[seg[i].seqs[j]].inverse;
            }
        }

        seg[i+seg_num].loads = seg[i].loads;
        seg[i+seg_num].dep_dist = min_cost[seg[i+seg_num].tail][DEPOT];;
        seg[i+seg_num].yield = seg[i].yield;
        seg[i+seg_num].serv_costs = seg[i].serv_costs;
        seg[i+seg_num].inverse = i;
    }
    seg_num *= 2;
    return seg_num;
}

void building_block_path_scanning(CARPInd *ps_sol, building_block *seg, int seg_num, int type, const Task *inst_tasks)
{
    int i;
    int tmp_dist, min_dist;
    int used[seg_num+1];
    int candi_segs[400];
    int nearest_isol_seg[400], nearest_inci_seg[400], sel_seg[400];

    CARPInd tmp_indi1, tmp_indi2, tmp_indi3, tmp_indi4, tmp_indi5;
    int curr_node, curr_loads, next_seg;

    ps_sol->TotalCost = INF;

    // Use Rule 1 to obtain a solution: maximize the distance from the head of task to the depot
    curr_node = 0;
    curr_loads = 0;
    memset(used, 0, sizeof(used));
    tmp_indi1.Sequence[0] = 1;
    tmp_indi1.Sequence[1] = 0;
    while (used[0] < seg_num)
    {
        curr_node = inst_tasks[tmp_indi1.Sequence[tmp_indi1.Sequence[0]]].tail_node;
        min_dist = INF;
        candi_segs[0] = 0;
        for (i=1; i<=seg_num; i++)
        {
            if (seg[i].loads + curr_loads > capacity || used[i])
                continue;

            tmp_dist = min_cost[curr_node][seg[i].head];
            if (tmp_dist < min_dist)
            {
                min_dist = tmp_dist;
                candi_segs[0] = 1;
                candi_segs[1] = i;
            } else if (tmp_dist == min_dist)
            {
                candi_segs[0]++;
                candi_segs[candi_segs[0]] = i;
            }
        }
        if(candi_segs[0] == 0)
        {
            tmp_indi1.Sequence[0]++;
            tmp_indi1.Sequence[tmp_indi1.Sequence[0]] = 0;
            curr_loads = 0;
            continue;
        }

        nearest_inci_seg[0] = 0;
        nearest_isol_seg[0] = 0;
        for (i=1; i<=candi_segs[0]; i++)
        {
            if (seg[candi_segs[i]].tail == 1)
            {
                nearest_inci_seg[0] ++;
                nearest_inci_seg[nearest_inci_seg[0]] = candi_segs[i];
            } else{
                nearest_isol_seg[0] ++;
                nearest_isol_seg[nearest_isol_seg[0]] = candi_segs[i];
            }
        }
        if (nearest_isol_seg[0] == 0)
        {
            memcpy(nearest_isol_seg, nearest_inci_seg, (nearest_inci_seg[0]+1)*sizeof(int));
        }
        // for 5 five phase, the above part is the same
        int max_dep_dist = -1;
        sel_seg[0] = 0;
        for (i=1; i<=nearest_isol_seg[0]; i++)
        {
            if(seg[nearest_isol_seg[i]].dep_dist > max_dep_dist)
            {
                max_dep_dist = seg[nearest_isol_seg[i]].dep_dist;
                sel_seg[0] = 1;
                sel_seg[1] = nearest_isol_seg[i];
            } else if (seg[nearest_isol_seg[i]].dep_dist == max_dep_dist)
            {
                sel_seg[0]++;
                sel_seg[sel_seg[0]] = nearest_isol_seg[i];
            }
        }
        next_seg = sel_seg[1];
        for (i = 1; i <= seg[next_seg].seqs[0]; i++) {
            tmp_indi1.Sequence[0]++;
            tmp_indi1.Sequence[tmp_indi1.Sequence[0]] = seg[next_seg].seqs[i];
        }
        curr_loads += seg[next_seg].loads;
        used[0]+=2;
        used[next_seg] = 1;
        used[seg[next_seg].inverse] = 1;
    }
    tmp_indi1.Sequence[0] ++;
    tmp_indi1.Sequence[tmp_indi1.Sequence[0]] = 0;
    tmp_indi1.TotalCost = get_task_seq_total_cost(tmp_indi1.Sequence, inst_tasks);

    if (tmp_indi1.TotalCost < ps_sol->TotalCost)
    {
        copy_individual(ps_sol, &tmp_indi1);
    }

    // Use Rule 2 to obtain a solution: minimize the distance from the head of task to the depot
    curr_node = 0;
    curr_loads = 0;
    memset(used, 0, sizeof(used));
    tmp_indi2.Sequence[0] = 1;
    tmp_indi2.Sequence[1] = 0;
    while (used[0] < seg_num)
    {
        curr_node = inst_tasks[tmp_indi2.Sequence[tmp_indi2.Sequence[0]]].tail_node;
        min_dist = INF;
        candi_segs[0] = 0;
        for (i=1; i<=seg_num; i++)
        {
            if (seg[i].loads + curr_loads > capacity || used[i])
                continue;

            tmp_dist = min_cost[curr_node][seg[i].head];
            if (tmp_dist < min_dist)
            {
                min_dist = tmp_dist;
                candi_segs[0] = 1;
                candi_segs[1] = i;
            } else if (tmp_dist == min_dist)
            {
                candi_segs[0]++;
                candi_segs[candi_segs[0]] = i;
            }
        }
        if(candi_segs[0] == 0)
        {
            tmp_indi2.Sequence[0]++;
            tmp_indi2.Sequence[tmp_indi2.Sequence[0]] = 0;
            curr_loads = 0;
            continue;
        }

        nearest_inci_seg[0] = 0;
        nearest_isol_seg[0] = 0;
        for (i=1; i<=candi_segs[0]; i++)
        {
            if (seg[candi_segs[i]].tail == 1)
            {
                nearest_inci_seg[0] ++;
                nearest_inci_seg[nearest_inci_seg[0]] = candi_segs[i];
            } else{
                nearest_isol_seg[0] ++;
                nearest_isol_seg[nearest_isol_seg[0]] = candi_segs[i];
            }
        }
        if (nearest_isol_seg[0] == 0)
        {
            memcpy(nearest_isol_seg, nearest_inci_seg, (nearest_inci_seg[0]+1)*sizeof(int));
        }
        // for 5 five phase, the above part is the same
        int min_dep_dist = INF;
        sel_seg[0] = 0;
        for (i=1; i<=nearest_isol_seg[0]; i++)
        {
            if(seg[nearest_isol_seg[i]].dep_dist < min_dep_dist)
            {
                min_dep_dist = seg[nearest_isol_seg[i]].dep_dist;
                sel_seg[0] = 1;
                sel_seg[1] = nearest_isol_seg[i];
            } else if (seg[nearest_isol_seg[i]].dep_dist == min_dep_dist)
            {
                sel_seg[0]++;
                sel_seg[sel_seg[0]] = nearest_isol_seg[i];
            }
        }
        next_seg = sel_seg[1];
        for (i = 1; i <= seg[next_seg].seqs[0]; i++) {
            tmp_indi2.Sequence[0]++;
            tmp_indi2.Sequence[tmp_indi2.Sequence[0]] = seg[next_seg].seqs[i];
        }
        curr_loads += seg[next_seg].loads;
        used[0]+=2;
        used[next_seg] = 1;
        used[seg[next_seg].inverse] = 1;
    }
    tmp_indi2.Sequence[0] ++;
    tmp_indi2.Sequence[tmp_indi2.Sequence[0]] = 0;
    tmp_indi2.TotalCost = get_task_seq_total_cost(tmp_indi2.Sequence, inst_tasks);
    if (tmp_indi2.TotalCost < ps_sol->TotalCost)
    {
        copy_individual(ps_sol, &tmp_indi2);
    }

    // Use Rule 3 to obtain a solution: maximize the term dem(t)/sc(t),
    curr_node = 0;
    curr_loads = 0;
    memset(used, 0, sizeof(used));
    tmp_indi3.Sequence[0] = 1;
    tmp_indi3.Sequence[1] = 0;
    while (used[0] < seg_num)
    {
        curr_node = inst_tasks[tmp_indi3.Sequence[tmp_indi3.Sequence[0]]].tail_node;
        min_dist = INF;
        candi_segs[0] = 0;
        for (i=1; i<=seg_num; i++)
        {
            if (seg[i].loads + curr_loads > capacity || used[i])
                continue;

            tmp_dist = min_cost[curr_node][seg[i].head];
            if (tmp_dist < min_dist)
            {
                tmp_dist = min_dist;
                candi_segs[0] = 1;
                candi_segs[1] = i;
            } else if (tmp_dist == min_dist)
            {
                candi_segs[0]++;
                candi_segs[candi_segs[0]] = i;
            }
        }
        if(candi_segs[0] == 0)
        {
            tmp_indi3.Sequence[0]++;
            tmp_indi3.Sequence[tmp_indi3.Sequence[0]] = 0;
            curr_loads = 0;
            continue;
        }

        nearest_inci_seg[0] = 0;
        nearest_isol_seg[0] = 0;
        for (i=1; i<=candi_segs[0]; i++)
        {
            if (seg[candi_segs[i]].tail == 1)
            {
                nearest_inci_seg[0] ++;
                nearest_inci_seg[nearest_inci_seg[0]] = candi_segs[i];
            } else{
                nearest_isol_seg[0] ++;
                nearest_isol_seg[nearest_isol_seg[0]] = candi_segs[i];
            }
        }
        if (nearest_isol_seg[0] == 0)
        {
            memcpy(nearest_isol_seg, nearest_inci_seg, (nearest_inci_seg[0]+1)*sizeof(int));
        }
        // for 5 five phase, the above part is the same
        double max_yield = -1;
        sel_seg[0] = 0;
        for (i=1; i<=nearest_isol_seg[0]; i++)
        {
            if(seg[nearest_isol_seg[i]].yield > max_yield)
            {
                max_yield = seg[nearest_isol_seg[i]].yield;
                sel_seg[0] = 1;
                sel_seg[1] = nearest_isol_seg[i];
            } else if (seg[nearest_isol_seg[i]].yield == max_yield)
            {
                sel_seg[0]++;
                sel_seg[sel_seg[0]] = nearest_isol_seg[i];
            }
        }
        next_seg = sel_seg[1];
        for (i = 1; i <= seg[next_seg].seqs[0]; i++) {
            tmp_indi3.Sequence[0]++;
            tmp_indi3.Sequence[tmp_indi3.Sequence[0]] = seg[next_seg].seqs[i];
        }
        curr_loads += seg[next_seg].loads;
        used[0]+=2;
        used[next_seg] = 1;
        used[seg[next_seg].inverse] = 1;
    }
    tmp_indi3.Sequence[0] ++;
    tmp_indi3.Sequence[tmp_indi3.Sequence[0]] = 0;
    tmp_indi3.TotalCost = get_task_seq_total_cost(tmp_indi3.Sequence, inst_tasks);
    if (tmp_indi3.TotalCost < ps_sol->TotalCost)
    {
        copy_individual(ps_sol, &tmp_indi3);
    }

    // Use Rule 4 to obtain a solution: minimize the term dem(t)/sc(t),
    curr_node = 0;
    curr_loads = 0;
    memset(used, 0, sizeof(used));
    tmp_indi4.Sequence[0] = 1;
    tmp_indi4.Sequence[1] = 0;
    while (used[0] < seg_num)
    {
        curr_node = inst_tasks[tmp_indi4.Sequence[tmp_indi4.Sequence[0]]].tail_node;
        min_dist = INF;
        candi_segs[0] = 0;
        for (i=1; i<=seg_num; i++)
        {
            if (seg[i].loads + curr_loads > capacity || used[i])
                continue;

            tmp_dist = min_cost[curr_node][seg[i].head];
            if (tmp_dist < min_dist)
            {
                min_dist = tmp_dist;
                candi_segs[0] = 1;
                candi_segs[1] = i;
            } else if (tmp_dist == min_dist)
            {
                candi_segs[0]++;
                candi_segs[candi_segs[0]] = i;
            }
        }
        if(candi_segs[0] == 0)
        {
            tmp_indi4.Sequence[0]++;
            tmp_indi4.Sequence[tmp_indi4.Sequence[0]] = 0;
            curr_loads = 0;
            continue;
        }

        nearest_inci_seg[0] = 0;
        nearest_isol_seg[0] = 0;
        for (i=1; i<=candi_segs[0]; i++)
        {
            if (seg[candi_segs[i]].tail == 1)
            {
                nearest_inci_seg[0] ++;
                nearest_inci_seg[nearest_inci_seg[0]] = candi_segs[i];
            } else{
                nearest_isol_seg[0] ++;
                nearest_isol_seg[nearest_isol_seg[0]] = candi_segs[i];
            }
        }
        if (nearest_isol_seg[0] == 0)
        {
            memcpy(nearest_isol_seg, nearest_inci_seg, (nearest_inci_seg[0]+1)*sizeof(int));
        }
        // for 5 five phase, the above part is the same
        double min_yield = INF;
        sel_seg[0] = 0;
        for (i=1; i<=nearest_isol_seg[0]; i++)
        {
            if(seg[nearest_isol_seg[i]].yield < min_yield)
            {
                min_yield = seg[nearest_isol_seg[i]].yield;
                sel_seg[0] = 1;
                sel_seg[1] = nearest_isol_seg[i];
            } else if (seg[nearest_isol_seg[i]].yield == min_yield)
            {
                sel_seg[0]++;
                sel_seg[sel_seg[0]] = nearest_isol_seg[i];
            }
        }
        next_seg = sel_seg[1];
        for (i = 1; i <= seg[next_seg].seqs[0]; i++) {
            tmp_indi4.Sequence[0]++;
            tmp_indi4.Sequence[tmp_indi4.Sequence[0]] = seg[next_seg].seqs[i];
        }
        curr_loads += seg[next_seg].loads;
        used[0]+=2;
        used[next_seg] = 1;
        used[seg[next_seg].inverse] = 1;
    }
    tmp_indi4.Sequence[0] ++;
    tmp_indi4.Sequence[tmp_indi4.Sequence[0]] = 0;
    tmp_indi4.TotalCost = get_task_seq_total_cost(tmp_indi4.Sequence, inst_tasks);
    if (tmp_indi4.TotalCost < ps_sol->TotalCost)
    {
        copy_individual(ps_sol, &tmp_indi4);
    }

    // Use Rule 5 to obtain a solution: use rule 1) if the vehicle is less than halffull,
    //otherwise use rule 2).
    curr_node = 0;
    curr_loads = 0;
    memset(used, 0, sizeof(used));
    tmp_indi5.Sequence[0] = 1;
    tmp_indi5.Sequence[1] = 0;
    while (used[0] < seg_num)
    {
        curr_node = inst_tasks[tmp_indi5.Sequence[tmp_indi5.Sequence[0]]].tail_node;
        min_dist = INF;
        candi_segs[0] = 0;
        for (i=1; i<=seg_num; i++)
        {
            if (seg[i].loads + curr_loads > capacity || used[i])
                continue;

            tmp_dist = min_cost[curr_node][seg[i].head];
            if (tmp_dist < min_dist)
            {
                min_dist = tmp_dist;
                candi_segs[0] = 1;
                candi_segs[1] = i;
            } else if (tmp_dist == min_dist)
            {
                candi_segs[0]++;
                candi_segs[candi_segs[0]] = i;
            }
        }
        if(candi_segs[0] == 0)
        {
            tmp_indi5.Sequence[0]++;
            tmp_indi5.Sequence[tmp_indi5.Sequence[0]] = 0;
            curr_loads = 0;
            continue;
        }

        nearest_inci_seg[0] = 0;
        nearest_isol_seg[0] = 0;
        for (i=1; i<=candi_segs[0]; i++)
        {
            if (seg[candi_segs[i]].tail == 1)
            {
                nearest_inci_seg[0] ++;
                nearest_inci_seg[nearest_inci_seg[0]] = candi_segs[i];
            } else{
                nearest_isol_seg[0] ++;
                nearest_isol_seg[nearest_isol_seg[0]] = candi_segs[i];
            }
        }
        if (nearest_isol_seg[0] == 0)
        {
            memcpy(nearest_isol_seg, nearest_inci_seg, (nearest_inci_seg[0]+1)*sizeof(int));
        }
        // for 5 five phase, the above part is the same
        if (curr_loads * 2 < capacity)
        {
            int max_dep_dist = -1;
            sel_seg[0] = 0;
            for (i=1; i<=nearest_isol_seg[0]; i++)
            {
                if(seg[nearest_isol_seg[i]].dep_dist > max_dep_dist)
                {
                    max_dep_dist = seg[nearest_isol_seg[i]].dep_dist;
                    sel_seg[0] = 1;
                    sel_seg[1] = nearest_isol_seg[i];
                } else if (seg[nearest_isol_seg[i]].dep_dist == max_dep_dist)
                {
                    sel_seg[0]++;
                    sel_seg[sel_seg[0]] = nearest_isol_seg[i];
                }
            }
        } else
        {
            int min_dep_dist = INF;
            sel_seg[0] = 0;
            for (i=1; i<=nearest_isol_seg[0]; i++)
            {
                if(seg[nearest_isol_seg[i]].dep_dist < min_dep_dist)
                {
                    min_dep_dist = seg[nearest_isol_seg[i]].dep_dist;
                    sel_seg[0] = 1;
                    sel_seg[1] = nearest_isol_seg[i];
                } else if (seg[nearest_isol_seg[i]].dep_dist == min_dep_dist)
                {
                    sel_seg[0]++;
                    sel_seg[sel_seg[0]] = nearest_isol_seg[i];
                }
            }
        }
        next_seg = sel_seg[1];
        for (i = 1; i <= seg[next_seg].seqs[0]; i++) {
            tmp_indi5.Sequence[0]++;
            tmp_indi5.Sequence[tmp_indi5.Sequence[0]] = seg[next_seg].seqs[i];
        }
        curr_loads += seg[next_seg].loads;
        used[0]+=2;
        used[next_seg] = 1;
        used[seg[next_seg].inverse] = 1;
    }
    tmp_indi5.Sequence[0] ++;
    tmp_indi5.Sequence[tmp_indi5.Sequence[0]] = 0;
    tmp_indi5.TotalCost = get_task_seq_total_cost(tmp_indi5.Sequence, inst_tasks);
    if (tmp_indi5.TotalCost < ps_sol->TotalCost)
    {
        copy_individual(ps_sol, &tmp_indi5);
    }

    ps_sol->Loads[0] = 0;
    curr_loads = 0;
    for (i=2; i<=ps_sol->Sequence[0]; i++)
    {
        if (ps_sol->Sequence[i] == 0)
        {
            ps_sol->Loads[0]++;
            ps_sol->Loads[ps_sol->Loads[0]] = curr_loads;
            curr_loads = 0;
            continue;
        }
        curr_loads += inst_tasks[ps_sol->Sequence[i]].demand;
    }
    ps_sol->TotalVioLoad = get_total_vio_load(ps_sol->Loads);
    if (type == FULL)
    {
        check_solution_valid(*ps_sol, inst_tasks);
    }
}

int old_solution_block(int *sol_seq, const Task *inst_tasks, int type, building_block *seg)
{
    int i,j;

    int Positions[MAX_TASK_SEG_LENGTH];
    find_ele_positions(Positions, sol_seq, -1);

    int seg_num = 0;

    int block_split[MAX_TASK_SEG_LENGTH];
    block_split[0] = 1;
    block_split[1] = -1;
    for (i=1; i<=sol_seq[0]; i++)
    {
        if (sol_seq[i] == -1)
        {
            continue;
        }

        if(sol_seq[i] ==0 || sol_seq[i] == -1)
        {
            continue;
        }

        if (sol_seq[i]!=0 && sol_seq[i] != -1)
        {
            if (sol_seq[i-1] == 0 || sol_seq[i-1] == -1)
            {
                seg[seg_num].head = inst_tasks[seg[seg_num].seqs[1]].head_node;
                seg[seg_num].tail = inst_tasks[seg[seg_num].seqs[seg[seg_num].seqs[0]]].tail_node;
                seg_num ++;
                seg[seg_num].seqs[0] = 0;
            }
            seg[seg_num].seqs[0] ++;
            seg[seg_num].seqs[seg[seg_num].seqs[0]] = sol_seq[i];
        }
    }
    seg[seg_num].head = inst_tasks[seg[seg_num].seqs[1]].head_node;
    seg[seg_num].tail = inst_tasks[seg[seg_num].seqs[seg[seg_num].seqs[0]]].tail_node;

    // each new task is regarded as a new building block;
    if (new_tasks_list[0] > 0 && type == FULL)
    {
        for (i=1; i<=new_tasks_list[0];i++)
        {
            seg_num++;
            seg[seg_num].seqs[0] = 1;
            seg[seg_num].seqs[1] = new_tasks_list[i];
            seg[seg_num].head = inst_tasks[new_tasks_list[i]].head_node;
            seg[seg_num].tail = inst_tasks[new_tasks_list[i]].tail_node;
        }
    }


    // add each virtual task as an independent task
    int vt_num = 0;
    // for (i=inst_tasks[1].inverse-1; ; i--)
    // {
    //     if (inst_tasks[i].vt == 0)
    //         break;
    //     vt_num++;
    //     seg_num++;
    //     seg[seg_num].seqs[0] = 1;
    //     seg[seg_num].seqs[1] = i;
    //     seg[seg_num].head = DEPOT;
    //     seg[seg_num].tail = inst_tasks[i].tail_node;
    //     seg[seg_num].loads = inst_tasks[i].demand;
    //     seg[seg_num].serv_costs = inst_tasks[i].serv_cost;
    //     seg[seg_num].dep_dist = min_cost[seg[seg_num].tail][DEPOT];;
    //     seg[seg_num].yield = 1.0*seg[seg_num].loads / inst_tasks[i].serv_cost;
    // }
    for (i=1; i<inst_tasks[1].inverse; i++)
    {
        if (inst_tasks[i].vt == 0) continue;
        vt_num++;
        seg_num++;
        seg[seg_num].seqs[0] = 1;
        seg[seg_num].seqs[1] = i;
        seg[seg_num].head = DEPOT;
        seg[seg_num].tail = inst_tasks[i].tail_node;
        seg[seg_num].loads = inst_tasks[i].demand;
        seg[seg_num].serv_costs = inst_tasks[i].serv_cost;
        seg[seg_num].dep_dist = min_cost[seg[seg_num].tail][DEPOT];;
        seg[seg_num].yield = 1.0*seg[seg_num].loads / inst_tasks[i].serv_cost;
    }


    int curr_loads, serv_costs;
    for (i=1; i<=seg_num-vt_num;i++)
    {
        curr_loads = 0;
        serv_costs = 0;
        for (j=1; j<=seg[i].seqs[0]; j++)
        {
            curr_loads += inst_tasks[seg[i].seqs[j]].demand;
        }
        int a, b;
        for (j=1; j<seg[i].seqs[0]; j++)
        {
            serv_costs += inst_tasks[seg[i].seqs[j]].serv_cost;
            a = inst_tasks[seg[i].seqs[j]].tail_node;
            b = inst_tasks[seg[i].seqs[j+1]].head_node;
            serv_costs += min_cost[a][b];
        }
        serv_costs += inst_tasks[seg[i].seqs[j]].serv_cost;

        seg[i].loads = curr_loads;
        seg[i].dep_dist = min_cost[seg[i].tail][DEPOT];
        seg[i].serv_costs = serv_costs;
        seg[i].yield = 1.0*seg[i].loads / serv_costs;
        seg[i].inverse = i+seg_num;

        seg[i+seg_num].head = seg[i].tail;
        seg[i+seg_num].tail = seg[i].head;
        seg[i+seg_num].loads = seg[i].loads;
        seg[i+seg_num].dep_dist = min_cost[seg[i].head][DEPOT];;
        seg[i+seg_num].yield = seg[i].yield;
        seg[i+seg_num].serv_costs = seg[i].serv_costs;
        seg[i+seg_num].inverse = i;
        seg[i+seg_num].seqs[0] = 0;
        for (j=seg[i].seqs[0]; j>=1; j--)
        {
            seg[i+seg_num].seqs[0]++;
            seg[i+seg_num].seqs[seg[i+seg_num].seqs[0]] = inst_tasks[seg[i].seqs[j]].inverse;
        }
    }
    for(i=seg_num-vt_num+1; i<=seg_num; i++)
    {
        seg[i].inverse = i+seg_num;
        seg[i+seg_num].head = seg[i].head;
        seg[i+seg_num].tail = seg[i].tail;
        seg[i+seg_num].loads = seg[i].loads;
        seg[i+seg_num].serv_costs = seg[i].serv_costs;
        seg[i+seg_num].dep_dist = min_cost[seg[i].tail][DEPOT];;
        seg[i+seg_num].yield = seg[i].yield;
        seg[i+seg_num].inverse = i;
        seg[i+seg_num].seqs[0] = 1;
        seg[i+seg_num].seqs[1] = inst_tasks[seg[i].seqs[1]].inverse;
    }
    seg_num *= 2;
    return seg_num;

//    int a=1;
//    int b=1;
}

repair_old_solutions::~repair_old_solutions(){
    delete[] this->sols;
}

insert_new_tasks::insert_new_tasks(const Task *inst_tasks) {
    int NO_task = inst_tasks[inst_tasks[1].inverse - 1].inverse;
    char path[100];
    memset(path, 0, sizeof(path));
    if (instance_idx == 1){
        sprintf(path, "data/s%d-sols%d.txt", scenario_idx, instance_idx-1);
    } else {
        sprintf(path, "data/s%d-sols%d.txt", scenario_idx, instance_idx-1);
    }

//    sprintf(path, "data/%s/sols%d.txt", map, dym_t-1);
    FILE *fp;
    fp = fopen(path, "r");

    int i,j, task_id;
    int node1, node2;
    int data_num;
    fscanf(fp, "num,%d\n", &data_num);

    this->rsols = new CARPInd[data_num+1];
    this->pssols = new CARPInd[data_num+1];
//    this->psisols = new CARPInd[data_num+1];

    int tmp_seq[MAX_TASK_SEG_LENGTH];
    CARPInd tmp_sol;

    int task_num_cal;
    for (i=0;i<data_num;i++)
    {
        memset(tmp_seq, 0, sizeof(tmp_seq));
        task_num_cal = 0;
        tmp_seq[0] = 0;
        while (fscanf(fp, "%d,", &node1))
        {
            if (node1 == -1)
            {
                break;
            }
            if (node1 == 0)
            {
                tmp_seq[0]++;
                tmp_seq[tmp_seq[0]] = 0;
                continue;
            }
            fscanf(fp, "%d,", &node2);
            task_id = FindTask(node1, node2, inst_tasks, NO_task);
            if (task_id == 0)
            {
                task_id = -1;
                continue;
            }else{
                task_num_cal++;
            }
            tmp_seq[0]++;
            tmp_seq[tmp_seq[0]] = task_id;
        }
        // remove redundant 0 in the sequence.
        memset(tmp_sol.Sequence, 0, sizeof(tmp_sol.Sequence));
        for (j=1; j<=tmp_seq[0]; j++)
        {
            if (tmp_seq[j]==0 && tmp_seq[j-1]==0) continue;
            tmp_sol.Sequence[0]++;
            tmp_sol.Sequence[tmp_sol.Sequence[0]] = tmp_seq[j];
        }

        memcpy(tmp_seq, tmp_sol.Sequence, sizeof(tmp_sol.Sequence));

        // ps + insertion: not good
        // this->ps_and_insert(i, tmp_seq, inst_tasks);
        // printf("ps and insert cost:%d\n", this->psisols[i].TotalCost);

        // ps repair solutions
        this->repair(i, tmp_seq, inst_tasks);
        // printf("ps repair cost:%d\n", this->pssols[i].TotalCost);

        // deal with the outside vehicles
        // repair tmp_sol_seq;
//        this->insertion(&tmp_sol, inst_tasks);
//        this->repair_capacity_constrain(&tmp_sol, inst_tasks);
        this->feasible_insert(&tmp_sol, inst_tasks);
        copy_individual(&this->rsols[i], &tmp_sol);
    }
    fclose(fp);
    this->old_num = data_num;

    this->selection(inst_tasks);
}

void insert_new_tasks::repair(int sol_idx, int *sol_seq, const Task *inst_tasks)
{
    building_block seg[400];
    memset(seg->seqs, 0, sizeof(int)*100);
    int i,j;
    int seg_num = old_solution_block(sol_seq, inst_tasks, FULL, seg);

    building_block_path_scanning(&this->pssols[sol_idx], seg, seg_num, FULL, inst_tasks);
}

void insert_new_tasks::feasible_insert(CARPInd *sol, const Task *inst_tasks)
{
    int i, j, r;

    // add ov into new tasks list and assign them first.
    int new_tasks[MAX_TASK_SEQ_LENGTH];
    memset(new_tasks, 0, sizeof(new_tasks));
    // for (i=inst_tasks[1].inverse-1; ;i--)
    // {
    //     if(inst_tasks[i].vt == 0) break;
    //     new_tasks[0]++;
    //     new_tasks[new_tasks[0]] = i;
    // }
    for (i=1; i<inst_tasks[1].inverse; i++)
    {
        if (inst_tasks[i].vt == 0) continue;
        new_tasks[0]++;
        new_tasks[new_tasks[0]] = i;
    }
    for (i=1; i<=new_tasks_list[0]; i++)
    {
        new_tasks[0]++;
        new_tasks[new_tasks[0]] = new_tasks_list[i];
    }

    lns_route curr_solution;
    int Positions[101];
    find_ele_positions(Positions, sol->Sequence, 0);
    memset(curr_solution.loads, 0, sizeof(curr_solution.loads));
    curr_solution.Route[0][0] = Positions[0] - 1;
    curr_solution.loads[0] = Positions[0] - 1;
    for (i = 1; i < Positions[0]; i++)
    {
        AssignSubArray(sol->Sequence, Positions[i], Positions[i+1], curr_solution.Route[i]); // Route[i]: 0 x x x x 0
        curr_solution.loads[i] = 0;
        for (j = Positions[i]+1; j < Positions[i+1]; j++)
        {
            curr_solution.loads[i] += inst_tasks[sol->Sequence[j]].demand;
        }
    }
    curr_solution.total_cost = sol->TotalCost;
    int total_num_route = curr_solution.Route[0][0];

    int total_loads = 0;
    for (i=1; i<=total_num_route; i++)
    {
        total_loads+=curr_solution.loads[i];
    }
    for (i=1; i<=new_tasks[0]; i++)
    {
        total_loads += inst_tasks[new_tasks[i]].demand;
    }
    int vk = total_loads/capacity + 1;

    while(total_num_route<vk)
    {
        total_num_route++;
        curr_solution.Route[0][0]++;
        curr_solution.Route[total_num_route][0] = 2;
        curr_solution.Route[total_num_route][1] = 0;
        curr_solution.Route[total_num_route][2] = 0;
        curr_solution.loads[0]++;
        curr_solution.loads[total_num_route] = 0;
    }

    int increase_cost = 0;
    int insert_task, insert_task_idx, insert_min_cost, insert_task_route, insert_task_pos, tk, tmp_cost;
    int u, v;
    while(new_tasks[0] > 0) // greedy insertion
    {
        // find the insertion with the smallest increasing cost.
        insert_min_cost=INF;
        for (i=1; i<=new_tasks[0]; i++)
        {
            tk = new_tasks[i];
            for (r=1; r<=total_num_route; r++)
            {
                if(inst_tasks[tk].demand + curr_solution.loads[r] > capacity) continue;
                for (j=1; j<curr_solution.Route[r][0]; j++)
                {
                    u = curr_solution.Route[r][j];
                    v = curr_solution.Route[r][j+1];
                    tmp_cost = -1 * min_cost[inst_tasks[u].tail_node][inst_tasks[v].head_node];
                    tmp_cost += 1 * min_cost[inst_tasks[u].tail_node][inst_tasks[tk].head_node];
                    tmp_cost += 1 * min_cost[inst_tasks[tk].tail_node][inst_tasks[v].head_node];

                    if (tmp_cost < insert_min_cost)
                    {
                        insert_min_cost = tmp_cost;
                        insert_task_idx = i;
                        insert_task_pos = j;
                        insert_task_route = r;
                    }

                    tmp_cost = -1 * min_cost[inst_tasks[u].tail_node][inst_tasks[v].head_node];
                    tmp_cost += 1 * min_cost[inst_tasks[u].tail_node][inst_tasks[tk].tail_node];
                    tmp_cost += 1 * min_cost[inst_tasks[tk].head_node][inst_tasks[v].head_node];
                    if (tmp_cost < insert_min_cost)
                    {
                        insert_min_cost = tmp_cost;
                        insert_task_idx = -1*i;
                        insert_task_pos = j;
                        insert_task_route = r;
                    }
                }
            }
        }
        if (insert_min_cost==INF)
        {
            // printf("It must have some problems when do greedy insertion.\n");
            total_num_route++;
            curr_solution.Route[0][0]++;
            curr_solution.Route[total_num_route][0] = 2;
            curr_solution.Route[total_num_route][1] = 0;
            curr_solution.Route[total_num_route][2] = 0;
            curr_solution.loads[0]++;
            curr_solution.loads[total_num_route] = 0;
            continue;
            // exit(0);
        }
        // insert task to the sequence
        if (insert_task_idx > 0) {
            insert_task = new_tasks[insert_task_idx];
        } else if (insert_task_idx < 0) {
            insert_task_idx *= -1;
            insert_task = inst_tasks[new_tasks[insert_task_idx]].inverse;
        }
        add_element(curr_solution.Route[insert_task_route], insert_task, insert_task_pos + 1);
        curr_solution.loads[insert_task_route] += inst_tasks[insert_task].demand;
        delete_element(new_tasks, insert_task_idx);
    }
    

    // route -> sequence
    CARPInd mted_child;
    mted_child.Sequence[0] = 1;
    int k = 0;
    for (i = 1; i <= curr_solution.Route[0][0]; i++)
    {
        if (curr_solution.Route[i][0] > 2)
        {
            mted_child.Sequence[0] --;
            JoinArray(mted_child.Sequence, curr_solution.Route[i]);
            k ++;
            mted_child.Loads[k] = curr_solution.loads[i];
            if (curr_solution.loads[i] > capacity)
            {
                printf("the solution is not feasible. It must have bugs. \n");
                exit(0);
            }
        }
    }
    mted_child.Loads[0] = k;
    mted_child.TotalCost = get_task_seq_total_cost(mted_child.Sequence, inst_tasks);
    mted_child.TotalVioLoad = 0;

    copy_individual(sol, &mted_child);
}



void insert_new_tasks::selection(const Task *inst_tasks)
{
    int i, j;
    

    CARPInd allsols[this->old_num*2+1];
//    int from[2*this->old_num+1];
//    memset(from, 0, sizeof(from));

    int ps_sol_num = 0, r_sol_num = 0;
//    copy_individual(&this->sols[0], &this->pssols[0]);
//    from[0] = 1;

    // rank each population
    CARPInd tmp_indi;
    for (i=0; i<this->old_num; i++)
    {
        for (j=i+1; j<this->old_num; j++)
        {
            // printf("%d, %d\n", i, j);
            if (this->pssols[i].TotalCost > this->pssols[j].TotalCost)
            {
                tmp_indi = this->pssols[j];
                this->pssols[j] = this->pssols[i];
                this->pssols[i] = tmp_indi;
            }

            if (this->rsols[i].TotalCost > this->rsols[j].TotalCost)
            {
                tmp_indi = this->rsols[j];
                this->rsols[j] = this->rsols[i];
                this->rsols[i] = tmp_indi;
            }
        }
    }


    int rsol = 0, psol = 0, sol_num = 0;
    if(this->rsols[0].TotalCost < this->pssols[0].TotalCost)
    {
        while (true)
        {
            if (rsol >= this->old_num && psol >= this->old_num) break;
            if (rsol < this->old_num){
                // printf("%d, %d\n", sol_num, rsol);
                copy_individual(&allsols[sol_num], &this->rsols[rsol]);
                rsol++;
                sol_num ++;
            }
            if (psol < this->old_num){
                // printf("%d, %d\n", sol_num, psol);
                copy_individual(&allsols[sol_num], &this->pssols[psol]);
                psol++;
                sol_num ++;
            }
            
        }
    } else {
        while (true)
        {
            if (rsol >= this->old_num && psol >= this->old_num) break;
            if (psol < this->old_num){
                // printf("%d, %d\n", sol_num, psol);
                copy_individual(&allsols[sol_num], &this->pssols[psol]);
                psol++;
                sol_num ++;
            }
            
            if (rsol < this->old_num){
                // printf("%d, %d\n", sol_num, rsol);
                copy_individual(&allsols[sol_num], &this->rsols[rsol]);
                rsol++;
                sol_num ++;
            }  
        }
    }

    sol_num = sol_num > 30 ? 30 : sol_num; 

    this->sols = new CARPInd[sol_num+1];
    for (i=0; i<sol_num; i++)
    {
        copy_individual(&this->sols[i], &allsols[i]);
    }
    this->sol_num = sol_num;
    int a =1;
}

insert_new_tasks::~insert_new_tasks() {
    delete[] this->rsols;
    delete[] this->pssols;
    delete[] this->sols;
//    free(this->psisols);
}
