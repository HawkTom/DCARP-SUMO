//
// generator: simulator 2.0
// remove the event: road broken down
// random choose outside vehicles and its stop position
//

#include "src.h"
#include <random>


#define UNSERVED 0
#define OUTSIDE 1
#define COMPLETE 2

void process_solution(CARPInd *Solution, const Task *inst_tasks_vt);
void executeSolution(CARPInd *Solution, int tau1, Vehicles *state, int (*serve_tasks)[MAX_NODE_TAG_LENGTH], const Task * inst_tasks_vt);
void dynamicChange(Task *inst_tasks, Arc *inst_arcs, const int (*serve_tasks)[MAX_NODE_TAG_LENGTH], Vehicles *info, unsigned int seed);
void get_unserved_task_endnodes(const Task *inst_tasks, int (*endnodes)[3], const Vehicles *state);
void generateNewInstance(Task *inst_tasks, Arc *inst_arcs, int (*endnodes)[3], Vehicles *state, unsigned int seed1);

typedef struct edge
{
    int head_node;
    int tail_node;
    int trav_cost;
    int serv_cost;
    int demand;
} Edge;



void nextScenario(CARPInd *Solution, Task *inst_tasks_vt, Task *inst_tasks, Arc *inst_arcs, Vehicles *state, unsigned int seed)
{
    int tau = 400;
    int serve_tasks[MAX_NODE_TAG_LENGTH][MAX_NODE_TAG_LENGTH];;
    memset(serve_tasks, 0, sizeof(serve_tasks));

    int unserved_task_endnodes[MAX_TASK_SEG_LENGTH][3];

    srand(seed);
    process_solution(Solution, inst_tasks_vt);
    init_vehicle_state(state);
    executeSolution(Solution, tau, state, serve_tasks, inst_tasks_vt); // inst_tasks_vt: old graph
    get_unserved_task_endnodes(inst_tasks_vt, unserved_task_endnodes, state);
    generateNewInstance(inst_tasks, inst_arcs, unserved_task_endnodes, state, seed);
}

void get_unserved_task_endnodes(const Task *inst_tasks, int (*endnodes)[3], const Vehicles *state)
{
    int i, head, tail;
    memset(endnodes, 0, sizeof(int)*2*MAX_TASK_SEG_LENGTH);
    for (i=1; i<=state->remain_seqs[0]; i++)
    {
        head = inst_tasks[state->remain_seqs[i]].head_node;
        tail = inst_tasks[state->remain_seqs[i]].tail_node;
        endnodes[0][0]++;
        endnodes[endnodes[0][0]][0] = head;
        endnodes[endnodes[0][0]][1] = tail;
        endnodes[endnodes[0][0]][2] = inst_tasks[state->remain_seqs[i]].demand;
    }
    endnodes[0][0]++;
    endnodes[endnodes[0][0]][0] = -1;
    endnodes[endnodes[0][0]][1] = -1;
    for (i=1; i<=state->not_served_task_seq[0]; i++)
    {
        head = inst_tasks[state->not_served_task_seq[i]].head_node;
        tail = inst_tasks[state->not_served_task_seq[i]].tail_node;
        endnodes[0][0]++;
        endnodes[endnodes[0][0]][0] = head;
        endnodes[endnodes[0][0]][1] = tail;
        endnodes[endnodes[0][0]][2] = inst_tasks[state->not_served_task_seq[i]].demand;
    }
}

void init_vehicle_state(Vehicles *state)
{
    memset(state->stop, 0, sizeof(state->stop));
    memset(state->remain_capacity, 0, sizeof(state->remain_capacity));
    memset(state->remain_seqs, 0, sizeof(state->remain_seqs));
    memset(state->not_served_task_seq, 0, sizeof(state->not_served_task_seq));
    memset(new_tasks_list, 0, sizeof(new_tasks_list));
}

void process_solution(CARPInd *Solution, const Task *inst_tasks_vt)
{
    int i, j;
    for (i =1; i < Solution->Sequence[0]; i++)
    {
        if (inst_tasks_vt[Solution->Sequence[i]].vt > 0 && Solution->Sequence[i-1] != 0)
        {
            add_element(Solution->Sequence, 0, i);
        }
    }
}

/*
 input: start point, solution,
 output: stop point, remain capacity;
 */
void executeSolution(CARPInd *Solution, int tau1, Vehicles *state, int (*serve_tasks)[MAX_NODE_TAG_LENGTH], const Task * inst_tasks_vt)
{
    int vehicle_num_tmp = INF;

    int i, j, k;

    for (i = 1; i <= Solution->Loads[0]; i++)
    {
        if (Solution->Loads[i] > capacity)
        {
            printf("Exceed the capacity %d\n", Solution->Loads[i]);
        }
    }

    int Route[101][MAX_TASK_SEQ_LENGTH];
    memset(Route, 0, sizeof(Route));

    int Positions[101];
    find_ele_positions(Positions, Solution->Sequence, 0);
    Route[0][0] = Positions[0]-1;
    int is_out_route[Positions[0]];
    memset(is_out_route, 0, sizeof(is_out_route));

    for (i=1; i < Positions[0]; i++)
    {
        AssignSubArray(Solution->Sequence, Positions[i], Positions[i+1], Route[i]);
        if (inst_tasks_vt[Solution->Sequence[Positions[i]+1]].vt > 0)
        {
            is_out_route[i] = 1;
        } else {
            is_out_route[i] = 0;
        }
    }


    int schedule[101][MAX_TASK_SEQ_LENGTH];
    memcpy(schedule, Route, sizeof(Route));


    int remain_load[101][100];
    memset(remain_load, 0, sizeof(remain_load));
    remain_load[0][0] = schedule[0][0];
    for (i = 1; i <= schedule[0][0]; i++)
    {
        
        if (is_out_route[i])
        {
            remain_load[i][0] = schedule[i][0]-1;
            // the remaining loads for outside vehicle is different
            remain_load[i][1] = capacity - inst_tasks_vt[schedule[i][2]].demand;
            delete_element(schedule[i], 2); // delete virtual task
        } else {
            remain_load[i][0] = schedule[i][0];
            remain_load[i][1] = capacity;
        }

        for(j = 2; j < schedule[i][0]; j++)
        {
            if (schedule[i][j] == 0)
            {
                remain_load[i][j] = capacity;
            } else {
                remain_load[i][j] = remain_load[i][j-1] - inst_tasks_vt[schedule[i][j]].demand;
            }
        }
    }
    // state->remain_seqs[0] = 1;
    // state->remain_seqs[1] = 0;

    // the code below is to randomly determine the stop state without considering the cost(distance)
    int tmp_tour[500];
    int stop, stop_task_index, start, end;
    double threshold_lb = capacity * 0.3;
    double threshold_ub = capacity * 0.6;

    // each vehicle has three different status: outside vehicles, served routes, unserved routes
    int vehicle_state[101];
    double p;
    vehicle_state[0] = schedule[0][0];
    int complete_route = rand_choose(schedule[0][0]);
    for (i = 1; i <= schedule[0][0]; i++)
    {
        if (schedule[i][0] <= 4 || i == complete_route)
        {
            vehicle_state[i] = COMPLETE;
            continue;
        }
        p = rand() / (float) RAND_MAX;
        if (p<0.4){
            vehicle_state[i] = UNSERVED;
        } else if (p >=0.4 && p <= 0.9) {
            vehicle_state[i] = OUTSIDE;
        }
    }

//    for (i=1; i<=schedule[0][0]; i++)
//    {
//        printf("state: %d. ", vehicle_state[i]);
//        for (j=1; j<=schedule[i][0]; j++)
//        {
//            printf("%d ", schedule[i][j]);
//        }
//        printf(("\n"));
//    }
    

    for (i = 1; i <= schedule[0][0]; i++)
    {
        if ( vehicle_state[i] == OUTSIDE )
        {
            int route_task_num = schedule[i][0];
            int stop_condition_avail[route_task_num];
            int stop_all_avial[route_task_num];
            memset(stop_condition_avail, 0, sizeof(stop_condition_avail));
            memset(stop_all_avial, 0, sizeof(stop_all_avial));

            int kk, stop_index;
            for (kk = 3; kk < remain_load[i][0]-1; kk++)
            {
                stop_all_avial[0]++;
                stop_all_avial[stop_all_avial[0]] = kk;
//                if (remain_load[i][kk] > threshold_lb && remain_load[i][kk] < threshold_ub)
//                {
//                    stop_condition_avail[0] ++;
//                    stop_condition_avail[stop_condition_avail[0]] = kk;
//                }
            }
            if (stop_condition_avail[0] == 0)
            {
                memcpy(stop_condition_avail, stop_all_avial, sizeof(stop_all_avial));
            }

            int random_select = stop_condition_avail[0]>8?8:stop_condition_avail[0];
            stop_task_index = stop_condition_avail[rand_choose(random_select)];

            if (stop_task_index == 0)
            {
                stop_task_index ++;
                printf("It has bugs here because stop_task_index can not be 0");
                exit(-1);
            }

            start = inst_tasks_vt[schedule[i][stop_task_index]].tail_node;
            end = inst_tasks_vt[schedule[i][stop_task_index+1]].head_node;
            AssignArray(shortest_path[start][end], tmp_tour);
            stop = tmp_tour[rand_choose(tmp_tour[0])];
            int trial=0;
            while (stop==1)
            {
                trial ++;
                stop = tmp_tour[rand_choose(tmp_tour[0])];
                if (trial > 10)
                {
                    printf("It can't select the stop node which is not the DEPOT. We need re-select the task");
                    exit(-1);
                    break;
                }
            }
            if (stop != 1)
            {
                state->stop[0] ++;
                state->stop[state->stop[0]] = stop;
                state->remain_capacity[0] ++;
                state->remain_capacity[state->remain_capacity[0]] = remain_load[i][stop_task_index];
            } else {
                printf("stop node is DEPOT. It must have bugs\n");
                exit(-1);
            }

            if (state->remain_capacity[state->remain_capacity[0]] == capacity)
            {
                printf("(Remain capacity will not equal to capacity anymore. It must have bugs here.)\n");
                exit(-1);
            }
            state->remain_seqs[0] ++;
            state->remain_seqs[state->remain_seqs[0]] = 0;
            for (j = stop_task_index+1; j < schedule[i][0]; j++)
            {
                state->remain_seqs[0] ++;
                state->remain_seqs[state->remain_seqs[0]] = schedule[i][j];
                if(schedule[i][j] == 0)
                {
                    printf("it has some bugs here");
                    int a = 1;
                }
            }

            // add served task into serve_tasks matrix
            for (j = 1; j <= stop_task_index; j++)
            {
                serve_tasks[inst_tasks_vt[schedule[i][j]].head_node][inst_tasks_vt[schedule[i][j]].tail_node] = 1;
                serve_tasks[inst_tasks_vt[schedule[i][j]].tail_node][inst_tasks_vt[schedule[i][j]].head_node] = 1;
            }
        }

        if ( vehicle_state[i] == UNSERVED )
        {
            for (j = 1; j < schedule[i][0]; j++)
            {
                state->not_served_task_seq[0] ++;
                state->not_served_task_seq[state->not_served_task_seq[0]] = schedule[i][j];
            }
        }

        if (vehicle_state[i] == COMPLETE)
        {
            for (j = 1; j <=  schedule[i][0]; j++)
            {
                serve_tasks[inst_tasks_vt[schedule[i][j]].head_node][inst_tasks_vt[schedule[i][j]].tail_node] = 1;
                serve_tasks[inst_tasks_vt[schedule[i][j]].tail_node][inst_tasks_vt[schedule[i][j]].head_node] = 1;
            }
        }
    }
    if (state->remain_seqs[0] > 0)
    {
        state->remain_seqs[0] ++;
        state->remain_seqs[state->remain_seqs[0]] = 0;
    }
    if (state->not_served_task_seq[0] > 0)
    {
        state->not_served_task_seq[0] ++;
        state->not_served_task_seq[state->not_served_task_seq[0]] = 0;
    }
    int a=1;
    int b=1;
}
/*
 input: inst_tasks, inst_arcs
 output: new_inst_tasks, new_inst_tasks, new_req_edge_num, new_nonreq_edge_num, new_vertex_num;
 */
void dynamicChange(Task *inst_tasks, Arc *inst_arcs, const int (*serve_tasks)[MAX_NODE_TAG_LENGTH], Vehicles *info, unsigned int seed1)
{

    int demand_change_flag = 0;
    int add_task_flag = 0;
    int cost_change_flag = 1;

    int seed = time(NULL);
    // printf("seed: %d\n", seed);
    srand(seed);

    memset(info_cost_change, 0, sizeof(info_cost_change));
    memset(info_task_change, 0, sizeof(info_task_change));
    memset(info_demand_change, 0, sizeof(info_demand_change));

    double p_cost, p_demand, p_task, cost_ratio;
    p_cost = 1; p_demand = 0; p_task = 0;  // related to cost

    pro_cost_change = 0.5;
    cost_change_ratio = 3;
    p_cost = pro_cost_change;
    cost_ratio = cost_change_ratio;

    int i, j;


    int link_adj[MAX_NODE_TAG_LENGTH][MAX_NODE_TAG_LENGTH];
    memset(link_adj, 0, sizeof(link_adj));

    // printf("route\n");
    int k = 0, start, end;
    for (i=1; i <= info->remain_seqs[0]; i++)
    {
        if (info->remain_seqs[i] == 0)
        {
            if (k <= info->stop[0])
            {
                k ++;
                start = info->stop[k];
            } else {
                start = DEPOT;
            }
            continue;
        }
        end = inst_tasks[info->remain_seqs[i]].head_node;
        // printf("start: %d, end: %d \n", start, end);
        for(j=1; j<shortest_path[start][end][0]; j++)
        {
            link_adj[shortest_path[start][end][j]][shortest_path[start][end][j+1]] ++;
            link_adj[shortest_path[start][end][j+1]][shortest_path[start][end][j]] ++;
        }
        start = inst_tasks[info->remain_seqs[i]].tail_node;
        if (info->remain_seqs[i+1] == 0)
        {
            end = DEPOT;
            // printf("start: %d, end: %d \n", start, end);
            for(j=1; j<shortest_path[start][end][0]; j++)
            {
                link_adj[shortest_path[start][end][j]][shortest_path[start][end][j+1]] ++;
                link_adj[shortest_path[start][end][j+1]][shortest_path[start][end][j]] ++;
            }
        }
    }



    
    Edge graph[MAX_TASKS_TAG_LENGTH];

    int AdMatrix[MAX_NODE_TAG_LENGTH][MAX_NODE_TAG_LENGTH];
    memset(AdMatrix, 0, sizeof(AdMatrix));

    req_edge_num = edge_index[0];
    nonreq_edge_num = edge_index[1];
    
    int head, tail;
    for (i=1; i < inst_tasks[i].inverse; i++)
    {
        head = inst_tasks[i].head_node;
        tail = inst_tasks[i].tail_node;
        graph[i].head_node = head;
        graph[i].tail_node = tail;
        if (serve_tasks[head][tail] || serve_tasks[tail][head])
        {
            graph[i].demand = 0;
        } else{
            graph[i].demand = inst_tasks[i].demand;
        }
        graph[i].serv_cost = inst_tasks[i].serv_cost;
        AdMatrix[head][tail] = i;
        AdMatrix[tail][head] = i;
    }

    int total_edge_num = i;
    for (i=1; i <= 2*(req_edge_num + nonreq_edge_num); i++)
    {
        head = inst_arcs[i].head_node;
        tail = inst_arcs[i].tail_node;
        if (AdMatrix[head][tail] > 0)
        {
            int idx = AdMatrix[head][tail];
            graph[idx].trav_cost = inst_arcs[i].trav_cost;
        } else{
            graph[total_edge_num].head_node = head;
            graph[total_edge_num].tail_node = tail;
            graph[total_edge_num].demand = 0;
            graph[total_edge_num].trav_cost = inst_arcs[i].trav_cost;
            AdMatrix[head][tail] = total_edge_num;
            AdMatrix[tail][head] = total_edge_num;
            total_edge_num++;
        }
    }
    total_edge_num --;

    // Demand change
    if (demand_change_flag)
    {
        int demand_change;
        for (i=1; i<= total_edge_num; i++)
        {   
            double rnum =  rand() / (float) RAND_MAX;
            if (graph[i].demand > 0 && rnum < p_demand) {
                // it is able to increase demand
                graph[i].demand += (int)(graph[i].demand*0.5);
                graph[i].serv_cost += (int)(graph[i].serv_cost*0.5);

                info_demand_change[0][0]++;
                info_demand_change[info_demand_change[0][0]][0] = graph[i].head_node;
                info_demand_change[info_demand_change[0][0]][1] = graph[i].tail_node;
                info_demand_change[info_demand_change[0][0]][2] = (int)(graph[i].demand*0.5);
            }
        }
    }

    // Add task
    if (add_task_flag)
    {
        for (i=1; i<= total_edge_num; i++)
        {   
            double rnum =  rand() / (float) RAND_MAX;
            if (graph[i].demand == 0 && rnum < p_task)
            {
                // it is able to add a task
                graph[i].demand = graph[i].trav_cost;
                graph[i].serv_cost = graph[i].trav_cost;

                info_task_change[0][0]++;
                info_task_change[info_task_change[0][0]][0] = graph[i].head_node;
                info_task_change[info_task_change[0][0]][1] = graph[i].tail_node;
                info_task_change[info_task_change[0][0]][2] = graph[i].demand;
            }
        }
    }

    // Cost change
    if (cost_change_flag)
    {
        int cost_change;
        for (i=1; i<= total_edge_num; i++)
        {   
            double rnum =  rand() / (float) RAND_MAX;
            // increase or decrease edge's costs
            if (rnum < p_cost)
            {
                // cost_change = ((rand()/(float) RAND_MAX) < 0.5?1:(-0.5)) * graph[i].trav_cost;
                // graph[i].trav_cost += (int)(cost_change);
                if (link_adj[graph[i].head_node][graph[i].tail_node] > 0)
                {
                    // in the solution's path
                    cost_change = graph[i].trav_cost * cost_ratio; // cost_ratio = {2, 4, 6, 8, 10}
                } else {
                    // not int the solution's path
                    cost_change = graph[i].trav_cost / cost_ratio;
                }

                info_cost_change[0][0]++;
                info_cost_change[info_cost_change[0][0]][0] = graph[i].head_node;
                info_cost_change[info_cost_change[0][0]][1] = graph[i].tail_node;
                info_cost_change[info_cost_change[0][0]][2] = (int)cost_change - graph[i].trav_cost;

                graph[i].trav_cost = (int)(cost_change);    
                // printf("%d %d \n", i, cost_change);
            }
        }
    }

    // Re-generate the inst_tasks and inst_arcs
    Task new_inst_tasks[MAX_TASKS_TAG_LENGTH];
    Arc new_inst_arcs[MAX_ARCS_TAG_LENGTH];

    req_edge_num = 0;
    nonreq_edge_num = 0;
    for (i=1; i<=total_edge_num; i++)
    {
        if (graph[i].demand > 0)
        {
            req_edge_num++;
            new_inst_tasks[req_edge_num].head_node = graph[i].head_node;
            new_inst_tasks[req_edge_num].tail_node = graph[i].tail_node;
            new_inst_tasks[req_edge_num].dead_cost = graph[i].trav_cost;
            new_inst_tasks[req_edge_num].serv_cost = graph[i].serv_cost;
            new_inst_tasks[req_edge_num].demand = graph[i].demand;   
        } else{
            nonreq_edge_num ++;
        }

        new_inst_arcs[i].head_node = graph[i].head_node;
        new_inst_arcs[i].tail_node = graph[i].tail_node;
        new_inst_arcs[i].trav_cost = graph[i].trav_cost;

        new_inst_arcs[i+total_edge_num].head_node = graph[i].tail_node;
        new_inst_arcs[i+total_edge_num].tail_node = graph[i].head_node;
        new_inst_arcs[i+total_edge_num].trav_cost = graph[i].trav_cost;
    }

    for (i=1; i<=req_edge_num; i++)
    {
        new_inst_tasks[i].inverse = req_edge_num+i;
        new_inst_tasks[i+req_edge_num].head_node = new_inst_tasks[i].tail_node;
        new_inst_tasks[i+req_edge_num].tail_node = new_inst_tasks[i].head_node;
        new_inst_tasks[i+req_edge_num].dead_cost = new_inst_tasks[i].dead_cost;
        new_inst_tasks[i+req_edge_num].serv_cost = new_inst_tasks[i].serv_cost;
        new_inst_tasks[i+req_edge_num].demand = new_inst_tasks[i].demand;
        new_inst_tasks[i+req_edge_num].inverse = i;
    }

    // adapt the task id in old solution to new instance.

    for (i = 1; i <= info->remain_seqs[0]; i++)
    {
        int old_task = info->remain_seqs[i];
        if (old_task != 0)
        {
            head = inst_tasks[old_task].head_node;
            tail = inst_tasks[old_task].tail_node;
            
            int tmp_head, tmp_tail;
            int flag = 0;
            for (j = 1; j<=req_edge_num; j++)
            {
                tmp_head = new_inst_tasks[j].head_node;
                tmp_tail = new_inst_tasks[j].tail_node;
                if (head == tmp_head && tail == tmp_tail)
                {
                    info->remain_seqs[i] = j;
                    flag = 1;
                    break;
                } else if (head == tmp_tail && tail == tmp_head)
                {
                    info->remain_seqs[i] = j+req_edge_num;
                    flag = 1;
                    break;
                }
            }
            if (flag == 0)
            {
                printf("Oops, there is no new corresponding task.\n");
                exit(0);
            }
        }
    }

    task_num = 2*req_edge_num;
    if (task_num != (new_inst_tasks[1].inverse-1)*2)
    {
        printf("Oops, task num is wrong.\n");
        exit(0);
    }
    total_arc_num = task_num + 2 * nonreq_edge_num + nonreq_arc_num;

    // output
    memcpy(inst_tasks, new_inst_tasks, sizeof(new_inst_tasks));
    memcpy(inst_arcs, new_inst_arcs, sizeof(new_inst_arcs));
    inst_tasks[0].head_node = DEPOT;
    inst_tasks[0].tail_node = DEPOT;
    inst_tasks[0].dead_cost = 0;
    inst_tasks[0].serv_cost = 0;
    inst_tasks[0].demand = 0;
    inst_tasks[0].inverse = 0;
    inst_arcs[0].head_node = DEPOT;
    inst_arcs[0].tail_node = DEPOT;
    inst_arcs[0].trav_cost = 0;
}


void generateNewInstance(Task *inst_tasks, Arc *inst_arcs, int (*endnodes)[3], Vehicles *state, unsigned int seed1)
{
    float cost_change_pro;
    cost_change_pro = 0.5;

    int cost_change_flag = 1;
    int add_task_flag = 1;
    int delete_task_flag = 1;

    std::default_random_engine e(seed1);
//    e.seed(seed1);
    int i,j;

    // cost change
    int edge_num1 = edge_index[0];
    int edge_num2 = edge_index[1];

    double rnum;
    std::uniform_real_distribution<double> u1(0.0, 1.0);
    double C;
    C = cost_change_ratio;
    std::uniform_real_distribution<double> r(1.0, C);

    info_cost_change[0][0] = 0;
    info_task_change[0][0] = 0;


    if (cost_change_flag) {
        for (i = 1; i <= edge_num1; i++) {
            rnum = u1(e);
            int old_cost = inst_arcs[i].trav_cost;
            if (rnum <= cost_change_pro) {
                inst_arcs[i].trav_cost = (int) (inst_arcs[i].base_cost * r(e));
                // inverse
                inst_arcs[i + edge_num1].trav_cost = inst_arcs[i].trav_cost;
            } else if (rnum > 0.75) {
                inst_arcs[i].trav_cost = inst_arcs[i].base_cost;
                inst_arcs[i + edge_num1].trav_cost = inst_arcs[i].trav_cost;
            }
            if (inst_arcs[i].trav_cost != old_cost) // The cost has changed
            {
                info_cost_change[0][0]++;
                info_cost_change[info_cost_change[0][0]][0] = inst_arcs[i].head_node;
                info_cost_change[info_cost_change[0][0]][1] = inst_arcs[i].tail_node;
                info_cost_change[info_cost_change[0][0]][2] = inst_arcs[i].trav_cost - old_cost;
            }
        }

        for (i = 2 * edge_num1 + 1; i <= 2 * edge_num1 + edge_num2; i++) {
            rnum = u1(e);
            if (rnum <= cost_change_pro) {
                inst_arcs[i].trav_cost = (int) (inst_arcs[i].base_cost * r(e));
                // inverse
                inst_arcs[i + edge_num2].trav_cost = inst_arcs[i].trav_cost;
            } else if (rnum > 0.75) {
                inst_arcs[i].trav_cost = inst_arcs[i].base_cost;
                inst_arcs[i + edge_num2].trav_cost = inst_arcs[i].trav_cost;
            }
        }
    }

    // task_change
    int head, tail;
    Task new_inst_tasks[MAX_TASKS_TAG_LENGTH];
    int new_req_edge_num = 0;
    int new_remain_seqs[MAX_TASK_SEQ_LENGTH];
    int new_not_served_task_seq[MAX_TASK_SEQ_LENGTH];
    memset(new_remain_seqs, 0, sizeof(new_remain_seqs));
    memset(new_not_served_task_seq, 0, sizeof(new_not_served_task_seq));

    int seq_flag = 1;
    int arc_is_task[MAX_ARCS_TAG_LENGTH];
    memset(arc_is_task, 0, sizeof(arc_is_task));

    // new task's part 1: un-served task
    for (i=1; i<=endnodes[0][0]; i++)
    {
        head = endnodes[i][0];
        tail = endnodes[i][1];
        if (head == DEPOT && tail == DEPOT)
        {
            // The task is depot
            if (seq_flag)
            {
                new_remain_seqs[0]++;
                new_remain_seqs[new_remain_seqs[0]] = 0;
            } else {
                new_not_served_task_seq[0]++;
                new_not_served_task_seq[new_not_served_task_seq[0]] = 0;
            }
            continue;
        }
        if (head == -1 && tail == -1)
        {
            // The following tasks come from the unserved_task_seqs.
            seq_flag = 0;
            continue;
        }
        for (j=1; j<=(edge_num1+edge_num2)*2; j++)
        {
            if (inst_arcs[j].head_node == head  && inst_arcs[j].tail_node == tail)
            {
                // this arc should be the task which is not un-served in the old solution
                // construct new inst_tasks;
                arc_is_task[j] = 1;
                break;
            }
        }

        // for deleting tasks
        if (add_task_flag && delete_task_flag && u1(e) < 0.2) // delete this task
        {
            info_task_change[0][0]++;
            info_task_change[info_task_change[0][0]][0] = inst_arcs[i].head_node * (-1);
            info_task_change[info_task_change[0][0]][1] = inst_arcs[i].tail_node * (-1);
            info_task_change[info_task_change[0][0]][2] = inst_arcs[i].base_cost * (-1);
            if (seq_flag)
            {
                new_remain_seqs[0]++;
                new_remain_seqs[new_remain_seqs[0]] = -1;
            } else {
                new_not_served_task_seq[0]++;
                new_not_served_task_seq[new_not_served_task_seq[0]] = -1;
            }
            continue;
        }

        new_req_edge_num++;
        new_inst_tasks[new_req_edge_num].head_node = head;
        new_inst_tasks[new_req_edge_num].tail_node = tail;
        new_inst_tasks[new_req_edge_num].serv_cost = inst_arcs[j].base_cost;
        new_inst_tasks[new_req_edge_num].dead_cost = inst_arcs[j].trav_cost;
        new_inst_tasks[new_req_edge_num].demand = endnodes[i][2];
        if (seq_flag)
        {
            new_remain_seqs[0]++;
            new_remain_seqs[new_remain_seqs[0]] = new_req_edge_num;
        } else {
            new_not_served_task_seq[0]++;
            new_not_served_task_seq[new_not_served_task_seq[0]] = new_req_edge_num;
        }
    }

    // new task's part 1: new task

    if (add_task_flag)
    {
        for (i=1; i<=edge_num1; i++)
        {
            if (inst_arcs[i].addt > 0)
            {
                printf("If the program runs here, it must have bugs. \n");
            }
        }
//        for (i = 2*edge_num1+1; i<=2*edge_num1+edge_num2; i++)
//        {
//            // version 1: add task with pre-defined settings
//            if (inst_arcs[i].addt == dym_t+1) //This function is used for generating (dym_t + 1)th instance.
//            {
//                new_req_edge_num++;
//                new_inst_tasks[new_req_edge_num].head_node = inst_arcs[i].head_node;
//                new_inst_tasks[new_req_edge_num].tail_node = inst_arcs[i].tail_node;
//                new_inst_tasks[new_req_edge_num].serv_cost = inst_arcs[i].base_cost;
//                new_inst_tasks[new_req_edge_num].dead_cost = inst_arcs[i].trav_cost;
//                new_inst_tasks[new_req_edge_num].demand = inst_arcs[i].base_cost;
//
//                info_task_change[0][0]++;
//                info_task_change[info_task_change[0][0]][0] = inst_arcs[i].head_node;
//                info_task_change[info_task_change[0][0]][1] = inst_arcs[i].tail_node;
//                info_task_change[info_task_change[0][0]][2] = inst_arcs[i].base_cost;
//            }
//            // version 2: add task with probability.
//            if (inst_arcs[i].addt > 0 && u1(e) < add_task_pro)
//            {
//                new_req_edge_num++;
//                new_inst_tasks[new_req_edge_num].head_node = inst_arcs[i].head_node;
//                new_inst_tasks[new_req_edge_num].tail_node = inst_arcs[i].tail_node;
//                new_inst_tasks[new_req_edge_num].serv_cost = inst_arcs[i].base_cost;
//                new_inst_tasks[new_req_edge_num].dead_cost = inst_arcs[i].trav_cost;
//                new_inst_tasks[new_req_edge_num].demand = inst_arcs[i].base_cost;
//
//                info_task_change[0][0]++;
//                info_task_change[info_task_change[0][0]][0] = inst_arcs[i].head_node;
//                info_task_change[info_task_change[0][0]][1] = inst_arcs[i].tail_node;
//                info_task_change[info_task_change[0][0]][2] = inst_arcs[i].base_cost;
//                inst_arcs[i].addt = 0;
//            }
//        }
        // version 3: add task in terms of the number of remaining tasks
        int add_task_num = (int)(new_req_edge_num * add_task_pro);
        int avail_arcs[200];
        memset(avail_arcs, 0, sizeof(avail_arcs));
        for (i = 2*edge_num1+1; i<=2*edge_num1+edge_num2; i++)
        {
            if (inst_arcs[i].addt > 0)
            {
                avail_arcs[0] ++;
                avail_arcs[avail_arcs[0]] = i;
            }
        }
        if (avail_arcs[0] == 0)
        {
            printf("no new task any more\n");
            exit(0);
        }

        if (avail_arcs[0] >= add_task_num)
        { // randomly select 'add_task_num' arcs as new tasks
            rand_shuffle(avail_arcs);
            avail_arcs[0] = add_task_num;
        } else { // select all remaining tasks as new tasks

        }
        for (i=1; i<=avail_arcs[0]; i++)
        {
            j = avail_arcs[i];
            new_req_edge_num++;
            new_inst_tasks[new_req_edge_num].head_node = inst_arcs[j].head_node;
            new_inst_tasks[new_req_edge_num].tail_node = inst_arcs[j].tail_node;
            new_inst_tasks[new_req_edge_num].serv_cost = inst_arcs[j].base_cost;
            new_inst_tasks[new_req_edge_num].dead_cost = inst_arcs[j].trav_cost;
            new_inst_tasks[new_req_edge_num].demand = inst_arcs[j].base_cost;

            info_task_change[0][0]++;
            info_task_change[info_task_change[0][0]][0] = inst_arcs[j].head_node;
            info_task_change[info_task_change[0][0]][1] = inst_arcs[j].tail_node;
            info_task_change[info_task_change[0][0]][2] = inst_arcs[j].base_cost;
            inst_arcs[j].addt = 0;
        }


        // we can also delete some existing tasks.
    }
    // make the new_inst_tasks complete
    for (i=1; i<=new_req_edge_num; i++)
    {
        new_inst_tasks[i+new_req_edge_num].head_node = new_inst_tasks[i].tail_node;
        new_inst_tasks[i+new_req_edge_num].tail_node = new_inst_tasks[i].head_node;
        new_inst_tasks[i+new_req_edge_num].serv_cost = new_inst_tasks[i].serv_cost;
        new_inst_tasks[i+new_req_edge_num].dead_cost = new_inst_tasks[i].dead_cost;
        new_inst_tasks[i+new_req_edge_num].demand = new_inst_tasks[i].demand;
        new_inst_tasks[i].inverse = i+new_req_edge_num;
        new_inst_tasks[i+new_req_edge_num].inverse = i;
    }

    memcpy(inst_tasks, new_inst_tasks, sizeof(Task)*MAX_TASKS_TAG_LENGTH);
    inst_tasks[0].head_node = DEPOT;
    inst_tasks[0].tail_node = DEPOT;
    inst_tasks[0].head_node_r = 0;
    inst_tasks[0].tail_node_r = 0;
    inst_tasks[0].dead_cost = 0;
    inst_tasks[0].serv_cost = 0;
    inst_tasks[0].demand = 0;
    inst_tasks[0].inverse = 0;

    memcpy(state->remain_seqs, new_remain_seqs, sizeof(int)*MAX_TASK_SEQ_LENGTH);
    memcpy(state->not_served_task_seq, new_not_served_task_seq, sizeof(int)*MAX_TASK_SEQ_LENGTH);

    req_edge_num = new_req_edge_num;
    nonreq_edge_num = edge_num1+edge_num2-req_edge_num;

    task_num = 2*req_edge_num;
    if (task_num != (new_inst_tasks[1].inverse-1)*2)
    {
        printf("Oops, task num is wrong.\n");
        exit(0);
    }
    total_arc_num = task_num + 2 * nonreq_edge_num + nonreq_arc_num;
}