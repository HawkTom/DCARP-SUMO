#include "src/src.h"
#include "libs/cmdline.h"

#include "globalvar.h"


#define CMDFLAG 0

int capacity;
int req_edge_num;
int DEPOT;
int DEPOT_R;
int task_num;

int scenario_idx;
int instance_idx;
int run_num;

int min_cost_r[NODE_NUM+1][NODE_NUM+1];
int min_cost[MAX_NODE_TAG_LENGTH][MAX_NODE_TAG_LENGTH+1];
int new_tasks_list[MAX_TASK_SEQ_LENGTH];

char version[20];

void init(Task *inst_tasks, CARPInd *old_solution);
void save_solution_xml(const CARPInd new_solution, const Task *inst_tasks);
void SimpleInsertion(const Task *inst_tasks, CARPInd old_solution, CARPInd *new_solution);

// write a main function for hello world
int main(int argc, char *argv[])
{
    // init();
    struct tm *tm_now;
    time_t now;
    now = time(NULL);
    tm_now = localtime(&now);
    printf("start datetime: %d-%d-%d %d:%d:%d\n",tm_now->tm_year+1900, tm_now->tm_mon+1, tm_now->tm_mday, tm_now->tm_hour, tm_now->tm_min, tm_now->tm_sec) ;
    int seed = (int)rand() + tm_now->tm_year + tm_now->tm_mon + tm_now->tm_mday + tm_now->tm_hour + tm_now->tm_min + tm_now->tm_sec;
    seed = 222;
    srand(seed);
    printf("seed: %d\n", seed);

    if (CMDFLAG)
    {
        cmdline::parser inst_info;
        inst_info.add<int>("scenario", 's', "scenario index", true, 0, cmdline::range(0, 65535));
        inst_info.add<int>("instance", 'i', "instance index", true, 0, cmdline::range(0, 65535));
        inst_info.add<std::string>("version", 'v', "optimization version", true, "");

        inst_info.parse_check(argc, argv);
        scenario_idx = inst_info.get<int>("scenario");
        instance_idx = inst_info.get<int>("instance");
        strcpy(version, inst_info.get<std::string>("version").c_str());
    } else {
        scenario_idx = 4;
        instance_idx = 1;
        strcpy(version, "dynamic");
    }
    printf("load scenario, instance, optimization version successfully. \n");


    Task inst_tasks[MAX_TASK_SEQ_LENGTH];
    CARPInd old_solution;
    init(inst_tasks, &old_solution);

    CARPInd ps_solution, new_solution;
    
    int ServeMark[2*req_edge_num + 1];
    memset(ServeMark, 1, sizeof(ServeMark));
    path_scanning(&ps_solution, inst_tasks, ServeMark);
    printf("ps_solution: %d\n", ps_solution.TotalCost);

    if (strcmp(version, "static") == 0)
    {
        SimpleInsertion(inst_tasks, old_solution, &new_solution);
    }

    if (strcmp(version, "dynamic") == 0)
    {
        MAENS(inst_tasks, &new_solution, ps_solution, 0); // 1 for dynamic, 0 for restart
    }

    printf("new_solution: %d\n", new_solution.TotalCost);
    save_solution_xml(new_solution, inst_tasks);

    return 0;

}

// this function is for new tasks without re-optimization
// insert new tasks into the old solution greedy one by one
void SimpleInsertion(const Task *inst_tasks, CARPInd old_solution, CARPInd *new_solution)
{
    int i,j,k;

    // sort new tasks according to costs to depot: far to close
    int cost[MAX_TASK_SEQ_LENGTH];
    memset(cost, 0, sizeof(cost));
    cost[0] = new_tasks_list[0];
    int c1, c2;
    for (i=1; i<=new_tasks_list[0]; i++)
    {
        c1 = min_cost[DEPOT][inst_tasks[new_tasks_list[i]].head_node];
        c2 = min_cost[DEPOT][inst_tasks[new_tasks_list[i]].tail_node];
        cost[i] = (int)((c1+c2)/2);
    }
    for (i=1; i<cost[0]; i++)
    {
        for(j=i+1; j<=cost[0];j++)
        {
            if (cost[i] < cost[j])
            {
                int tmp = cost[j];
                cost[j] = cost[i];
                cost[i] = tmp;

                tmp = new_tasks_list[j];
                new_tasks_list[j] = new_tasks_list[i];
                new_tasks_list[i] = tmp;
            }
        }
    }

    lns_route curr_solution;
    int Positions[101];
    find_ele_positions(Positions, old_solution.Sequence, 0);
    memset(curr_solution.loads, 0, sizeof(curr_solution.loads));
    curr_solution.Route[0][0] = Positions[0] - 1;
    curr_solution.loads[0] = Positions[0] - 1;
    for (i = 1; i < Positions[0]; i++)
    {
        AssignSubArray(old_solution.Sequence, Positions[i], Positions[i+1], curr_solution.Route[i]); // Route[i]: 0 x x x x 0
        curr_solution.loads[i] = 0;
        for (j = Positions[i]+1; j < Positions[i+1]; j++)
        {
            curr_solution.loads[i] += inst_tasks[old_solution.Sequence[j]].demand;
        }
    }
    curr_solution.total_cost = old_solution.TotalCost;
    int total_num_route = curr_solution.Route[0][0];

    for (i=1; i<=new_tasks_list[0]; i++)
    {   
        int new_task = new_tasks_list[i];
        int new_task_demand = inst_tasks[new_task].demand;
        int insert_cost, min_insert_cost=INF;
        int min_insert_pos[2];
        for (j=1; j<=total_num_route; j++)
        {
            if (curr_solution.loads[j] + new_task_demand > capacity) continue;

            for (k=1; k<curr_solution.Route[j][0]; k++)
            {
                if (k==1 && inst_tasks[curr_solution.Route[j][k+1]].vt > 0) continue;

                insert_cost = -1*min_cost[DEPOT][inst_tasks[curr_solution.Route[j][2]].head_node];
                insert_cost += min_cost[DEPOT][inst_tasks[new_task].head_node];
                insert_cost += min_cost[inst_tasks[new_task].tail_node][inst_tasks[curr_solution.Route[j][2]].head_node];

                if (insert_cost < min_insert_cost)
                {
                    min_insert_cost = insert_cost;
                    min_insert_pos[0] = j;
                    min_insert_pos[1] = k;
                }

                insert_cost = -1*min_cost[DEPOT][inst_tasks[curr_solution.Route[j][2]].head_node];
                insert_cost += min_cost[DEPOT][inst_tasks[new_task].tail_node];
                insert_cost += min_cost[inst_tasks[new_task].head_node][inst_tasks[curr_solution.Route[j][2]].head_node];
                if (insert_cost < min_insert_cost)
                {
                    min_insert_cost = insert_cost;
                    min_insert_pos[0] = j;
                    min_insert_pos[1] = -1*k;
                }
            }
        }
        if (min_insert_cost < INF)
        {
            if (min_insert_pos[1] < 0)
            {
                new_task = inst_tasks[new_task].inverse;
                min_insert_pos[1] *= -1;
            }
            add_element(curr_solution.Route[min_insert_pos[0]], new_task, min_insert_pos[1]+1);
            curr_solution.loads[min_insert_pos[0]] += new_task_demand;
        } else{ // no feasible route avaliable
            curr_solution.Route[0][0]++;
            total_num_route ++;
            curr_solution.Route[total_num_route][0] = 3;
            curr_solution.Route[total_num_route][1] = 0;
            curr_solution.Route[total_num_route][2] = new_task;
            curr_solution.Route[total_num_route][3] = 0;
            curr_solution.loads[0] ++;
            curr_solution.loads[curr_solution.loads[0]] = new_task_demand;
        }
    }

    // route -> sequence
    new_solution->Sequence[0] = 1;
    k = 0;
    for (i = 1; i <= curr_solution.Route[0][0]; i++)
    {
        if (curr_solution.Route[i][0] > 2)
        {
            new_solution->Sequence[0] --;
            JoinArray(new_solution->Sequence, curr_solution.Route[i]);
            k ++;
            new_solution->Loads[k] = curr_solution.loads[i];
            if (curr_solution.loads[i] > capacity)
            {
                printf("the solution is not feasible. It must have bugs. \n");
                exit(0);
            }
        }
    }
    new_solution->Loads[0] = k;
    new_solution->TotalCost = get_task_seq_total_cost(new_solution->Sequence, inst_tasks);
    new_solution->TotalVioLoad = 0;
}


