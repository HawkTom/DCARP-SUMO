#include "src/src.h"
#include "libs/cmdline.h"

#include "globalvar.h"


#define CMDFLAG 1

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

void init(Task *inst_tasks, CARPInd *old_solution);
void save_solution_xml(const CARPInd new_solution, const Task *inst_tasks);

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
        inst_info.parse_check(argc, argv);

        scenario_idx = inst_info.get<int>("scenario");
        instance_idx = inst_info.get<int>("instance");
    } else {
        scenario_idx = 1;
        instance_idx = 3;
    }
    printf("load scenario and instance successfully. \n");


    Task inst_tasks[MAX_TASK_SEQ_LENGTH];
    CARPInd old_solution;
    init(inst_tasks, &old_solution);

    CARPInd ps_solution, maens_solution;
    
    int ServeMark[2*req_edge_num + 1];
    memset(ServeMark, 1, sizeof(ServeMark));
    path_scanning(&ps_solution, inst_tasks, ServeMark);

    // printf("ps_solution: %d\n", ps_solution.TotalCost);

    MAENS(inst_tasks, &maens_solution, ps_solution, 0); // 1 for dynamic, 0 for restart
    save_solution_xml(maens_solution, inst_tasks); 

    return 0;

}

// this function is for new tasks without re-optimization
void SimpleInsertion(const Task *inst_tasks, CARPInd *sol)
{

}


