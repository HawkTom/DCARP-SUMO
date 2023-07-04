#ifndef _GLOBALVAR_H
#define _GLOBALVAR_H

#include <string>
#include <iostream>
#include <map>
#include <cstring>
#include <time.h>

#define MAX_TASK_SEQ_LENGTH 500
#define MAX_NODE_TAG_LENGTH 500

#define MAX_TASK_TAG_LENGTH 500
#define MAX_TASK_SEG_LENGTH 1000

#define INF 1000000000

#define NODE_NUM 2895
#define ARC_NUM 6633


extern int capacity;
extern int req_edge_num;
extern int DEPOT;
extern int DEPOT_R;
extern int task_num;

extern int scenario_idx;
extern int instance_idx;
extern int run_num;

extern int min_cost_r[NODE_NUM+1][NODE_NUM+1];
extern int min_cost[MAX_NODE_TAG_LENGTH][MAX_NODE_TAG_LENGTH+1];

extern char version[20];

// record the new tasks
extern int new_tasks_list[MAX_TASK_SEQ_LENGTH];

typedef struct task
{
    int head_node;
    int tail_node;
    int head_node_r;
    int tail_node_r;
    int dead_cost;
    int serv_cost;
    int demand;
    int inverse;
    int vt;
    int IDindex; // the index of road in tha map
} Task;

// format: head ----> tail
typedef struct arc
{
    int head_node_r;
    int tail_node_r;
    int head_node;
    int tail_node;
    int base_cost;
    int trav_cost;
    unsigned int addt;
} Arc;

typedef struct individual
{
    int Sequence[MAX_TASK_SEQ_LENGTH]; // task id with depot inside
    int Assignment[250]; // when the representation is chromosome,this is used as the chromosome.
    int TotalCost;
    int Loads[80];
    int TotalVioLoad;
    double Fitness;
//    int start[101]; // for DCARP
} CARPInd;

typedef struct vehicle_state
{
    int remain_seqs[MAX_TASK_SEQ_LENGTH];
    int stop[101];
    int remain_capacity[101];
    int not_served_task_seq[MAX_TASK_SEQ_LENGTH];
} Vehicles;


int MAENS(const Task *inst_tasks, CARPInd *MAENSolution, CARPInd InitSolution, const int dynamic_type);

#endif