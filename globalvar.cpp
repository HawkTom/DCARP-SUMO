#include "globalvar.h"



int req_arc_num = 0;
int req_edge_num; 
int actual_req_edge_num;
int nonreq_arc_num = 0;
int nonreq_edge_num;
int vertex_num;
int vehicle_num;
int capacity;
int task_max_cost;
int task_num;
int total_arc_num;
int DEPOT;
int trav_cost[MAX_NODE_TAG_LENGTH][MAX_NODE_TAG_LENGTH];
int serve_cost[MAX_NODE_TAG_LENGTH][MAX_NODE_TAG_LENGTH];
int min_cost[MAX_NODE_TAG_LENGTH][MAX_NODE_TAG_LENGTH];
int shortest_path[MAX_NODE_TAG_LENGTH][MAX_NODE_TAG_LENGTH][MAX_NODE_TAG_LENGTH];

int cost_backup[MAX_NODE_TAG_LENGTH][MAX_NODE_TAG_LENGTH];
char map[20];
int instance_id;


int costlb = INF;
int costub = 0;
int demandlb = INF;
int demandub = 0;

int edge_index[2];
int info_demand_change[300][3];
int info_cost_change[300][3];
int info_task_change[300][3];

int new_tasks_list[MAX_TASK_SEQ_LENGTH];

double add_task_pro;
double pro_cost_change; // for old simulator
double cost_change_ratio;
int file_name_sep;

int dym_t;
int run_num;
int scn_num;


// int cost_change_edges[300][2];
// std::set<std::string> cost_change_edges;