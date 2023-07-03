#ifndef  _SRC_H
#define _SRC_H

#include "../globalvar.h"
#include <fstream>

// function for read map data
// void readMap(Task *inst_tasks, Arc *inst_arcs, const char *map1);
void readMap(Task *inst_tasks, Arc *inst_arcs, const char *map1, CARPInd *sol_seq);
void save_dcarp_instance(const Task *inst_tasks, const Arc *inst_arcs, Vehicles state, int instance_idx);
void read_instance_from_xml(Task *inst_tasks, Arc *inst_arcs, Vehicles *state);
void save_dcarp_best(const CARPInd best_ind, const int add_cost, const int instance_idx);
void read_dynamic_map(Task *inst_tasks, Arc *inst_arcs, const char *map1, CARPInd *sol_seq);

// calculate cost of map
void mod_dijkstra();
void update_cost(const Task *inst_tasks, const Arc *inst_arcs);
int get_additional_cost(Vehicles state);
int get_task_seq_total_cost(int *task_seq, const Task *inst_tasks);
void get_cost_scale(const Task *inst_tasks);
void get_task_seq_loads(int *loads, const int *task_seq, const Task *inst_tasks);
int get_route_loads(const int *task_seq, const Task *inst_tasks);


// heuristic algorithms
void path_scanning(CARPInd *ps_indi, const Task *inst_tasks, const int *serve_mark);
void FredericksonHeuristic(int *FHRoute, int *Route, const Task *inst_tasks);
int split(int *split_task_seq, int *one_task_seq, int *split_route_loads, const Task *inst_tasks);
void augment_merge(CARPInd *am_indi, const Task *inst_tasks);
void rand_scanning(CARPInd *rs_indi, const Task *inst_tasks);
// void RepairInfeasibility (CARPInd *Indi, const Task *inst_tasks);


// module for processing virtual task related solution/individual
void indi_route_converter(CARPInd *dst, CARPInd *src, const Task *inst_tasks);
void construct_virtual_task(const Task *inst_tasks, Task *tasks_vt, const int *stop, const int *remain_capacity);
void inher_solution(CARPInd *inhrSolution, Vehicles state, const Task *inst_tasks_vt);
int repair_solution_greedy_insertion(CARPInd *solution, int *remain_seq, const int *stop, const Task *inst_tasks_vt);
void remain_solution(CARPInd *inhrSolution, Vehicles state, const Task *inst_tasks_vt);

// array operations functions
void delete_element(int *a, int k);
void find_ele_positions(int *positions, int *a, int e);
void add_element(int *a, int e, int k);
int rand_choose(int num);
void rand_perm(int *a, int num);
void rand_shuffle(int *a);
int max(int *Array);
void rand_selection(int *id1, int *id2, int popsize);
void AssignArray(int *Array1, int *Array2);
void AssignSubArray(int *Array1, int k1, int k2, int *Array2);
void JoinArray(int *JointArray, int *Array);
void ReverseDirection(int *Array, int k1, int k2);
int find_min(int *Array);


// simulator to generate the new DCARP instance
void nextScenario(CARPInd *Solution, Task *inst_tasks_vt, Task *inst_tasks, Arc *inst_arcs, Vehicles *state, unsigned int seed);
void init_vehicle_state(Vehicles *state);

// other 'solution'/'individual' related functions
void clear_solution(CARPInd *solution);
void copy_individual(CARPInd *dest, CARPInd *src);
void check_solution_valid(CARPInd solution, const Task *inst_task);
void check_seq_valid(CARPInd solution, const Task *inst_task);
int check_task_valid(int *seq);
int check_cost(CARPInd solution, const Task *inst_tasks);
int FindTask(int a, int b, const Task *inst_tasks, int NO_Task);
int get_total_vio_load(int *route_seg_load);
void saveResult(char *algorithm, int best);
void saveERT(char *algorithm, int state, double tpoint, int best);


void vt_demand_change(Task *inst_tasks, int flag);

typedef struct lns_route{
    int Route[101][MAX_TASK_SEQ_LENGTH];
    int total_cost;
    int loads[101];
    int total_vio_loads;
    double fitness;
} lns_route;

class experience
{
    public:
        int best_so_far[30][400];
        int queue_index = 0;
        int next = 0;
        int save_cost[30];
        int costScale, LB;
        experience();
        int save_best_so_far(CARPInd sol);
        void save_best_so_far_queue(CARPInd sol);
        void move_best_to_first();
        void save_solutions(const Task *inst_tasks);
        void load_old_sol(const Task *inst_tasks);
//        void process_train_data(const Task *inst_tasks);
};

void solution_to_state_data(const int *sol_seq, const Task *inst_tasks, int *data_num, int (*data)[5]);
void process_and_save_data(CARPInd indi, const Task *inst_tasks);



struct building_block{
    int seqs[100];
    int head;
    int tail;
    int dep_dist;
    int serv_costs;
    double yield;
    int inverse;
    int loads;
};
int old_solution_block(int *sol_seq, const Task *inst_tasks, int type, building_block *seg);
void building_block_path_scanning(CARPInd *ps_sol, building_block *seg, int seg_num, int type, const Task *inst_tasks);
int inherited_cluster(building_block *seg, const int *split_seqs, const Task *inst_tasks);

class repair_old_solutions
{
public:
    explicit repair_old_solutions(const Task *inst_tasks, CARPInd old_sol);
    ~repair_old_solutions();
    int sol_num;
//    CARPInd sols[30];
    CARPInd *sols;
    CARPInd inherited_sol;
    CARPInd cps_sol;
    // void get_inherited_solution(Vehicles state, const Task *inst_tasks);
    // void get_cps_solution(const Task *inst_tasks);
    void get_inherited_solution(CARPInd old_sol);
    void repair(CARPInd *tmp_sol, int *sol_seq, const Task *inst_tasks);
};


typedef struct roptmove{
    int break1;
    int break2;
    int link;
    int route1;
    int route2;
}roptmove;


class insert_new_tasks{
public:
    explicit insert_new_tasks(const Task *inst_tasks);
    ~insert_new_tasks();
    int old_num;
    int sol_num;
    int from[70]; // 1: ps; 2: ropt
    CARPInd *sols;
    CARPInd *rsols;
    CARPInd *pssols;
    CARPInd *psisols;
    void ps_and_insert(int sol_idx, int *sol_seq, const Task *inst_tasks);
    void feasible_insert(CARPInd *sol, const Task *inst_tasks);
    void insertion(CARPInd *sol, const Task *inst_tasks);
    void repair_capacity_constrain(CARPInd *sol, const Task *inst_tasks);
    void repair(int sol_idx, int *sol_seq, const Task *inst_tasks);
    void selection(const Task *inst_tasks);
};

class record{
public:
    std::fstream stream;
    double tpoint=0;
    explicit record(const char *path);
    explicit record(int type);
    explicit record();
    ~record();
    void createfile(int type);
    void parse_data(double t, int cost, int line_num);

};

//class one_class_model
//{
//    public:
//        svm_model *model;
//        void construct(const Task *inst_tasks);
//
//        void train_model();
//        void load_model();
//        void predict_file_data();
//        double predict_one(const double *attr, double *probab);
//        explicit one_class_model(int from_type);
//        ~one_class_model();
//};
//
//class knn_model
//{
//public:
//    struct data_node{
//        double x[2];
//        double y[3];
//    };
//    struct route_node{
//
//    };
//    data_node *data;
//    int data_num;
//    void construct(const Task *inst_tasks, CARPInd *sol);
//    void find_the_nearest_state(int k, int *index, double att1, double att2);
//    double calculate_the_similarity(const int *index, int k, const double *atts) const;
//    explicit knn_model();
//    ~knn_model();
//};


class rule_extraction{
public:
    explicit rule_extraction();
    ~rule_extraction();

    void extract_rule(CARPInd sol, const Task *inst_tasks);
};

#endif