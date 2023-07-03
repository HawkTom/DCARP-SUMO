//
// Created by hao on 15/06/2020.
//

#ifndef _MAENS_H
#define _MAENS_H


#define MAX_POPSIZE 30
#define MAX_TOTALSIZE 210
#define MAX_NONDOMINATED_NUM 1000

#define M_trial 10
#define M_PROB 0.2
#define M_ite 100
#define M_wite 100
#define SI 1
#define DI 2
#define SWAP 3

#define MAX_NSIZE 10 // upper bound of nsize
#define MAX_ENSSIZE 100 // maximal ENS neighborhood size


#include "../globalvar.h"
#include "../src/src.h"


typedef struct move
{
    int type;
    int task1;
    int task2;
    int orig_seg;
    int targ_seg;
    int orig_pos;
    int targ_pos;
    int total_cost;
    int total_vio_load;
    double fitness;
} Move;

void rand_scanning(CARPInd *rs_indi, const Task *inst_tasks, const int *serve_mark);
void indi_copy(CARPInd *target, CARPInd *source);
void SBX(CARPInd *xed_child, CARPInd *p1, CARPInd *p2, const Task *inst_tasks);
void lns_mut(CARPInd *c, CARPInd *p, CARPInd *best_fsb_solution, const Task *inst_tasks);

void lns(CARPInd *indi, double coef, int nsize, const Task *inst_tasks);

void single_insertion(Move *best_move, CARPInd *indi, double coef, const Task *inst_tasks);
void double_insertion(Move *best_move, CARPInd *indi, double coef, const Task *inst_tasks);
void swap(Move *best_move, CARPInd *indi, double coef, const Task *inst_tasks);

void MAENS_main_loop(int popsize, CARPInd *best_fsb_solution, CARPInd *pop, const Task *inst_tasks, const int dynamic_type);
int generate_rand_solution(int tmp_popsize, CARPInd *init_indi, CARPInd *pop, const Task *inst_tasks);

#endif //CARP_MAENS_H
