//
// Created by hao on 13/06/2020.
//

#ifndef _TSA_H
#define _TSA_H

#include "../globalvar.h"
#include "../src/src.h"
#include <math.h>

int SingleInsertion(CARPInd *BestSINeighbor, CARPInd *CurrSolution, int *MovedTasks, int *ChangedRoutesID, const Task *inst_tasks, int (*TabuList)[101],
                    double PenPrmt, double BestFitness, double BestFsbFitness );
int DoubleInsertion(CARPInd *BestDINeighbor, CARPInd *CurrSolution, int *MovedTasks, int *ChangedRoutesID, const Task *inst_tasks, int (*TabuList)[101],
                    double PenPrmt, double BestFitness, double BestFsbFitness);
int SWAP(CARPInd *BestSWAPNeighbor, CARPInd *CurrSolution, int *MovedTasks, int *ChangedRoutesID, const Task *inst_tasks, int (*TabuList)[101],
                    double PenPrmt, double BestFitness, double BestFsbFitness);
void RepairInfeasibility(CARPInd *Indi, const Task *inst_tasks);
void FredericksonHeuristic(int *FHRoute, int *Route, const Task *inst_tasks);

int check_cost(CARPInd solution, const Task *inst_tasks);
void check_solution_valid(CARPInd solution, const Task *inst_task);

#endif //_TSA_H
