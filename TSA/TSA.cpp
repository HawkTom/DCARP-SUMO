//
// Created by hao on 13/06/2020.
//
#include "TSA.h"

#define RECORD 0
#define STATIC_MAX_TIME 10
#define DYNAMIC_MAX_TIME 60
#define DYNMAIC 1
#define RESTART 0
#define SAVE 1
#define LOG 0

void TSA(const Task *inst_tasks, CARPInd *TSASolution, CARPInd InitSolution, const int dynamic_type)
{
    int i, j, k, m;

    int NVer, NRE, NRA, NNR, NVeh, Cap, LLB;
    NVer = vertex_num;
    NRE = req_edge_num;
    NRA = req_arc_num;
    NNR = 2*nonreq_edge_num+nonreq_arc_num;
    NVeh = vehicle_num;
    Cap = capacity;

    if (dynamic_type == DYNMAIC)
    {
        int best_idx=0, best_cost = INF;

//        repair_old_solutions exp(inst_tasks, InitSolution);
        insert_new_tasks exp(inst_tasks);
        for (i = 0; i < exp.sol_num; i++)
        {
            if (best_cost > exp.sols[i].TotalCost)
            {
                best_cost = exp.sols[i].TotalCost;
                best_idx = i;
            }
        }
        copy_individual(&InitSolution, &exp.sols[best_idx]);
    } else {
//         CARPInd InitSolution;
         int ServMark[2*NRE+NRA+1]; // the mark for served tasks
         memset(ServMark, 1, sizeof(ServMark));
         path_scanning(&InitSolution, inst_tasks, ServMark);
    }


    // CARPInd InitSolution;
    // int ServMark[2*NRE+NRA+1]; // the mark for served tasks
    // memset(ServMark, 1, sizeof(ServMark));
    // path_scanning(&InitSolution, inst_tasks, ServMark);

     InitSolution.Fitness = InitSolution.TotalCost;

    // printf("TSA: initial is ok \n");

    // following five variables are defined for Frederick Heuristic
    int Route[MAX_TASK_SEQ_LENGTH], FHRoute[MAX_TASK_SEQ_LENGTH], Position[101], TmpSeq[MAX_TASK_SEQ_LENGTH], Cost1, Cost2;

    TmpSeq[0] = 1; // tmp solution to link all processed sub-routes

    CARPInd CurrSolution, BestSolution, BestFsbSolution, NextSolution, TmpSolution;
    int TabuList[2*NRE+NRA+1][101];
    memset(TabuList, 0, sizeof(TabuList));
    int TabuTenure = (NRE + NRA) / 2;

    check_solution_valid(InitSolution, inst_tasks);

    CurrSolution = InitSolution;
    BestSolution = InitSolution;

    BestFsbSolution.TotalCost = 0;
    if (InitSolution.TotalVioLoad == 0)
    {
        BestFsbSolution = InitSolution;
    }

    double PenPrmt = 1; //penalty parameter
    int ConstFsb, ConstInFsb; // the number of iterations for feasible/unfeasible solution

    int FSI, FDI, FSWAP; //frequency of the different types of move

    ConstFsb = 0;
    ConstInFsb = 0;

    FSI = 1;
    FDI = 1; // 5 in original paper
    FSWAP = 1; // 5 in original paper

    int npi, mnpi, nti, mnti;

    mnpi = 900 * (int)sqrt(NRE + NRA); //

    npi = 0; // iteration
    nti = 0; //

    CARPInd BestSINeighbor, BestDINeighbor, BestSWAPNeighbor;
    int ChangedRoutesID[2], MovedTasks[3], BestMovedTasks[3], RID1, RID2;
    int TotalFitEval = 0, FitEval = 0, FitCost[40000][3];
    FitCost[0][0] = 0;
    // while (BestFsbSolution.TotalCost > LLB)

    experience process;
    if(dynamic_type == DYNMAIC)
    {
        process.save_best_so_far_queue(BestFsbSolution);
    }

    record savedata;
    if (SAVE)
    {
        savedata.createfile(dynamic_type);
        savedata.parse_data(0.0, BestFsbSolution.TotalCost, 1);
    }

    
    clock_t start_t, finish_t;
    start_t = clock();
    double duration=0;
    int prev_best=BestFsbSolution.TotalCost;
    
    int terminal_duration = 60;
    // while (npi < mnpi && stop_iter < terminal_condition)   
    while (duration < terminal_duration)
    {
        NextSolution.Fitness = INF;
        int improved = 0;

        if (npi % FSI == 0)
        {
            // single insertion
            FitEval = SingleInsertion(&BestSINeighbor, &CurrSolution, MovedTasks, ChangedRoutesID, inst_tasks, TabuList, PenPrmt, BestSolution.Fitness, BestFsbSolution.Fitness);
            TotalFitEval += FitEval;


            if (BestSINeighbor.TotalVioLoad == 0 && BestSINeighbor.TotalCost < BestFsbSolution.TotalCost)
            {
                improved = 1;
                BestFsbSolution = BestSINeighbor;
                FitCost[0][0]++;
                FitCost[FitCost[0][0]][1] = TotalFitEval;
                FitCost[FitCost[0][0]][0] = npi;
                FitCost[FitCost[0][0]][2] = BestFsbSolution.TotalCost;
            }
            if (BestSINeighbor.Fitness < NextSolution.Fitness)
            {
                NextSolution = BestSINeighbor;
                RID1 = ChangedRoutesID[0];
                RID2 = ChangedRoutesID[1];
                AssignArray(MovedTasks, BestMovedTasks);
            }
            
        }
        check_solution_valid(CurrSolution, inst_tasks);

        if (npi % FDI == 0)
        {
            // double insertion
            FitEval = DoubleInsertion(&BestDINeighbor, &CurrSolution, MovedTasks, ChangedRoutesID, inst_tasks, TabuList, PenPrmt, BestSolution.Fitness, BestFsbSolution.Fitness);
            TotalFitEval += FitEval;

            if (BestDINeighbor.TotalVioLoad == 0 && BestDINeighbor.TotalCost < BestFsbSolution.TotalCost)
            {
                improved = 1;
                BestFsbSolution = BestDINeighbor;
                FitCost[0][0]++;
                FitCost[FitCost[0][0]][1] = TotalFitEval;
                FitCost[FitCost[0][0]][0] = npi;
                FitCost[FitCost[0][0]][2] = BestFsbSolution.TotalCost;
            }
            if (BestDINeighbor.Fitness < NextSolution.Fitness)
            {
                NextSolution = BestDINeighbor;
                RID1 = ChangedRoutesID[0];
                RID2 = ChangedRoutesID[1];
                AssignArray(MovedTasks, BestMovedTasks);
            }
        }

        check_solution_valid(CurrSolution, inst_tasks);

        if (npi % FSWAP == 0)
        {
            // swap
            FitEval = SWAP(&BestSWAPNeighbor, &CurrSolution, MovedTasks, ChangedRoutesID, inst_tasks, TabuList, PenPrmt, BestSolution.Fitness, BestFsbSolution.Fitness);
            TotalFitEval += FitEval;

            if (BestSWAPNeighbor.TotalVioLoad == 0 && BestSWAPNeighbor.TotalCost < BestFsbSolution.TotalCost)
            {
                improved = 1;
                BestFsbSolution = BestSWAPNeighbor;
                FitCost[0][0]++;
                FitCost[FitCost[0][0]][1] = TotalFitEval;
                FitCost[FitCost[0][0]][0] = npi;
                FitCost[FitCost[0][0]][2] = BestFsbSolution.TotalCost;
            }
            if (BestSWAPNeighbor.Fitness < NextSolution.Fitness)
            {
                NextSolution = BestSWAPNeighbor;
                RID1 = ChangedRoutesID[0];
                RID2 = ChangedRoutesID[1];
                AssignArray(MovedTasks, BestMovedTasks);
            }
        }
        check_solution_valid(CurrSolution, inst_tasks);

        npi ++;
        nti ++;

        if (NextSolution.TotalVioLoad == 0)
        {
            ConstFsb ++;
        } else{
            ConstInFsb ++;
        }

        // repair operator
        
        if (NextSolution.TotalCost < BestFsbSolution.TotalCost && NextSolution.TotalVioLoad > 0)
        {
            TmpSolution = NextSolution;
            indi_route_converter(&TmpSolution, &NextSolution, inst_tasks);
            check_solution_valid(TmpSolution, inst_tasks);
            RepairInfeasibility(&TmpSolution, inst_tasks);
            check_solution_valid(TmpSolution, inst_tasks);
            TotalFitEval ++;


            if(TmpSolution.TotalVioLoad < NextSolution.TotalVioLoad || (TmpSolution.TotalVioLoad == NextSolution.TotalVioLoad && TmpSolution.TotalCost < NextSolution.TotalCost))
            {
                NextSolution = TmpSolution;
            }

            if (NextSolution.TotalVioLoad == 0 && NextSolution.TotalCost < BestFsbSolution.TotalCost)
            {
                improved = 1;
                BestFsbSolution = NextSolution;
                FitCost[0][0]++;
                FitCost[FitCost[0][0]][1] = TotalFitEval;
                FitCost[FitCost[0][0]][0] = npi;
                FitCost[FitCost[0][0]][2] = BestFsbSolution.TotalCost;
            }
        }
        check_cost(BestFsbSolution, inst_tasks);
 
        if (NextSolution.TotalVioLoad == 0)
        {
            TmpSeq[0] = 1; // tmp solution to link all processed sub-routes
            NextSolution.TotalCost = 0;

            find_ele_positions(Position, NextSolution.Sequence, 0);
            for(i = 1; i < Position[0]; i++)
            {
                AssignSubArray(NextSolution.Sequence, Position[i], Position[i+1], Route);
                FredericksonHeuristic(FHRoute, Route, inst_tasks);
                Cost1 =get_task_seq_total_cost(Route, inst_tasks);
                Cost2 = get_task_seq_total_cost(FHRoute, inst_tasks);

                TmpSeq[0] --;
                if (Cost1 < Cost2)
                {
                    JoinArray(TmpSeq, Route); //link two routes
                    NextSolution.TotalCost += Cost1;
                } else {
                    JoinArray(TmpSeq, FHRoute);
                    NextSolution.TotalCost += Cost2;
                }

            }

            NextSolution.Fitness = NextSolution.TotalCost;
            AssignArray(TmpSeq, NextSolution.Sequence);
        }
        

        if (improved && NextSolution.TotalVioLoad == 0)
        {
            BestFsbSolution = NextSolution;
            nti = 0;
        }

        NextSolution.Assignment[0] = req_edge_num + req_arc_num;
        j = 1;
        for (i = 2; i < NextSolution.Sequence[0]; i++)
        {
            if (NextSolution.Sequence[i] == 0)
            {
                j++;
                continue;
            }

            int tmp = NextSolution.Sequence[i];
            if ( tmp > req_edge_num)
                tmp -= req_edge_num ;

            NextSolution.Assignment[tmp] = j;

        }

        for (i = 1; i <= 2*req_edge_num + req_arc_num; i++)
        {
            for (j = 0; j <= vehicle_num; j++)
            {
                if (TabuList[i][j] > 0)
                    TabuList[i][j] --;
            }
        }

        // Assignment: each task belongs to which routes, 1 -> req_edge_num
        for (i = 1; i <= NextSolution.Assignment[0]; i++)
        {
            if (NextSolution.Assignment[i] == CurrSolution.Assignment[i])
                continue;

            int tmp = i;
            if (tmp > req_edge_num)
            {
                tmp += req_edge_num;
            }


            TabuList[tmp][CurrSolution.Assignment[i]] = TabuTenure;
            if ( i <= req_edge_num)
            {
                TabuList[i+req_edge_num][CurrSolution.Assignment[i]] = TabuTenure;
            }
        }

        if (NextSolution.Fitness < BestSolution.Fitness)
            BestSolution = NextSolution;

        CurrSolution = NextSolution;

        if (npi % 10 == 0)
        {
            if (ConstFsb == 10)
            {
                PenPrmt /= 2;
                BestSolution.Fitness = BestSolution.TotalCost + PenPrmt * BestSolution.TotalVioLoad;
                CurrSolution.Fitness = CurrSolution.TotalCost + PenPrmt * CurrSolution.TotalVioLoad;
            } else if (ConstInFsb == 10)
            {
                PenPrmt *= 2;
                BestSolution.Fitness = BestSolution.TotalCost + PenPrmt * BestSolution.TotalVioLoad;
                CurrSolution.Fitness = CurrSolution.TotalCost + PenPrmt * CurrSolution.TotalVioLoad;
            }
            // printf("PenPrmt: %f \n", PenPrmt);
            ConstFsb = 0;
            ConstInFsb = 0;
        }

        // if (npi == mnpi)
        //     break;

        // if (npi % 100 == 0)
        // // // if (npi)
        // {
        //         printf("npi, %d, BestFsbCost = %d\n", npi, BestFsbSolution.TotalCost);
        // }

        if(SAVE){
            finish_t = clock();
            duration = (double)(finish_t - start_t) / CLOCKS_PER_SEC;
            savedata.parse_data(duration, BestFsbSolution.TotalCost, 2);
        }

        // save solutions
        if(BestFsbSolution.TotalCost < prev_best) {
            if ((dynamic_type == DYNMAIC && run_num == 1) || (dynamic_type == RESTART && dym_t == 0)) {
                process.save_best_so_far_queue(BestFsbSolution);
            }
            prev_best = BestFsbSolution.TotalCost;
        }

        finish_t = clock();
        duration = (double)(finish_t - start_t) / CLOCKS_PER_SEC;
    }

    if ((dynamic_type == DYNMAIC && run_num == 1) || (dynamic_type == RESTART && dym_t == 0)) {
        process.move_best_to_first();
        process.save_solutions(inst_tasks);
    }

    indi_route_converter(TSASolution, &BestFsbSolution, inst_tasks);
    memcpy(TSASolution->Assignment, BestFsbSolution.Assignment, sizeof(BestFsbSolution.Assignment));
    TSASolution->TotalCost = BestFsbSolution.TotalCost;
    TSASolution->TotalVioLoad = BestFsbSolution.TotalVioLoad;
    TSASolution->Fitness = BestFsbSolution.Fitness;
    printf("type: %d. runnum: %d. Best Solution's Cost: %d\n", dynamic_type, run_num, BestFsbSolution.TotalCost);
}



void check_solution_valid(CARPInd solution, const Task *inst_task)
{
    int i, j;
    int used[MAX_TASK_SEQ_LENGTH];
    memset(used, 0, sizeof(used));
    int route_num = 0;\
    for (i=1; i<solution.Sequence[0]; i++)
    {
        if (solution.Sequence[i] == 0 && solution.Sequence[i+1] == 0)
        {
            printf("sequence has some errors.\n");
        }
    }
    for(i = 1; i <= solution.Sequence[0]; i++)
    {
        if (solution.Sequence[i] == 0)
        {
            route_num ++;
            continue;
        }
        if (solution.Sequence[i] <= req_edge_num)
        {
            used[solution.Sequence[i]] = 1;
        } else
        {
            used[inst_task[solution.Sequence[i]].inverse] = 1;
        }
    }
    int flag = 0;
    if((route_num-1) != solution.Loads[0])
    {
        printf("route num error\n");
        flag = 1;
    }
//     printf("lack of: ");
     for (i = 1; i <= req_edge_num; i++)
     {
         if(used[i] == 0)
         {
             printf("not used %d \t", i);
             flag = 1;
         }
     }
     // printf("\n");
     // get_each_load(solution.Sequence, inst_task);
//     if ( solution.TotalCost != get_task_seq_total_cost(solution.Sequence, inst_task))
//     {
//         flag = 1;
//         printf("solution's cost error\n");
//     }
     if (flag)
     {
         printf("\n");
         exit(0);
     }
}
int check_cost(CARPInd solution, const Task *inst_tasks)
{
    int cost = get_task_seq_total_cost(solution.Sequence, inst_tasks);
    if (cost != solution.TotalCost)
    {
        return 0;
    }
     return 1;
}