//
// Created by hao on 15/06/2020.
//

#include "MAENS.h"
//#include "../utils.h"
#define RECORD 0
#define STATIC_MAX_TIME 180
#define DYNAMIC_MAX_TIME 60
#define DYNMAIC 1
#define RESTART 0
#define SAVE 0
#define LOG 0

int MAENS(const Task *inst_tasks, CARPInd *MAENSolution, CARPInd InitSolution, const int dynamic_type)
{

    int i;

    int popsize = 30;
    CARPInd pop[MAX_TOTALSIZE];
    CARPInd best_fsb_solution;
    best_fsb_solution.TotalCost = INF;
    indi_copy(&best_fsb_solution, &InitSolution);
    // initialization
    int tmp_popsize = 0;
    int algorithm_best= INF;
    if (dynamic_type==DYNMAIC) {

        // use inherited solutions
       repair_old_solutions exp(inst_tasks, InitSolution);
        // insert_new_tasks exp(inst_tasks);

        tmp_popsize = 0;
        int old_sol_idx = 0;
        while (tmp_popsize < popsize) {
            if (old_sol_idx < exp.sol_num) {
                pop[tmp_popsize] = exp.sols[old_sol_idx];
                if(exp.sols[old_sol_idx].TotalCost < algorithm_best)
                {
                    algorithm_best = exp.sols[old_sol_idx].TotalCost;
                }
                tmp_popsize++;
                old_sol_idx++;
            } else {
                CARPInd init_indi;
                int stop = generate_rand_solution(tmp_popsize, &init_indi, pop, inst_tasks);
                if (stop)
                    break;
                pop[tmp_popsize] = init_indi;
                tmp_popsize++;
            }
            if (pop[tmp_popsize - 1].TotalVioLoad == 0 &&
                pop[tmp_popsize - 1].TotalCost < best_fsb_solution.TotalCost) {
                best_fsb_solution = pop[tmp_popsize - 1];
            }
//            printf("dynamic cost: %d\n", pop[tmp_popsize - 1].TotalCost);
        }
        popsize = tmp_popsize;
//        printf("rand cost: %d\n", best_fsb_solution.TotalCost);
    } else {
        // use random generation
        while (tmp_popsize < popsize) {
            CARPInd init_indi;

            int stop = generate_rand_solution(tmp_popsize, &init_indi, pop, inst_tasks);
            if (stop)
                break;

            pop[tmp_popsize] = init_indi;
//            printf("rand cost: %d\n", init_indi.TotalCost);
            tmp_popsize++;
            if (init_indi.TotalVioLoad == 0 && init_indi.TotalCost < best_fsb_solution.TotalCost) {
                best_fsb_solution = init_indi;
            }
        }
        algorithm_best = best_fsb_solution.TotalCost;
        popsize = tmp_popsize;
    //    printf("best: %d\n", best_fsb_solution.TotalCost);
    }
    // main loop
//    return;

    MAENS_main_loop(popsize, &best_fsb_solution, pop, inst_tasks, dynamic_type);

    if (DYNMAIC)
    {
        // population has been ranked by stochastic ranking
        experience process;
        int save_sol_num;
        
        save_sol_num = process.save_best_so_far(best_fsb_solution);
        for (i=0; i<MAX_TOTALSIZE; i++)
        {
            if(pop[i].Sequence[0] == 0)
            {
                continue;
            }
            if (pop[i].TotalCost == best_fsb_solution.TotalCost) 
            {
                continue;
            }
            if (pop[i].TotalVioLoad == 0&& pop[i].TotalCost != 0 )
            {
                save_sol_num = process.save_best_so_far(pop[i]);
//                printf("save solution's cost: %d\n", pop[i].TotalCost);
            }
            if(save_sol_num>=30)
            {
                break;
            }
        }
        process.save_solutions(inst_tasks);
    }

    indi_route_converter(MAENSolution, &best_fsb_solution, inst_tasks);
    memcpy(MAENSolution->Assignment, best_fsb_solution.Assignment, sizeof(best_fsb_solution.Assignment));
    MAENSolution->TotalCost = best_fsb_solution.TotalCost;
    MAENSolution->TotalVioLoad = best_fsb_solution.TotalVioLoad;
    MAENSolution->Fitness = best_fsb_solution.Fitness;
//    printf("type: %d. runnum: %d. Best Solution's Cost: %d\n", dynamic_type, run_num, MAENSolution->TotalCost);
    printf("type: %d.. Best Solution's Cost: %d\n", dynamic_type, MAENSolution->TotalCost);

    return algorithm_best;
}

int generate_rand_solution(int tmp_popsize, CARPInd *init_indi, CARPInd *pop, const Task *inst_tasks)
{
    int i, used;
    int trial = 0;
//    CARPInd init_indi;
    while (trial < M_trial)
    {
        trial ++;
        int serve_mark[MAX_TASK_TAG_LENGTH];
        memset(serve_mark, 0, sizeof(serve_mark));
        for (i = 1; i <= task_num; i++)
        {
            serve_mark[i] = 1;
        }

        rand_scanning(init_indi, inst_tasks, serve_mark);
        used = 0;
        for (i = 0; i < tmp_popsize; i++)
        {
            if (init_indi->TotalCost == pop[i].TotalCost && init_indi->TotalVioLoad == pop[i].TotalVioLoad)
            {
                used = 1;
                break;
            }
        }
        if ( !used )
            break;
    }

    if (trial == M_trial && used == 1)
        return 1;

    return 0;
}

void MAENS_main_loop(int popsize, CARPInd *best_fsb_solution, CARPInd *pop, const Task *inst_tasks, const int dynamic_type)
{
    record savedata;
    if (SAVE)
    {
        savedata.createfile(dynamic_type);
        savedata.parse_data(0.0, best_fsb_solution->TotalCost, 1);
    }
    if (LOG){
        printf("best: %d\n", best_fsb_solution->TotalCost);
    }

    int i,j;
    clock_t start_t, finish_t;
    start_t = clock();
    double duration = 0.0;
//    int short_record_flag = 1;
//    int success=0, tmptpoint=0;
//    double tpoint=0;

    int ite, wite;
    CARPInd parent1, parent2, xed_child, mted_child, child;

    int offsize = 6*popsize;
    int totalsize = popsize + offsize;

    int used;
    ite = 0;
    wite = 0;
    int stop_iter = 0, old_best = INF;
    // while (ite < M_ite && stop_iter < terminal_condition)
    double max_duration;

    max_duration = DYNAMIC_MAX_TIME;

    while (duration < max_duration)
    {
        ite ++;
        wite ++;

        int ptr = popsize;
        while (ptr < totalsize)
        {
            child.TotalCost = 0;

            // randomly select two parents
            int par_id1, par_id2;
            if (popsize > 1)
            {
                rand_selection(&par_id1, &par_id2, popsize);
            } else {
                par_id1 = 0;
                par_id2 = 0;
            }
            
            parent1 = pop[par_id1];
            // parent1 = InitSolution;
            parent2 = pop[par_id2];

            // crossover
            SBX(&xed_child, &parent1, &parent2, inst_tasks);
            if (xed_child.TotalVioLoad == 0 && xed_child.TotalCost < best_fsb_solution->TotalCost)
            {
                indi_copy(best_fsb_solution, &xed_child);
                if (LOG){
                    finish_t = clock();
                    duration = (double)(finish_t - start_t) / CLOCKS_PER_SEC;
                    printf("line209, duration:%f, best: %d\n", duration, best_fsb_solution->TotalCost);
                }
                if(SAVE){
                    finish_t = clock();
                    duration = (double)(finish_t - start_t) / CLOCKS_PER_SEC;
                    savedata.parse_data(duration, best_fsb_solution->TotalCost, 2);
                }
                wite = 0;
            }

            used = 0;
            for (i = 0; i < ptr; i++)
            {
                if (i == par_id1 || i == par_id2)
                    continue;

                if (xed_child.TotalCost == pop[i].TotalCost && xed_child.TotalVioLoad == pop[i].TotalVioLoad)
                {
                    used = 1;
                    break;
                }
            }

            if (!used)
            {
                child = xed_child;
            }

            // Local Search with Probability
            double random = 1.0 * rand() / RAND_MAX;
            if (random < M_PROB)
            {
                // do the local search.
//                int best_before = best_fsb_solution->TotalCost;
                lns_mut(&mted_child, &xed_child, best_fsb_solution, inst_tasks);
                if (LOG){
                    finish_t = clock();
                    duration = (double)(finish_t - start_t) / CLOCKS_PER_SEC;
                    printf("line248, duration:%f, best: %d\n", duration, best_fsb_solution->TotalCost);
                }
                //

                if(SAVE){
                    finish_t = clock();
                    duration = (double)(finish_t - start_t) / CLOCKS_PER_SEC;
                    savedata.parse_data(duration, best_fsb_solution->TotalCost,3);
                }

                //
                check_solution_valid(mted_child, inst_tasks);
                used = 0;
                for (i = 0; i < ptr; i++)
                {
                    if (i == par_id1 || i == par_id2)
                        continue;

                    if (mted_child.TotalCost == pop[i].TotalCost && mted_child.TotalVioLoad == pop[i].TotalVioLoad)
                    {
                        used = 1;
                        break;
                    }
                }

                if (!used)
                {
                    child = mted_child;
                }
            }

            if (child.TotalCost == parent1.TotalCost && child.TotalVioLoad == parent1.TotalVioLoad)
            {
                pop[par_id1] = child;
            } else if (child.TotalCost == parent2.TotalCost && child.TotalVioLoad == parent2.TotalVioLoad)
            {
                pop[par_id2] = child;
            } else if (child.TotalCost > 0)
            {
                pop[ptr] = child;
                ptr ++;
            }
            finish_t = clock();
            duration = (double)(finish_t - start_t) / CLOCKS_PER_SEC;
//            printf("t: %f, best: %d\n", duration, best_fsb_solution->TotalCost);
        }

        // stochastic ranking
        float Pf = 0.45;
        CARPInd tmp_indi;


        for (i = 0; i < totalsize; i++)
        {
            for (j = 0; j < i; j++)
            {
                double random = 1.0 * rand() / RAND_MAX;
                if ( (pop[j].TotalVioLoad == 0 && pop[j+1].TotalVioLoad == 0) || random < Pf )
                {
                    if (pop[j].TotalCost > pop[j+1].TotalCost)
                    {
                        tmp_indi = pop[j];
                        pop[j] = pop[j+1];
                        pop[j+1] = tmp_indi;
                    }
                } else {
                    if (pop[j].TotalVioLoad > pop[j+1].TotalVioLoad)
                    {
                        tmp_indi = pop[j];
                        pop[j] = pop[j+1];
                        pop[j+1] = tmp_indi;
                    }
                }
            }
        }


        if (best_fsb_solution->TotalCost < old_best)
        {
            stop_iter = 0;
            old_best = best_fsb_solution->TotalCost;
        } else
        {
            stop_iter ++;
        }
        if (LOG){
            printf("line328, best: %d\n", best_fsb_solution->TotalCost);
        }
        if(SAVE){
            finish_t = clock();
            duration = (double)(finish_t - start_t) / CLOCKS_PER_SEC;
            savedata.parse_data(duration, best_fsb_solution->TotalCost, 4);
        }
//        printf("ite: %d, MAENS: %d\n", ite, best_fsb_solution->TotalCost);
    }
}



