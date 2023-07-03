#include "utils.h"
#include "src/src.h"
#include "libs/cmdline.h"
#include "new_heuristic.h"

#include "MAENS/MAENS.h"
#include <fstream>

#define CMDFLAG 1

#define DYNMAIC 1
#define RESTART 0

void static_optimization(Task *inst_tasks, Task *inst_tasks_vt, Arc *inst_arcs, Vehicles state);
void dynamic_scenario();
void check_save_solutions(const Task *inst_tasks);
void compare_best_old_solution(const Task *inst_tasks, CARPInd OldSolution);
void compare_initial_solutions_quality(const Task *inst_tasks, CARPInd OldSolution);
void read_static_best(const Task *inst_tasks, CARPInd *best);

int main(int argc, char *argv[])
{
    run_num = 1;
    scn_num = 1;
    int init_dym_t = 1;
    if (CMDFLAG)
    {
        cmdline::parser inst_info;
        inst_info.add<double>("costScale", 'C', "cost change ratio", false, 5.0);
        inst_info.add<double>("taskPro", 'p', "add task probability", false, 0.2);
        inst_info.add<int>("scenario", 's', "scenario repetition", true, 0, cmdline::range(0, 65535));
//        inst_info.add<int>("ins", 'i', "instance idx", true, 0, cmdline::range(0, 65535));
        inst_info.add<std::string>("map", 'm', "map name", true, "");

        inst_info.parse_check(argc, argv);
        // pro_cost_change = inst_info.get<double>("pro");
        cost_change_ratio = inst_info.get<double>("costScale");
        add_task_pro = inst_info.get<double>("taskPro");
        scn_num = inst_info.get<int>("scenario");
        strcpy(map, inst_info.get<std::string>("map").c_str());
//        init_dym_t = inst_info.get<int>("ins");
    } else {
        strcpy(map, "egl-g1-A");
        run_num = 1;
        init_dym_t = 1;
        scn_num=1;
        cost_change_ratio = 5;
        add_task_pro = 0.6;
    }
    printf("map: %s\n", map);

    file_name_sep = (int)(add_task_pro * 10);
//    file_name_sep = (int)(cost_change_ratio);

    dynamic_scenario();
    return 0;



    Task inst_tasks[MAX_TASKS_TAG_LENGTH];
    Task inst_tasks_vt[MAX_TASKS_TAG_LENGTH];
    Arc inst_arcs[MAX_ARCS_TAG_LENGTH];
    Vehicles state;

    init_dym_t = 0;
    dym_t = 0;
//    static_optimization(inst_tasks, inst_tasks_vt, inst_arcs, state);

    for (dym_t=1; dym_t<=5; dym_t++) {
        init_vehicle_state(&state);
        read_instance_from_xml(inst_tasks, inst_arcs,&state); // ************* obtain instance information from xml *************
        if (inst_tasks[1].inverse < 20) {
            break;
        }

        update_cost(inst_tasks, inst_arcs);
        mod_dijkstra();
        construct_virtual_task(inst_tasks, inst_tasks_vt, state.stop, state.remain_capacity);
        int additional_cost = get_additional_cost(state);
        available_edges_map(inst_tasks_vt);

        CARPInd dcarp_solution, old_solution, exe_solution, init_solution;
        construct_old_solution(&old_solution, state, inst_tasks_vt);
        copy_individual(&init_solution, &old_solution);

//        compare_best_old_solution(inst_tasks_vt, old_solution);
        compare_initial_solutions_quality(inst_tasks_vt, old_solution);
        continue;

//        CARPInd ps_solution;
//        int ServeMark[2*req_edge_num + req_arc_num + 1];
//        memset(ServeMark, 1, sizeof(ServeMark));
//        path_scanning(&ps_solution, inst_tasks_vt, ServeMark);
//        copy_individual(&init_solution, &ps_solution);


        for (run_num=1; run_num<=25; run_num++)
        {
            int seed = run_num * 100;
            srand(seed);
            TSA(inst_tasks_vt, &dcarp_solution, init_solution, DYNMAIC); // use experience

            if (run_num == 1)
            {
                copy_individual(&exe_solution, &dcarp_solution);
            }

            srand(seed);
            TSA(inst_tasks_vt, &dcarp_solution, init_solution, RESTART); // restart from scratch
        }
//        copy_individual(&exe_solution, &ps_solution);
        int seed = 1655895207 * scn_num + 20221111 * dym_t;
        nextScenario(&exe_solution, inst_tasks_vt, inst_tasks, inst_arcs, &state, seed);
        save_dcarp_instance(inst_tasks, inst_arcs, state, instance_id);
        if (inst_tasks[1].inverse < 20) {
            break;
        }
    }

    return 0;
}

void dynamic_scenario()
{
    Task inst_tasks[MAX_TASKS_TAG_LENGTH];
    Task inst_tasks_vt[MAX_TASKS_TAG_LENGTH];
    Arc inst_arcs[MAX_ARCS_TAG_LENGTH];
    Vehicles state;

    dym_t = 0;
    static_optimization(inst_tasks, inst_tasks_vt, inst_arcs, state);

    char path[100];
    memset(path, 0, sizeof(path));
    sprintf(path, "degl/%s/init-cost-P%d-s%d.txt", map, file_name_sep, scn_num);
    std::fstream stream;
    stream.open(path, std::ios::out);
    if (!stream.is_open())
    {
        std::cout << path << " open fail" << std::endl;
        exit(0);
    }

    for (dym_t=1; dym_t<=5; dym_t++) {
        init_vehicle_state(&state);
        read_instance_from_xml(inst_tasks, inst_arcs,&state); // ************* obtain instance information from xml *************
        update_cost(inst_tasks, inst_arcs);
        mod_dijkstra();
        get_cost_scale(inst_tasks);
        construct_virtual_task(inst_tasks, inst_tasks_vt, state.stop, state.remain_capacity);
        int additional_cost = get_additional_cost(state);
        available_edges_map(inst_tasks_vt);

        CARPInd dcarp_solution, ps_solution, exe_solution;
    //    construct_old_solution(&old_solution, state, inst_tasks_vt);

        int ServeMark[2*req_edge_num + req_arc_num + 1];
        memset(ServeMark, 1, sizeof(ServeMark));
        path_scanning(&ps_solution, inst_tasks_vt, ServeMark);

        for (run_num=1; run_num<=25; run_num++)
        {
            int seed = run_num * 100;
            srand(seed);
            int hsas_cost, rand_cost;
            hsas_cost = MAENS(inst_tasks_vt, &dcarp_solution, ps_solution, DYNMAIC); // use experience

            if (run_num == 1)
            {
                copy_individual(&exe_solution, &dcarp_solution);
            }

            srand(seed);
            rand_cost = MAENS(inst_tasks_vt, &dcarp_solution, ps_solution, RESTART); // restart from scratch

            stream << "H " << hsas_cost << " P " << ps_solution.TotalCost << " R " << rand_cost << std::endl;
        }
        stream << "XXXXXX" << std::endl;

        int seed = 1655895207 * scn_num + 20221111 * dym_t;
        nextScenario(&exe_solution, inst_tasks_vt, inst_tasks, inst_arcs, &state, seed);
        save_dcarp_instance(inst_tasks, inst_arcs, state, instance_id);
        if (inst_tasks[1].inverse < 20) {
            break;
        }
    }
    stream.close();
}
void static_optimization(Task *inst_tasks, Task *inst_tasks_vt, Arc *inst_arcs, Vehicles state)
{
    CARPInd static_best;
    read_dynamic_map(inst_tasks, inst_arcs, map, &static_best);
    memcpy(inst_tasks_vt, inst_tasks, sizeof(Task)*MAX_TASKS_TAG_LENGTH);
    update_cost(inst_tasks, inst_arcs);
    mod_dijkstra();
    get_cost_scale(inst_tasks);
    CARPInd ps_solution;
    int ServeMark[2*req_edge_num + req_arc_num + 1];
    memset(ServeMark, 1, sizeof(ServeMark));
    path_scanning(&ps_solution, inst_tasks_vt, ServeMark);

//    copy_individual(&static_best, &ps_solution);

//    MAENS(inst_tasks_vt, &static_best, ps_solution, RESTART); // obtain static best solution, once obtain and write into .dat, comment this line
    read_static_best(inst_tasks, &static_best);

    int seed;
    seed = 1655895207*scn_num;
    nextScenario(&static_best, inst_tasks_vt, inst_tasks, inst_arcs, &state, seed);
    save_dcarp_instance(inst_tasks, inst_arcs, state, instance_id);
}

void read_static_best(const Task *inst_tasks, CARPInd *best)
{
    // best solution for dym_t
    char path[100];
    int NO_task = inst_tasks[inst_tasks[1].inverse - 1].inverse;
//    char path[100];
    memset(path, 0, sizeof(path));
    sprintf(path, "degl/%s/s%d-sols0.txt", map, scn_num);
    FILE *fp;
    fp = fopen(path, "r");
    int i,j, task_id;
    int node1, node2, data_num;
    fscanf(fp, "num,%d\n", &data_num);
    int task_num_cal, new_sol_seq[MAX_TASK_SEG_LENGTH];
    for (i=0;i < data_num;i++)
    {
        memset(new_sol_seq, 0, sizeof(new_sol_seq));
        task_num_cal = 0;
        new_sol_seq[0] = 0;
        while (fscanf(fp, "%d,", &node1))
        {
            if (node1 == -1)
            {
                break;
            }
            if (node1 == 0)
            {
                new_sol_seq[0]++;
                new_sol_seq[new_sol_seq[0]] = 0;
                continue;
            }
            fscanf(fp, "%d,", &node2);
            task_id = FindTask(node1, node2, inst_tasks, NO_task);
            if (task_id == 0)
            {
                task_id = -1;
            }else{
                task_num_cal++;
            }
            new_sol_seq[0]++;
            new_sol_seq[new_sol_seq[0]] = task_id;
        }
        int cost = get_task_seq_total_cost(new_sol_seq, inst_tasks);
        best->TotalCost = cost;
        memcpy(best->Sequence, new_sol_seq, sizeof(new_sol_seq));
        get_task_seq_loads(best->Loads, new_sol_seq, inst_tasks);
        break;
        // repair tmp_sol_seq;
    }
    fclose(fp);
}

void compare_best_old_solution(const Task *inst_tasks, CARPInd OldSolution)
{
    // hamming distance write to file
    char path[100];
    memset(path, 0, sizeof(path));
    sprintf(path, "degl/%s/dis-s%d-i%d.txt", map, scn_num, dym_t);

    std::fstream stream;
    stream.open(path, std::ios::out);
    if (!stream.is_open())
    {
        std::cout << path << " open fail" << std::endl;
        exit(0);
    }

    // best solution for dym_t
    int NO_task = inst_tasks[inst_tasks[1].inverse - 1].inverse;
//    char path[100];
    memset(path, 0, sizeof(path));
    sprintf(path, "degl/%s/s%d-sols%d.txt", map, scn_num, dym_t);
    FILE *fp;
    fp = fopen(path, "r");
    int i,j, task_id;
    int node1, node2, data_num;
    fscanf(fp, "num,%d\n", &data_num);
    int task_num_cal, new_sol_seq[MAX_TASK_SEG_LENGTH];
    for (i=0;i < data_num;i++)
    {
        memset(new_sol_seq, 0, sizeof(new_sol_seq));
        task_num_cal = 0;
        new_sol_seq[0] = 0;
        while (fscanf(fp, "%d,", &node1))
        {
            if (node1 == -1)
            {
                break;
            }
            if (node1 == 0)
            {
                new_sol_seq[0]++;
                new_sol_seq[new_sol_seq[0]] = 0;
                continue;
            }
            fscanf(fp, "%d,", &node2);
            task_id = FindTask(node1, node2, inst_tasks, NO_task);
            if (task_id == 0)
            {
                task_id = -1;
            }else{
                task_num_cal++;
            }
            new_sol_seq[0]++;
            new_sol_seq[new_sol_seq[0]] = task_id;
        }
        break;
        // repair tmp_sol_seq;
    }
    fclose(fp);

    CARPInd best_fsb_solution;
    // initial solutions for dym_t-1
    insert_new_tasks EXP(inst_tasks);
//    repair_old_solutions EXP(inst_tasks, OldSolution);
    int popsize0 = EXP.sol_num;

    // remove virtual task in ros solutions and compute the hamming distance
    int old_sol_seqs[MAX_TASK_SEG_LENGTH];
    stream << "D ";
    for (i=0; i<popsize0; i++)
    {
        memcpy(old_sol_seqs, &EXP.sols[i].Sequence, sizeof(new_sol_seq));
        for (j=old_sol_seqs[0]; j>=1; j--)
        {
            if(inst_tasks[old_sol_seqs[j]].vt > 0)
            {
                delete_element(old_sol_seqs, j);
            }
        }
        float sim = seq_similarity(old_sol_seqs, new_sol_seq, inst_tasks);
        stream << sim << " ";
    }
    stream << std::endl;
//    printf("*************\n");

    // solutions generated by path scanning
    stream << "P ";
    CARPInd ps_solution;
    int ServeMark[2*req_edge_num + req_arc_num + 1];
    memset(ServeMark, 1, sizeof(ServeMark));
    path_scanning(&ps_solution, inst_tasks, ServeMark);
    memcpy(old_sol_seqs, ps_solution.Sequence, sizeof(new_sol_seq));
    for (j=old_sol_seqs[0]; j>=1; j--)
    {
        if(inst_tasks[old_sol_seqs[j]].vt > 0)
        {
            delete_element(old_sol_seqs, j);
        }
    }
    float sim = seq_similarity(old_sol_seqs, new_sol_seq, inst_tasks);
    stream << sim << " ";
    stream << std::endl;


    // solutions generated by MAENS restart
    int seed = 1 * 100;
    srand(seed);

    int tmp_popsize = 0, popsize = 30;
    int used;
    CARPInd pop[210];
    best_fsb_solution.TotalCost = INF;
    while (tmp_popsize < popsize)
    {
        int trial = 0;
        CARPInd init_indi;
        while (trial < 10)
        {
            trial++;
            int serve_mark[MAX_TASK_TAG_LENGTH];
            memset(serve_mark, 0, sizeof(serve_mark));
            for (i = 1; i <= task_num; i++)
            {
                serve_mark[i] = 1;
            }

            rand_scanning(&init_indi, inst_tasks, serve_mark);
            used = 0;
            for (i = 0; i < tmp_popsize; i++)
            {
                if (init_indi.TotalCost == pop[i].TotalCost && init_indi.TotalVioLoad == pop[i].TotalVioLoad)
                {
                    used = 1;
                    break;
                }
            }
            if (!used)
                break;
        }

        if (trial == 10 && used == 1)
            break;

        pop[tmp_popsize] = init_indi;
        tmp_popsize++;
        if (init_indi.TotalVioLoad == 0 && init_indi.TotalCost < best_fsb_solution.TotalCost)
        {
            best_fsb_solution = init_indi;
        }
    }
    popsize = tmp_popsize;

    stream << "R ";
    for (i=0; i<popsize; i++)
    {
        memcpy(old_sol_seqs, &pop[i].Sequence, sizeof(new_sol_seq));
        for (j=old_sol_seqs[0]; j>=1; j--)
        {
            if(inst_tasks[old_sol_seqs[j]].vt > 0)
            {
                delete_element(old_sol_seqs, j);
            }
        }
        float sim = seq_similarity(old_sol_seqs, new_sol_seq, inst_tasks);
        stream << sim << " ";
    }
    stream << std::endl;

    stream.close();
    int a=1;
    int b=1;

}

void obtain_best_seq(const Task *inst_tasks, int *new_sol_seq);
void obtain_best_seq(const Task *inst_tasks, int *new_sol_seq)
{
    char path[100];
    int NO_task = inst_tasks[inst_tasks[1].inverse - 1].inverse;
    memset(path, 0, sizeof(path));
    sprintf(path, "degl/%s/s%d-sols%d.txt", map, scn_num, dym_t);
    FILE *fp;
    fp = fopen(path, "r");
    int i,j, task_id;
    int node1, node2, data_num;
    fscanf(fp, "num,%d\n", &data_num);
    int task_num_cal; //new_sol_seq[MAX_TASK_SEG_LENGTH];
    for (i=0;i < data_num;i++)
    {
        memset(new_sol_seq, 0, MAX_TASK_SEG_LENGTH*sizeof(int));
        task_num_cal = 0;
        new_sol_seq[0] = 0;
        while (fscanf(fp, "%d,", &node1))
        {
            if (node1 == -1)
            {
                break;
            }
            if (node1 == 0)
            {
                new_sol_seq[0]++;
                new_sol_seq[new_sol_seq[0]] = 0;
                continue;
            }
            fscanf(fp, "%d,", &node2);
            task_id = FindTask(node1, node2, inst_tasks, NO_task);
            if (task_id == 0)
            {
                task_id = -1;
            }else{
                task_num_cal++;
            }
            new_sol_seq[0]++;
            new_sol_seq[new_sol_seq[0]] = task_id;
        }
        break;
    }
    fclose(fp);
}

void compare_initial_solutions_quality(const Task *inst_tasks, CARPInd OldSolution)
{
    // quality comparison results write to file
    char path[100];
    memset(path, 0, sizeof(path));
    sprintf(path, "degl/%s/aa-s%d-i%d.txt", map, scn_num, dym_t);

    std::fstream stream;
    stream.open(path, std::ios::out);
    if (!stream.is_open())
    {
        std::cout << path << " open fail" << std::endl;
        exit(0);
    }

    int i, j;
    // load best cost
    memset(path, 0, sizeof(path));
    sprintf(path, "degl/%s/s%d-d-i%d-1.txt", map, scn_num, dym_t);
    FILE *fp;
    fp = fopen(path, "r");
    char dummy[101];
    double tpoint;
    int best;
    while (true)
    {
        fscanf(fp, "%s", dummy);
        fscanf(fp, "%lf", &tpoint);
        fscanf(fp, "%d", &best);
        if (tpoint >= 60 )
        {
            break;
        }
    }
//    printf("%d \n", best) ;
    fclose(fp);
    stream << "B "<<best<<"/1"<<std::endl;


    int new_sol_seq[MAX_TASK_SEG_LENGTH];
    obtain_best_seq(inst_tasks, new_sol_seq);

    // initial solutions for dym_t-1
//    insert_new_tasks EXP(inst_tasks);
     repair_old_solutions EXP(inst_tasks, OldSolution);


    stream << "D ";
    int old_sol_seqs[MAX_TASK_SEG_LENGTH];
    for (i=0; i<EXP.sol_num; i++)
    {
        memcpy(old_sol_seqs, &EXP.sols[i].Sequence, sizeof(old_sol_seqs));
        for (j=old_sol_seqs[0]; j>=1; j--)
        {
            if(inst_tasks[old_sol_seqs[j]].vt > 0)
            {
                delete_element(old_sol_seqs, j);
            }
        }
        float sim = seq_similarity(old_sol_seqs, new_sol_seq, inst_tasks);
        stream << EXP.sols[i].TotalCost << "/" << sim << " ";
    }
    stream << std::endl;


    // solutions generated by path scanning
    stream << "P ";
    CARPInd ps_solution;
    int ServeMark[2*req_edge_num + req_arc_num + 1];
    memset(ServeMark, 1, sizeof(ServeMark));
    path_scanning(&ps_solution, inst_tasks, ServeMark);
    memcpy(old_sol_seqs, ps_solution.Sequence, sizeof(old_sol_seqs));
    for (j=old_sol_seqs[0]; j>=1; j--)
    {
        if(inst_tasks[old_sol_seqs[j]].vt > 0)
        {
            delete_element(old_sol_seqs, j);
        }
    }
    float sim = seq_similarity(old_sol_seqs, new_sol_seq, inst_tasks);
    stream << ps_solution.TotalCost << "/" << sim << std::endl;


    // solutions generated by greedy rand scanning restart
    int seed = 1 * 100;
    srand(seed);

    int tmp_popsize = 0, popsize = 30;
    int used;
    CARPInd pop[210], best_fsb_solution;
    best_fsb_solution.TotalCost = INF;
    while (tmp_popsize < popsize)
    {
        int trial = 0;
        CARPInd init_indi;
        while (trial < 10)
        {
            trial++;
            int serve_mark[MAX_TASK_TAG_LENGTH];
            memset(serve_mark, 0, sizeof(serve_mark));
            for (i = 1; i <= task_num; i++)
            {
                serve_mark[i] = 1;
            }

            rand_scanning(&init_indi, inst_tasks, serve_mark);
            used = 0;
            for (i = 0; i < tmp_popsize; i++)
            {
                if (init_indi.TotalCost == pop[i].TotalCost && init_indi.TotalVioLoad == pop[i].TotalVioLoad)
                {
                    used = 1;
                    break;
                }
            }
            if (!used)
                break;
        }

        if (trial == 10 && used == 1)
            break;

        pop[tmp_popsize] = init_indi;
        tmp_popsize++;
        if (init_indi.TotalVioLoad == 0 && init_indi.TotalCost < best_fsb_solution.TotalCost)
        {
            best_fsb_solution = init_indi;
        }
    }
    popsize = tmp_popsize;

    stream << "R ";
    for (i=0; i<popsize; i++)
    {
        memcpy(old_sol_seqs, &pop[i].Sequence, sizeof(old_sol_seqs));
        for (j=old_sol_seqs[0]; j>=1; j--)
        {
            if(inst_tasks[old_sol_seqs[j]].vt > 0)
            {
                delete_element(old_sol_seqs, j);
            }
        }
        float sim = seq_similarity(old_sol_seqs, new_sol_seq, inst_tasks);
        stream << pop[i].TotalCost << "/" << sim << " ";
    }
    stream << std::endl;

    stream.close();
    int a=1;
    int b=1;
}

void check_save_solutions(const Task *inst_tasks)
{
    int NO_task = inst_tasks[inst_tasks[1].inverse - 1].inverse;
    char path[100];
    memset(path, 0, sizeof(path));
    sprintf(path, "degl/%s/s%d-sols%d.txt", map, scn_num, dym_t);
//    sprintf(path, "degl/%s/sols%d.txt", map, dym_t-1);
    FILE *fp;
    fp = fopen(path, "r");

    int i,j, task_id;
    int node1, node2;
    int data_num;
    fscanf(fp, "num,%d\n", &data_num);

    CARPInd sols[data_num+1];

    int tmp_sol_seq[MAX_TASK_SEG_LENGTH];

    int task_num_cal;
    for (i=0;i<data_num;i++)
    {
        memset(tmp_sol_seq, 0, sizeof(tmp_sol_seq));
        task_num_cal = 0;
        tmp_sol_seq[0] = 0;
        while (fscanf(fp, "%d,", &node1))
        {
            if (node1 == -1)
            {
                break;
            }
            if (node1 == 0)
            {
                tmp_sol_seq[0]++;
                tmp_sol_seq[tmp_sol_seq[0]] = 0;
                continue;
            }
            fscanf(fp, "%d,", &node2);
            task_id = FindTask(node1, node2, inst_tasks, NO_task);
            if (task_id == 0)
            {
                task_id = -1;
            }else{
                task_num_cal++;
            }
            tmp_sol_seq[0]++;
            tmp_sol_seq[tmp_sol_seq[0]] = task_id;
        }
        int cost = get_task_seq_total_cost(tmp_sol_seq, inst_tasks);
        printf("Cost: %d\n", cost);
        int loads[100];
        memset(loads, 0, sizeof(loads));
        get_task_seq_loads(loads, tmp_sol_seq, inst_tasks);
        int a = 1;
        int b = 1;
        // repair tmp_sol_seq;
    }
    fclose(fp);
}