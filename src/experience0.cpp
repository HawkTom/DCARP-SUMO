#include "src.h"

//experience::experience()
//{
//    memset(this->costMatrix, 0, sizeof(costMatrix));
//    memset(this->priorityMatrix, 0, sizeof(priorityMatrix));
//    memset(this->pairNum, 0, sizeof(pairNum));
//    char path[100];
//    memset(path, 0, sizeof(path));
//    sprintf(path, "cost-change-data/k=%d/%s/best/instance%d.txt",
//            (int)cost_change_ratio, map, instance_id);
//
//    FILE *fp;
//    fp = fopen(path, "r");
//    int value;
//    fscanf(fp, "%d\n", &value);
//    int i = -1, len;
//    fscanf(fp, "%d,", &len);
//    for (i=1; i <= len; i++)
//    {
//        fscanf(fp, "%d,", &value);
//    }
//    fscanf(fp, "\nadd:%d\n", &value);
//    fscanf(fp, "maxcost:%d\n", &this->costScale);
//    fscanf(fp, "lb:%d\n", &this->LB);
//    fclose(fp);
//    this->LB = 16093;
//    this->costScale = task_max_cost;
//    printf("class of experience is constructed.%d, %d\n", this->costScale, this->LB);
//}

experience::experience()
{
//    memset(this->costMatrix, 0, sizeof(costMatrix));
//    memset(this->priorityMatrix, 0, sizeof(priorityMatrix));
//    memset(this->transformMatrix, 0, sizeof(transformMatrix));
//    memset(this->pairNum, 0, sizeof(pairNum));
    this->costScale = task_max_cost;
    this->LB = 0;
}

//void experience::initTransformMatrix()
//{
//    memset(this->transformMatrix, 0, sizeof(transformMatrix));
//}
//void experience::setCostMatrix(const Task *inst_tasks)
//{
//    int i, j;
//    int first, next;
//    this->total_task_num = inst_tasks[inst_tasks[1].inverse - 1].inverse;
//
//    int max_cost=0;
//
//    for (i=0; i<=this->total_task_num; i++)
//    {
//        first = inst_tasks[i].tail_node;
//        for (j=0; j<= this->total_task_num; j++)
//        {
//            if (i==j || i==inst_tasks[j].inverse)
//            {
//                this->costMatrix[i][j] = 1;
//                continue;
//            }
//            next = inst_tasks[j].head_node;
//            if(max_cost < min_cost[first][next])
//            {
//                max_cost = min_cost[first][next];
//            }
//            this->costMatrix[i][j] = min_cost[first][next]*1.0/this->costScale;
//        }
//    }
////    printf("maxcost: %d\n", max_cost);
//}

//void experience::updatePriorityMatrix(CARPInd sol)
//{
//    int i;
//    int first, next;
//    for (i=1; i<sol.Sequence[0]; i++)
//    {
//        first = sol.Sequence[i];
//        next = sol.Sequence[i+1];
//        this->priorityMatrix[first][next] = 1.0*(this->pairNum[first][next] *
//                this->priorityMatrix[first][next] + 1.0*this->LB/sol
//                .TotalCost) / (this->pairNum[first][next] + 1);
//        this->pairNum[first][next] += 1;
//    }
//}

//void experience::saveMatrix()
//{
//    char path[100];
//    memset(path, 0, sizeof(path));
//    sprintf(path, "cost-change-data/%s.txt", map);
//
//    FILE *fp;
//    fp = fopen(path, "w");
//
//    int i,j;
//    for (i=0; i<=this->total_task_num; i++)
//    {
//        for (j=0; j<=this->total_task_num; j++)
//        {
//            fprintf(fp, "%.4f ", this->costMatrix[i][j]);
//        }
//        fprintf(fp, "\n");
//    }
//    fprintf(fp, "\n");
//    for (i=0; i<=this->total_task_num; i++)
//    {
//        for (j=0; j<=this->total_task_num; j++)
//        {
//            fprintf(fp, "%.4f ", this->priorityMatrix[i][j]);
//        }
//        fprintf(fp, "\n");
//    }
//    fclose(fp);
//}

//void experience::loadTransformMatrix()
//{
//    char path[100];
//    memset(path, 0, sizeof(path));
//    sprintf(path, "cost-change-data/k=%d/%s/instance/ins-transform%d.txt", (int)
//            cost_change_ratio, map, instance_id);
//
//    FILE *fp;
//    fp = fopen(path, "r");
//
//    int i,j;
//    for (i=0; i<=this->total_task_num; i++)
//    {
//        for (j=0; j<=this->total_task_num; j++)
//        {
//            fscanf(fp, "%lf", &this->transformMatrix[i][j]);
//        }
//    }
//    fclose(fp);
//}

void experience::save_best_so_far(CARPInd sol)
{
    this->queue_index ++;
    int index = (this->queue_index-1)%10;
    for (int i=0; i<=sol.Sequence[0]; i++)
    {
        this->best_so_far[index][i] = sol.Sequence[i];
    }
}

void solution_to_state_data(const int *sol_seq, const Task *inst_tasks, int *data_num, int (*data)[5])
{
    int current_load, next_demand, dis_task, dis_depot, dis_task_depot;
    int current_node, next_node;
    int j;
    for (j=1; j<sol_seq[0]; j++)
    {
        (*data_num) ++;
        if  (sol_seq[j] == 0)
        {
            current_load = 0;
        }
        next_demand = inst_tasks[sol_seq[j+1]].demand;
        current_node = inst_tasks[sol_seq[j]].tail_node;
        next_node = inst_tasks[sol_seq[j+1]].head_node;
        dis_task = min_cost[current_node][next_node];
        dis_depot = min_cost[current_node][DEPOT];
        dis_task_depot = min_cost[inst_tasks[sol_seq[j+1]].tail_node][DEPOT];

        data[*data_num][0] = current_load;
        data[*data_num][1] = next_demand;
        data[*data_num][2] = dis_task;
        data[*data_num][3] = dis_depot;
        data[*data_num][4] = dis_task_depot;

        current_load += inst_tasks[sol_seq[j+1]].demand;
    }
}

void experience::process_train_data(const Task *inst_tasks)
{
    int i, j;
//    int current_load, next_demand, dis_task, dis_depot, dis_task_depot;
//    int current_node, next_node;

    int data_num = -1;
    for (i=0; i<10; i++)
    {
        solution_to_state_data(this->best_so_far[i], inst_tasks, &data_num, this->data);
    }
    if(data_num > 4000)
    {
        printf("the memory is not enough for save training data\n");
        exit(-1);
    }

    char path[100];
    memset(path, 0, sizeof(path));
    sprintf(path, "cost-change-data/%s-fived.txt", map);

    FILE *fp;
    fp = fopen(path, "w");
    fprintf(fp, "cap,%d,cost,%d,num,%d\n", capacity, this->costScale, data_num);
    for (i=0; i<=data_num; i++)
    {
        fprintf(fp, "%d,%d,%d,%d,%d\n", this->data[i][0], this->data[i][1], this->data[i][2], this->data[i][3], this->data[i][4]);
    }
    fclose(fp);
}

one_class_model::one_class_model(int from_type) {
    if (from_type)
    {
        this->train_model();
    } else{
        this->load_model();
    }
}

void one_class_model::train_model() {
    char path[100];
    memset(path, 0, sizeof(path));
    sprintf(path, "cost-change-data/%s_process.txt", map);

    FILE *fp;
    fp = fopen(path, "r");

    int data_num, train_num, test_num;
    char dummy[101];
    double att1, att2, att3, att4, att5;

    fscanf(fp, "num,%d\n", &data_num);
    printf("data_num:%d\n", data_num);
    fscanf(fp, "train,%d\n", &train_num);

    svm_problem prob{};
    svm_parameter param{};

    prob.l = train_num;
    prob.x = new svm_node*[train_num];
    prob.y = new double[train_num];

    int i;
    for (i=0; i<train_num; i++)
    {
        prob.x[i] = new svm_node[5+1];
        fscanf(fp, "%lf,%lf,%lf,%lf,%lf\n", &att1, &att2, &att3, &att4, &att5);
        prob.x[i][0].index = 1;
        prob.x[i][0].value = att1;
        prob.x[i][1].index = 2;
        prob.x[i][1].value = att2;
        prob.x[i][2].index = 3;
        prob.x[i][2].value = att3;
        prob.x[i][3].index = 4;
        prob.x[i][3].value = att4;
        prob.x[i][4].index = 5;
        prob.x[i][4].value = att5;
        prob.x[i][5].index = -1;
        prob.y[i] = 1;
    }
    fscanf(fp, "test,%d\n", &test_num);

    struct svm_node **test_data;
    test_data = new svm_node*[test_num];
    for(i=0; i<test_num; i++)
    {
        test_data[i] = new svm_node[5+1];
        fscanf(fp, "%lf,%lf,%lf,%lf,%lf\n", &att1, &att2, &att3, &att4, &att5);
        test_data[i][0].index = 1;
        test_data[i][0].value = att1;
        test_data[i][1].index = 2;
        test_data[i][1].value = att2;
        test_data[i][2].index = 3;
        test_data[i][2].value = att3;
        test_data[i][3].index = 4;
        test_data[i][3].value = att4;
        test_data[i][4].index = 5;
        test_data[i][4].value = att5;
        test_data[i][5].index = -1;
    }
    fclose(fp);

    param.svm_type = ONE_CLASS;
    param.kernel_type = RBF;
    param.degree = 3;
    param.gamma = 2;	// 1/num_features
    param.coef0 = 0;
    param.nu = 0.001;
    param.cache_size = 100;
    param.C = 1;
    param.eps = 1e-3;
    param.p = 0.1;
    param.shrinking = 1;
    param.probability = 1;
    param.nr_weight = 0;
    param.weight_label = nullptr;
    param.weight = nullptr;

    this->model = svm_train(&prob, &param);

    memset(path, 0, sizeof(path));
    sprintf(path, "cost-change-data/%s.model", map);
    if (svm_save_model(path, model)) {
        printf("Save One-Class SVM to %s FAILED \n", path);
    } else {
        printf("Save One-Class SVM to %s SUCCEED \n", path);
    }

    free(prob.x);
    free(prob.y);
    free(test_data);
    svm_destroy_param(&param);
}

void one_class_model::load_model() {
    char path[100];
    memset(path, 0, sizeof(path));
    sprintf(path, "cost-change-data/%s.model", map);

    this->model = svm_load_model(path);
    if(this->model == nullptr)
    {
        printf("Load One-Class SVM to %s FAILED \n", path);
        exit(-1);
    }
}

void one_class_model::predict_file_data()
{
    char path[100];
    memset(path, 0, sizeof(path));
    sprintf(path, "cost-change-data/%s-test-process.txt", map);

    FILE *fp;
    fp = fopen(path, "r");

    int test_num;
    char dummy[101];
    double att1, att2, att3, att4, att5;

    fscanf(fp, "num,%d\n", &test_num);
    printf("data_num:%d\n", test_num);

    struct svm_node **test_data;
    test_data = new svm_node*[test_num];
    for(int i=0; i<test_num; i++)
    {
        test_data[i] = new svm_node[5+1];
        fscanf(fp, "%lf,%lf,%lf,%lf,%lf\n", &att1, &att2, &att3, &att4, &att5);
        test_data[i][0].index = 1;
        test_data[i][0].value = att1;
        test_data[i][1].index = 2;
        test_data[i][1].value = att2;
        test_data[i][2].index = 3;
        test_data[i][2].value = att3;
        test_data[i][3].index = 4;
        test_data[i][3].value = att4;
        test_data[i][4].index = 5;
        test_data[i][4].value = att5;
        test_data[i][5].index = -1;
    }
    fclose(fp);
}

void one_class_model::construct(const Task *inst_tasks) {
    int final_task = inst_tasks[1].inverse - 1;
    int total_task_num = inst_tasks[inst_tasks[1].inverse - 1].inverse;

    int current_load, next_demand, dis_task, dis_depot, dis_task_depot;
    int current_node, next_node;
    int trail=0;
    double max_prob = 0;
    int i,j;
    int used[MAX_TASK_SEG_LENGTH], fsb_task[MAX_TASK_TAG_LENGTH], candi_task[MAX_TASK_TAG_LENGTH];
    int sol_seq[MAX_TASK_SEQ_LENGTH];
    memset(used, 0, sizeof(used));
    memset(candi_task, 0, sizeof(candi_task));
    memset(fsb_task, 0, sizeof(fsb_task));
    memset(sol_seq, 0, sizeof(sol_seq));

    double attr[5], label, probab[2];

    sol_seq[0] = 1;
    sol_seq[1] = 0;
//    for (i=final_task; ; i--)
    i = final_task+1;
    while(trail < final_task)
    {
        i--;
        if (i<=0)
        {
            printf("invalid i\n");
            exit(-1);
        }

        if (inst_tasks[i].vt == 0)
        {
            current_load = 0;
            current_node = DEPOT;
            printf("vehicles start from the depot\n");
        }

        if (inst_tasks[i].vt > 0)
        {
            current_load = inst_tasks[i].demand;
            current_node = inst_tasks[i].tail_node;
            sol_seq[0]++;
            sol_seq[sol_seq[0]] = i;
            trail ++;
        }

        attr[0] = current_load*1.0/capacity;
        dis_depot = min_cost[current_node][DEPOT];
        attr[3] = dis_depot*1.0/task_max_cost;
        while (true)
        {
            candi_task[0] = 0;
            fsb_task[0] = 0;
            max_prob = 0;
            for (j=1; j<=total_task_num; j++)
            {
                if (inst_tasks[j].vt > 0 || used[j])
                {
                    continue;
                }
                next_demand = inst_tasks[j].demand;
                attr[1] = next_demand*1.0/capacity;
                if (current_load + next_demand > capacity)
                {
                    continue;
                }
                fsb_task[0] ++;
                fsb_task[fsb_task[0]] = j;

                next_node = inst_tasks[j].head_node;
                dis_task = min_cost[current_node][next_node];
                dis_task_depot = min_cost[inst_tasks[j].tail_node][DEPOT];
                attr[2] = dis_task*1.0/task_max_cost;
                attr[4] = dis_task_depot*1.0/task_max_cost;
                label = this->predict_one(attr, probab);
                printf("%d, %f, %f, %f, %f, %f, label:%f, prob:%f\n", j, attr[0], attr[1], attr[2], attr[3], attr[4], label, probab[0]);
                if(label > 0)
                {
                    if (probab[0] > max_prob)
                    {
                        max_prob = probab[0];
                        candi_task[0] = 1;
                        candi_task[1] = j;
                    } else if (probab[0] == max_prob)
                    {
                        candi_task[0]++;
                        candi_task[candi_task[0]] = j;
                    }
                }
            }
            if (fsb_task[0] == 0)
            { // It has no task satisfying the capacity constraints.
                sol_seq[0]++;
                sol_seq[sol_seq[0]] = 0;
                break;
            }

            if (candi_task[0] == 1) // has only one task with maximum probability
            {
                sol_seq[0]++;
                sol_seq[sol_seq[0]] = candi_task[1];
            } else if (candi_task[0] > 1) // has several tasks with same maximum probability
            { // randomly select one task from these tasks
                int k = rand_choose(candi_task[0]);
                sol_seq[0]++;
                sol_seq[sol_seq[0]] = candi_task[k];
            } else if (candi_task[0] == 0) // all tasks are predicted as negative label
            { // select the nearest task from all feasible tasks.
                sol_seq[0]++;
                int min_dist = INF;
                for (int k=1; k<=fsb_task[0]; k++)
                {
                    if (min_cost[current_node][inst_tasks[fsb_task[k]].head_node] < min_dist)
                    {
                        min_dist = min_cost[current_node][inst_tasks[fsb_task[k]].head_node];
                        sol_seq[sol_seq[0]] = fsb_task[k];
                    }
                }
//                printf("all tasks are predicted as negative label\n");
            }
            trail ++;
            current_load += inst_tasks[sol_seq[sol_seq[0]]].demand;
//            printf("next_task: %d\n", sol_seq[sol_seq[0]]);
            used[sol_seq[sol_seq[0]]] = 1;
            used[inst_tasks[sol_seq[sol_seq[0]]].inverse] = 1;
        }
    }

    CARPInd sol;
    memcpy(sol.Sequence, sol_seq, sizeof(sol_seq));
    int total_cost = get_task_seq_total_cost(sol.Sequence, inst_tasks);
    sol.TotalCost = 0;
    sol.Loads[0] = -1;
    current_load = 0;
    for (i=1; i<=sol.Sequence[0]; i++)
    {
        printf("%d ", sol.Sequence[i]);
        if (sol.Sequence[i] == 0)
        {
            sol.Loads[0]++;
            sol.Loads[sol.Loads[0]] = current_load;
            current_load = 0;
            continue;
        }
        current_load += inst_tasks[sol.Sequence[i]].demand;
    }

    // following are Frederick Heuristic
    int Route[MAX_TASK_SEQ_LENGTH], FHRoute[MAX_TASK_SEQ_LENGTH], Position[101], TmpSeq[MAX_TASK_SEQ_LENGTH], Cost1, Cost2;;
    find_ele_positions(Position, sol.Sequence, 0);
    for(i = 1; i < Position[0]; i++)
    {
        AssignSubArray(sol.Sequence, Position[i], Position[i+1], Route);
        FredericksonHeuristic(FHRoute, Route, inst_tasks);
        Cost1 =get_task_seq_total_cost(Route, inst_tasks);
        Cost2 = get_task_seq_total_cost(FHRoute, inst_tasks);

        TmpSeq[0] --;
        if (Cost1 < Cost2)
        {
            JoinArray(TmpSeq, Route); //link two routes
            sol.TotalCost += Cost1;
        } else {
            JoinArray(TmpSeq, FHRoute);
            sol.TotalCost += Cost2;
        }
    }
    AssignArray(TmpSeq, sol.Sequence);

    int total_cost1 = get_task_seq_total_cost(sol.Sequence, inst_tasks);
    printf("\ntotal_cost:%d %d %d\n", total_cost, total_cost1, sol.TotalCost);
}

double one_class_model::predict_one(const double *attr, double *probab)
{
    svm_node test_x[6];
    double label;
    test_x[0].index = 1;
    test_x[0].value = attr[0];
    test_x[1].index = 2;
    test_x[1].value = attr[1];
    test_x[2].index = 3;
    test_x[2].value = attr[2];
    test_x[3].index = 4;
    test_x[3].value = attr[3];
    test_x[4].index = 5;
    test_x[4].value = attr[4];
    test_x[5].index = -1;
    label = svm_predict_probability(this->model, test_x, probab);
    return label;
}

one_class_model::~one_class_model() {
    svm_free_and_destroy_model(&this->model);
}

knn_model::knn_model() {
    char path[100];
    memset(path, 0, sizeof(path));
    sprintf(path, "cost-change-data/%s-process.txt", map);

    FILE *fp;
    fp = fopen(path, "r");

    double att1, att2, att3, att4, att5;

    fscanf(fp, "num,%d\n", &this->data_num);
    printf("data_num:%d\n", this->data_num);

    this->data = new data_node[4000];

    int i;
    for (i=0; i<this->data_num; i++)
    {
        fscanf(fp, "%lf,%lf,%lf,%lf,%lf\n", &att1, &att2, &att3, &att4, &att5);
        this->data[i].x[0] = att1;
        this->data[i].x[1] = att4;
        this->data[i].y[0] = att2;
        this->data[i].y[1] = att3;
        this->data[i].y[2] = att5;
    }
    fclose(fp);
}

void knn_model::construct(const Task *inst_tasks, CARPInd *init_sol) {

    int final_task = inst_tasks[1].inverse - 1;
    int total_task_num = inst_tasks[inst_tasks[1].inverse - 1].inverse;

    int current_load, next_demand, dis_task, dis_depot, dis_task_depot;
    int current_node, next_node;
    int trail=0;

    int i,j;
    int used[MAX_TASK_SEG_LENGTH], fsb_task[MAX_TASK_TAG_LENGTH], candi_task[MAX_TASK_TAG_LENGTH];
    int sol_seq[MAX_TASK_SEQ_LENGTH];
    memset(used, 0, sizeof(used));
    memset(candi_task, 0, sizeof(candi_task));
    memset(fsb_task, 0, sizeof(fsb_task));
    memset(sol_seq, 0, sizeof(sol_seq));

    double attr[5], similarity;
    double min_dis = 0;
    int nearest_k = 1;
    int nearest[nearest_k];

    sol_seq[0] = 1;
    sol_seq[1] = 0;
//    for (i=final_task; ; i--)
    i = final_task+1;
    while(trail < final_task)
    {
        i--;
        if (i<=0)
        {
            printf("invalid i\n");
            exit(-1);
        }

        if (inst_tasks[i].vt == 0)
        {
            current_load = 0;
            current_node = DEPOT;
            printf("vehicles start from the depot\n");
        }

        if (inst_tasks[i].vt > 0)
        {
            current_load = inst_tasks[i].demand;
            current_node = inst_tasks[i].tail_node;
            sol_seq[0]++;
            sol_seq[sol_seq[0]] = i;
            trail ++;
        }


        dis_depot = min_cost[current_node][DEPOT];
        attr[3] = dis_depot*1.0/task_max_cost;

        // find the nearest state in the database

        
        while (true)
        {
            candi_task[0] = 0;
            fsb_task[0] = 0;
            min_dis = 10;
            attr[0] = current_load*1.0/capacity;
            this->find_the_nearest_state(nearest_k, nearest, attr[0], attr[3]);
            for (j=0; j<=total_task_num; j++)
            {
                if (inst_tasks[j].vt > 0 || used[j])
                {
                    continue;
                }
                next_demand = inst_tasks[j].demand;
                attr[1] = next_demand*1.0/capacity;
                if (current_load + next_demand > capacity)
                {
                    continue;
                }
                fsb_task[0] ++;
                fsb_task[fsb_task[0]] = j;

                next_node = inst_tasks[j].head_node;
                dis_task = min_cost[current_node][next_node];
                dis_task_depot = min_cost[inst_tasks[j].tail_node][DEPOT];
                attr[2] = dis_task*1.0/task_max_cost;
                attr[4] = dis_task_depot*1.0/task_max_cost;
                // calculate the similarity of candidate tasks.
                similarity = this->calculate_the_similarity(nearest, nearest_k, attr);
//                printf("%d, %f, %f, %f, %f, %f, similarity:%f \n", j, attr[0], attr[1], attr[2], attr[3], attr[4], similarity);
                if (similarity < min_dis)
                {
                    min_dis = similarity;
                    candi_task[0] = 1;
                    candi_task[1] = j;
                } else if (similarity == min_dis)
                {
                    candi_task[0]++;
                    candi_task[candi_task[0]] = j;
                }
            }
            if (fsb_task[0] == 0)
            { // It has no task satisfying the capacity constraints.
                sol_seq[0]++;
                sol_seq[sol_seq[0]] = 0;
                printf("route completed\n");
                break;
            }

            if (candi_task[0] == 1) // has only one task with maximum similarity
            {
                sol_seq[0]++;
                sol_seq[sol_seq[0]] = candi_task[1];
                printf("next task: %d\n", candi_task[1]);
            } else if (candi_task[0] > 1) // has several tasks with same maximum similarity
            { // randomly select one task from these tasks
                int k = rand_choose(candi_task[0]);
                sol_seq[0]++;
                sol_seq[sol_seq[0]] = candi_task[k];
            } else if (candi_task[0] == 0) // candi_task can not be empty
            {
                printf("it has some bugs\n");
                exit(-1);
            }
            if (sol_seq[sol_seq[0]] != 0)
            {
                trail ++;
                current_load += inst_tasks[sol_seq[sol_seq[0]]].demand;
                used[sol_seq[sol_seq[0]]] = 1;
                used[inst_tasks[sol_seq[sol_seq[0]]].inverse] = 1;
            }else{
                break;
            }
        }
    }

    CARPInd sol;
    memcpy(sol.Sequence, sol_seq, sizeof(sol_seq));
    int total_cost = get_task_seq_total_cost(sol.Sequence, inst_tasks);
    sol.TotalCost = 0;
    sol.Loads[0] = -1;
    current_load = 0;
    for (i=1; i<=sol.Sequence[0]; i++)
    {
        printf("%d ", sol.Sequence[i]);
        if (sol.Sequence[i] == 0)
        {
            sol.Loads[0]++;
            sol.Loads[sol.Loads[0]] = current_load;
            current_load = 0;
            continue;
        }
        current_load += inst_tasks[sol.Sequence[i]].demand;
    }

//    // following are Frederick Heuristic
//    int Route[MAX_TASK_SEQ_LENGTH], FHRoute[MAX_TASK_SEQ_LENGTH], Position[101], TmpSeq[MAX_TASK_SEQ_LENGTH], Cost1, Cost2;
//    TmpSeq[0] = 1;
//    find_ele_positions(Position, sol.Sequence, 0);
//    for(i = 1; i < Position[0]; i++)
//    {
//        AssignSubArray(sol.Sequence, Position[i], Position[i+1], Route);
//        FredericksonHeuristic(FHRoute, Route, inst_tasks);
//        Cost1 =get_task_seq_total_cost(Route, inst_tasks);
//        Cost2 = get_task_seq_total_cost(FHRoute, inst_tasks);
//
//        TmpSeq[0] --;
//        if (Cost1 < Cost2)
//        {
//            JoinArray(TmpSeq, Route); //link two routes
//            sol.TotalCost += Cost1;
//        } else {
//            JoinArray(TmpSeq, FHRoute);
//            sol.TotalCost += Cost2;
//        }
//    }
//    memset(Position, 0, sizeof(Position));
//    for (i=1; i<=TmpSeq[0]; i++)
//    {
//        if (inst_tasks[TmpSeq[i]].vt > 0 && TmpSeq[i-1]!=0)
//        {
//            Position[0]++;
//            Position[Position[0]] = i;
//        }
//    }
//    for (i=1; i<=Position[i]; i++)
//    {
//        add_element(TmpSeq, 0, Position[i]+i-1);
//    }
//    AssignArray(TmpSeq, sol.Sequence);

    int total_cost1 = get_task_seq_total_cost(sol.Sequence, inst_tasks);
    printf("\ntotal_cost:%d %d %d\n", total_cost, total_cost1, sol.TotalCost);

    copy_individual(init_sol, &sol);
}

void knn_model::find_the_nearest_state(int k, int *index, double att1, double att2)
{
    int i, j;
    struct diff{
        double value;
        int idx;
    };

    diff nodes[this->data_num];
    double tmp0;
    for (i=0; i<this->data_num; i++)
    {
        tmp0 = pow(this->data[i].x[0] - att1, 2);
        tmp0 += pow(this->data[i].x[1] - att2, 2);

        nodes[i].value = sqrt(tmp0);
        nodes[i].idx = i;
    }

    double min_value;
    int min_idx;
    diff tmp{0.0,0};

    for (i=0; i < k; i++)
    {
        min_value = nodes[i].value;
        min_idx = i;
        for (j=i+1; j<this->data_num; j++)
        {
            if (nodes[j].value < min_value)
            {
                min_value = nodes[j].value;
                min_idx = j;
            }
        }
        tmp = nodes[min_idx];
        nodes[min_idx] = nodes[i];
        nodes[i] = tmp;
    }
    for (i=0; i<k; i++)
    {
        index[i] = nodes[i].idx;
    }
}

double knn_model::calculate_the_similarity(const int *index, int k, const double *atts) const
{
    double sim=0;
    double tmp;

    for (int i=0; i<k; i++)
    {
        // printf("xxxxx: %f, %f, %f\n", this->data[index[i]].y[0], this->data[index[i]].y[1], this->data[index[i]].y[2]);
        tmp = 0.1*pow(this->data[index[i]].y[0] - atts[1], 2);
        tmp += 0.6*pow(this->data[index[i]].y[1] - atts[2], 2);
        tmp += 0.3*pow(this->data[index[i]].y[2] - atts[4], 2);
        sim += sqrt(tmp);
    }
    
    double avr_sim = sim/k;
    return avr_sim;
}


knn_model::~knn_model() {
    free(this->data);
}