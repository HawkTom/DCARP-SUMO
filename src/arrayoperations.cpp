#include "src.h"

void find_ele_positions(int *positions, int *a, int e)
{
    positions[0] = 0;
    for (int i = 1; i <= a[0]; i++)
    {
        if (a[i] == e)
        {
            positions[0] ++;
            positions[positions[0]] = i;
        }
    }
}

void delete_element(int *a, int k)
{
    if (k < 1 || k > a[0])
    {
        printf("\n a[0]: %d, k: %d \n", a[0], k);
        printf("the deleting position is wrong! Jump.\n");
        // longjmp(buf, 2);
        exit(0);
    }

    for (int i = k; i < a[0]; i++)
    {
        a[i] = a[i+1];
    }
    a[a[0]] = 0;
    a[0] --;
}

void add_element(int *a, int e, int k)
// add element e in k position of a
{
    if (k < 1 || k > a[0]+1)
    {
        printf("the inserting position is wrong!\n");
        // longjmp(buf, 2);
        exit(0);
    }

    a[0] ++;
    for (int i = a[0]; i > k; i--)
    {
        a[i] = a[i-1];
    }
    a[k] = e;
}

int rand_choose(int num)
{
    int x = rand();
    int k = x%num;

    k++;
    // printf("%d %d %d\n", x, num, k);

    return k;
}


void rand_perm(int *a, int num)
{
    // int *a = (int *)malloc((num+1)*sizeof(int));
    int *left_ele = (int *)malloc((num+1)*sizeof(int));
    left_ele[0] = num;
    for (int i = 1; i <= num; i++)
    {
        left_ele[i] = i;
    }

    a[0] = num;
    for (int i = 1; i <= num; i++)
    {
        int k = rand_choose(left_ele[0]);
        a[i] = left_ele[k];
        delete_element(left_ele, k);
    }

    free(left_ele);
}

void rand_shuffle(int *a)
{
    // int *a = (int *)malloc((num+1)*sizeof(int));
    int *left_ele = (int *)malloc((a[0]+1)*sizeof(int));
    int *b = (int *)malloc((a[0]+1)*sizeof(int));
    left_ele[0] = a[0];
    b[0] = a[0];
    for (int i = 1; i <= a[0]; i++)
    {
        left_ele[i] = i;
        b[i] = a[i];
    }

    for (int i = 1; i <= a[0]; i++)
    {
        int k = rand_choose(left_ele[0]);
        a[i] = b[left_ele[k]];
        delete_element(left_ele, k);
    }
    free(left_ele);
    free(b);
}

void rand_selection(int *id1, int *id2, int popsize)
/* pop has been sorted increasingly already */
{
    int k1, k2;
    int candi[popsize+1];
    candi[0] = popsize;
    for (int i = 1; i <= popsize; i++)
    {
        candi[i] = i-1;
    }

    k1 = rand_choose(candi[0]);
    *id1 = candi[k1];
    delete_element(candi, k1);
    k2 = rand_choose(candi[0]);
    *id2 = candi[k2];
}

int max(int *Array)
{
    int Length = Array[0];
    int i, maximum;

    maximum = 0;
    for (i = 1; i <= Length; i++)
    {
        if (Array[i] > maximum)
        {
            maximum = Array[i];
        }
    }

    return maximum;
}

int find_min(int *Array)
{
    int i, mininum;
    mininum = INF;
    int min_pos;
    for (i = 1; i <= Array[0]; i++)
    {
        if (Array[i] < mininum)
        {
            mininum = Array[i];
            min_pos = i;
        }
    }
    return min_pos;
}

void AssignArray(int *Array1, int *Array2)
// assign array1 to array2
{
    int i;

    for (i = 0; i <= Array1[0]; i++)
    {
        Array2[i] = Array1[i];
    }
}

void AssignSubArray(int *Array1, int k1, int k2, int *Array2)
// assign array1[k1:k2] to array2
{
    int i;

    Array2[0] = k2-k1+1;

    for (i = k1; i <= k2; i++)
    {
        Array2[i-k1+1] = Array1[i];
    }
}

void JoinArray(int *JointArray, int *Array)
{
    int i;
    for (i = 1; i <= Array[0]; i++)
    {
        JointArray[0] ++;
        JointArray[JointArray[0]] = Array[i];
    }
}

void ReverseDirection(int *Array, int k1, int k2)
// reverse the subarray of Array[1:n] from the position k1 to position k2
{
    int i, k, tmp;
    double m = (k2-k1+1)/2;
    k = (int)m;

    for (i = k1; i < k1+k; i++)
    {
        tmp = Array[i];
        Array[i] = Array[k1+k2-i];
        Array[k1+k2-i] = tmp;
    }
}

int get_total_vio_load(int *route_seg_load)
{
    int total_vio_load = 0;
    for (int i = 1; i <= route_seg_load[0]; i++)
    {
        if (route_seg_load[i] > capacity)
            total_vio_load += route_seg_load[i]-capacity;
    }

    return total_vio_load;
}

int FindTask(int a, int b, const Task *inst_tasks, int NO_Task)
{
    int i;

    for (i = 1; i <= NO_Task; i++)
    {
        if (inst_tasks[i].head_node_r == a && inst_tasks[i].tail_node_r == b)
            return i;
    }

    return 0;
}

// the following functions are copied from another file
void indi_route_converter(CARPInd *dst, CARPInd *src, const Task *inst_tasks)
{
    int i, load;
    load = 0;
    memset(dst->Sequence, 0, sizeof(int) * MAX_TASK_SEQ_LENGTH);
    memset(dst->Loads, 0, sizeof(int) * 50);
    dst->Sequence[0] = 1;
    dst->Sequence[1] = 0;
    for (i = 2; i <= src->Sequence[0]; i++)
    {
        if (src->Sequence[i] == 0)
        {
            dst->Sequence[0] ++;
            dst->Sequence[dst->Sequence[0]] = 0;
            dst->Loads[0] ++;
            dst->Loads[dst->Loads[0]] = load;
            load = 0;
            continue;
        }
        if (inst_tasks[src->Sequence[i]].vt > 0 && src->Sequence[i-1] != 0)
        {
            dst->Sequence[0] ++;
            dst->Sequence[dst->Sequence[0]] = 0;
            dst->Loads[0] ++;
            dst->Loads[dst->Loads[0]] = load;
            load = 0;
        }

        load += inst_tasks[src->Sequence[i]].demand;
        dst->Sequence[0] ++;
        dst->Sequence[dst->Sequence[0]] = src->Sequence[i];
    }
}

int get_task_seq_total_cost(int *task_seq, const Task *inst_tasks)
{
    int total_cost =  0;
    for (int i = 1; i < task_seq[0]; i++)
    {
        total_cost += min_cost[inst_tasks[task_seq[i]].tail_node][inst_tasks[task_seq[i+1]].head_node]+inst_tasks[task_seq[i]].serv_cost;
        // printf("(%d, %d, %d) \t", inst_tasks[task_seq[i]].tail_node, inst_tasks[task_seq[i+1]].head_node, min_cost[inst_tasks[task_seq[i]].tail_node][inst_tasks[task_seq[i+1]].head_node]);
    }
    // printf("\n");
    return total_cost;
}

void get_task_seq_loads(int *loads, const int *task_seq, const Task *inst_tasks)
{
    loads[0] = 0;
    int i, curr_load = 0;
    for (i=2; i<=task_seq[0]; i++)
    {
        if (task_seq[i] == 0)
        {
            loads[0] ++;
            loads[loads[0]] = curr_load;
            curr_load = 0;
            continue;
        }
        curr_load += inst_tasks[task_seq[i]].demand;
    }
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

void copy_individual(CARPInd *dest, CARPInd *src)
{
    memcpy(dest->Sequence, src->Sequence, sizeof(src->Sequence));
    memcpy(dest->Loads, src->Loads, sizeof(src->Loads));
    dest->TotalCost = src->TotalCost;
    dest->TotalVioLoad = src->TotalVioLoad;
    dest->Fitness = src->Fitness;
}