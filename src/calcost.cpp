#include "src.h"
#include "string.h"

void mod_dijkstra()
{
    memset(min_cost, 0, sizeof(min_cost));
    memset(shortest_path, 0, sizeof(shortest_path));
    int i, j, k, m, minimum;
//    printf("Dijastra\n");
    for (i = 1; i <= vertex_num; i++)
    {
        for (j = 1; j <= vertex_num; j++)
        {
            if (j == i)
                continue;

            shortest_path[i][j][0] = 1;
            shortest_path[i][j][1] = i;
            min_cost[i][j] = INF;
        }
    }

    int mark[MAX_NODE_TAG_LENGTH], dist[MAX_NODE_TAG_LENGTH], dist1[MAX_NODE_TAG_LENGTH], nearest_neighbor[MAX_NODE_TAG_LENGTH];

    for (i = 1; i <= vertex_num; i++)
    {
        mark[i] = 1;

        for (j = 1; j <= vertex_num; j++)
        {
            if (j == i)
                continue;

            mark[j] = 0;
            dist[j] = trav_cost[i][j];
            dist1[j] = dist[j];
        }

        for (k = 1; k < vertex_num; k++)
        {
            minimum = INF;
            nearest_neighbor[0] = 0;

            for (j = 1; j <= vertex_num; j++)
            {
                if (mark[j])
                    continue;

                if (dist1[j] == INF)
                    continue;

                if (dist1[j] < minimum)
                    minimum = dist1[j];
            }

            if (minimum == INF)
                continue;

            for (j = 1; j <= vertex_num; j++)
            {
                if (mark[j])
                    continue;

                if (dist1[j] == minimum)
                {
                    nearest_neighbor[0] ++;
                    nearest_neighbor[nearest_neighbor[0]] = j;
                }
            }

            int v = nearest_neighbor[1];
            dist1[v] = INF;
            mark[v] = 1;

            if (shortest_path[i][v][0] == 0 || (shortest_path[i][v][0] > 0 && shortest_path[i][v][shortest_path[i][v][0]] != v))
            {
                shortest_path[i][v][0] ++;
                shortest_path[i][v][shortest_path[i][v][0]] = v;
            }

            for (j = 1; j <= vertex_num; j++)
            {
                if (mark[j])
                    continue;

                if (minimum+trav_cost[v][j] < dist[j])
                {
                    dist[j] = minimum+trav_cost[v][j];
                    dist1[j] = minimum+trav_cost[v][j];
                    for (m = 0; m <= shortest_path[i][v][0]; m++)
                    {
                        shortest_path[i][j][m] = shortest_path[i][v][m];
                    }
                }
            }

            for (j = 1; j <= vertex_num; j++)
            {
                if (j == i)
                    continue;

                min_cost[i][j] = dist[j];
            }
        }
    }

    for (i = 1; i <= vertex_num; i++)
    {
        for (j = 1; j <= vertex_num; j++)
        {
            if (shortest_path[i][j][0] == 1)
                shortest_path[i][j][0] = 0;
        }
    }

    for (i = 1; i <= vertex_num; i++)
    {
        shortest_path[i][i][0] = 1;
        shortest_path[i][i][1] = i;
        min_cost[i][i] = 0;
    }
}


void update_cost(const Task *inst_tasks, const Arc *inst_arcs)
{

    memset(trav_cost, 0, sizeof(trav_cost));
    // update trav_cost, serve_cost, min_cost after each dynamic change
    for (int i=1; i<=vertex_num; i++)
    {
        for (int j=1; j<=vertex_num; j++)
        {
            trav_cost[i][j] = INF;
            serve_cost[i][j] = 0;
        }
    }

    trav_cost[1][0] = 0;
    trav_cost[0][1] = 0;

    for (int i=1; i<=task_num; i++)
    {
        serve_cost[inst_tasks[i].head_node][inst_tasks[i].tail_node] = inst_tasks[i].serv_cost;
    }

    for (int i=1; i<=total_arc_num; i++)
    {
        // printf("%d, %d: %d\n", inst_arcs[i].head_node, inst_arcs[i].tail_node, inst_arcs[i].trav_cost);
        trav_cost[inst_arcs[i].head_node][inst_arcs[i].tail_node] = inst_arcs[i].trav_cost;
    }
//    printf("\n");
}

void get_cost_scale(const Task *inst_tasks)
{
    // update max_task_cost: should be the maximum cost in min_costs
    int i, j;
    task_max_cost = 0;
    for (i = 1; i <= vertex_num; i++ )
    {
        for (j=1; j <= vertex_num; j++)
        {
            if (task_max_cost < min_cost[i][j])
            {
                task_max_cost = min_cost[i][j];
            }
        }
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

int get_route_loads(const int *task_seq, const Task *inst_tasks)
{
    int load = 0;
    int i;
    for (i=2; i<task_seq[0]; i++)
    {
        load += inst_tasks[task_seq[i]].demand;
    }
    return load;
}

int get_additional_cost(Vehicles state)
{
    int i, add_cost = 0;
    for (i = 1; i <= state.stop[0]; i++)
    {
        add_cost += min_cost[state.stop[i]][DEPOT];
    }

    return add_cost;
}