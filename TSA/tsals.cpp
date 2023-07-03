//
// Created by hao on 13/06/2020.
//

# include "TSA.h"

int SingleInsertion(CARPInd *BestSINeighbor, CARPInd *CurrSolution, int *MovedTasks, int *ChangedRoutesID, const Task *inst_tasks, int (*TabuList)[101],
        double PenPrmt, double BestFitness, double BestFsbFitness )
{
    int i, j, k, u, v, z;

    int count = 0;  // record neighbourhood moves (number of fitness evaluation)
    int Besti, Bestj, Bestu, Bestv, RID1, RID2;
    int Positions[101], Route[101][MAX_TASK_SEQ_LENGTH];
    find_ele_positions(Positions, CurrSolution->Sequence, 0);

    Route[0][0] = Positions[0] - 1; // PA*
    for(i = 1; i < Positions[0]; i++)
    {
        AssignSubArray(CurrSolution->Sequence, Positions[i], Positions[i+1], Route[i]);
    }

    CARPInd Neighbor;

    MovedTasks[0] = 1;
    BestSINeighbor->Fitness = INF;
    Bestu = -1;

    for (i = 1; i < Positions[0]; i++)
    {
        RID1 = i;
        for (j = 2; j < Route[i][0]; j++) // Route[i]: 0, t1, t2, t3, ..., tn, 0
        {
            for (u = 0; u < Positions[0]; u++)
            {
                if (u == i)
                    continue; // insertion happens in two different routes

                RID2 = u;
                if (u == 0 && Route[0][0]<vehicle_num && Route[i][0] > 3)
                {
                    // assign a new route, the route number < max vehicle, over one tasks in Route[i];
                    // AssignArray(CurrSolution->Loads, Neighbor.Loads);
                    memcpy(Neighbor.Loads, CurrSolution->Loads, sizeof(CurrSolution->Loads));
                    Neighbor.Loads[i] -= inst_tasks[Route[i][j]].demand;
                    Neighbor.TotalVioLoad = CurrSolution->TotalVioLoad;

                    if (Neighbor.Loads[i] > capacity)
                    {
                        Neighbor.TotalVioLoad -= inst_tasks[Route[i][j]].demand;
                    } else if ( Neighbor.Loads[i] > capacity - inst_tasks[Route[i][j]].demand)
                    {
                        Neighbor.TotalVioLoad -= Neighbor.Loads[i] + inst_tasks[Route[i][j]].demand - capacity;
                    }

                    Neighbor.Loads[0] ++;
                    Neighbor.Loads[Neighbor.Loads[0]] = inst_tasks[Route[i][j]].demand;

                    if (Neighbor.Loads[Neighbor.Loads[0]] > capacity)
                    {
                        Neighbor.TotalVioLoad += Neighbor.Loads[Neighbor.Loads[0]] - capacity;
                    }
                    int tmp_vio_load = get_total_vio_load(Neighbor.Loads);

                    Neighbor.TotalCost = CurrSolution->TotalCost - min_cost[inst_tasks[Route[i][j-1]].tail_node][inst_tasks[Route[i][j]].head_node]
                                                - min_cost[inst_tasks[Route[i][j]].tail_node][inst_tasks[Route[i][j+1]].head_node]
                                                + min_cost[inst_tasks[Route[i][j-1]].tail_node][inst_tasks[Route[i][j+1]].head_node]
                                                + min_cost[DEPOT][inst_tasks[Route[i][j]].head_node]
                                                + min_cost[inst_tasks[Route[i][j]].tail_node][DEPOT];
                    count ++;
                    Neighbor.Fitness = Neighbor.TotalCost + PenPrmt * Neighbor.TotalVioLoad;

                    // check tabu list
                    if (TabuList[Route[i][j]][u] == 0 || (Neighbor.TotalVioLoad > 0 && Neighbor.Fitness < BestFitness)
                        || (Neighbor.TotalVioLoad == 0 && Neighbor.Fitness < BestFsbFitness))
                    {
                        if (Neighbor.Fitness < BestSINeighbor->Fitness)
                        {
                            MovedTasks[1] = Route[i][j];
                            Besti = i;
                            Bestj = j;
                            Bestu = u;
                            Bestv = 0;
                            ChangedRoutesID[0] = RID1;
                            ChangedRoutesID[1] = RID2;

                            BestSINeighbor->TotalCost = Neighbor.TotalCost;
                            // AssignArray(Neighbor.Loads, BestSINeighbor->Loads);
                            memcpy(BestSINeighbor->Loads, Neighbor.Loads, sizeof(CurrSolution->Loads));
                            BestSINeighbor->TotalVioLoad = Neighbor.TotalVioLoad;
                            BestSINeighbor->Fitness = Neighbor.Fitness;
                        }
                    }
                }
                if (u == 0)
                    continue;

                for (v = 2; v <= Route[u][0]; v++) // Route[i]: 0, t1, t2, t3, ..., tn, 0
                {
                    if ( u == i && v == j)
                        continue;

                    // AssignArray(CurrSolution->Loads, Neighbor.Loads);
                    memcpy(Neighbor.Loads, CurrSolution->Loads, sizeof(CurrSolution->Loads));
                    Neighbor.Loads[i] -= inst_tasks[Route[i][j]].demand;
                    Neighbor.Loads[u] += inst_tasks[Route[i][j]].demand;
                    Neighbor.TotalVioLoad = CurrSolution->TotalVioLoad;

                    if (Neighbor.Loads[i] > capacity)
                    {
                        Neighbor.TotalVioLoad -= inst_tasks[Route[i][j]].demand;
                    } else if (Neighbor.Loads[i] > capacity - inst_tasks[Route[i][j]].demand)
                    {
                        Neighbor.TotalVioLoad -= (Neighbor.Loads[i] + inst_tasks[Route[i][j]].demand - capacity);
                    }

                    if (Neighbor.Loads[u] > capacity + inst_tasks[Route[i][j]].demand)
                    {
                        Neighbor.TotalVioLoad += inst_tasks[Route[i][j]].demand;

                    } else if (Neighbor.Loads[u] > capacity)
                    {
                        Neighbor.TotalVioLoad += Neighbor.Loads[u] - capacity;
                    }

                    if (Neighbor.Loads[i] == 0)
                    {
                        RID1 = 0;
                        delete_element(Neighbor.Loads, i);
                    }
                    


                    Neighbor.TotalCost = CurrSolution->TotalCost - min_cost[inst_tasks[Route[i][j-1]].tail_node][inst_tasks[Route[i][j]].head_node]
                            - min_cost[inst_tasks[Route[i][j]].tail_node][inst_tasks[Route[i][j+1]].head_node]
                            + min_cost[inst_tasks[Route[i][j-1]].tail_node][inst_tasks[Route[i][j+1]].head_node]
                            - min_cost[inst_tasks[Route[u][v-1]].tail_node][inst_tasks[Route[u][v]].head_node]
                            + min_cost[inst_tasks[Route[u][v-1]].tail_node][inst_tasks[Route[i][j]].head_node]
                            + min_cost[inst_tasks[Route[i][j]].tail_node][inst_tasks[Route[u][v]].head_node];

                    count ++;
                    Neighbor.Fitness = Neighbor.TotalCost + PenPrmt * Neighbor.TotalVioLoad;

                    if (TabuList[Route[i][j]][u] == 0 || (Neighbor.TotalVioLoad > 0 && Neighbor.Fitness < BestFitness)
                        || (Neighbor.TotalVioLoad == 0 && Neighbor.Fitness < BestFsbFitness))
                    {
                        if (Neighbor.Fitness < BestSINeighbor->Fitness)
                        {
                            MovedTasks[1] = Route[i][j];
                            Besti = i;
                            Bestj = j;
                            Bestu = u;
                            Bestv = v;
                            ChangedRoutesID[0] = RID1;
                            ChangedRoutesID[1] = RID2;

                            BestSINeighbor->TotalCost = Neighbor.TotalCost;
                            // AssignArray(Neighbor.Loads, BestSINeighbor->Loads);
                            memcpy(BestSINeighbor->Loads, Neighbor.Loads, sizeof(CurrSolution->Loads));
                            BestSINeighbor->TotalVioLoad = Neighbor.TotalVioLoad;
                            BestSINeighbor->Fitness = Neighbor.Fitness;
                        }
                    }
                    // invert selected task
                    z = inst_tasks[Route[i][j]].inverse;
                    Neighbor.TotalCost = CurrSolution->TotalCost - min_cost[inst_tasks[Route[i][j-1]].tail_node][inst_tasks[Route[i][j]].head_node]
                                         - min_cost[inst_tasks[Route[i][j]].tail_node][inst_tasks[Route[i][j+1]].head_node]
                                         + min_cost[inst_tasks[Route[i][j-1]].tail_node][inst_tasks[Route[i][j+1]].head_node]
                                         - min_cost[inst_tasks[Route[u][v-1]].tail_node][inst_tasks[Route[u][v]].head_node]
                                         + min_cost[inst_tasks[Route[u][v-1]].tail_node][inst_tasks[z].head_node]
                                         + min_cost[inst_tasks[z].tail_node][inst_tasks[Route[u][v]].head_node];
                    count ++;
                    Neighbor.Fitness = Neighbor.TotalCost + PenPrmt * Neighbor.TotalVioLoad;
                    if (TabuList[Route[i][j]][u] == 0 || (Neighbor.TotalVioLoad > 0 && Neighbor.Fitness < BestFitness)
                        || (Neighbor.TotalVioLoad == 0 && Neighbor.Fitness < BestFsbFitness))
                    {
                        if (Neighbor.Fitness < BestSINeighbor->Fitness)
                        {
                            MovedTasks[1] = z;
                            Besti = i;
                            Bestj = j;
                            Bestu = u;
                            Bestv = v;
                            ChangedRoutesID[0] = RID1;
                            ChangedRoutesID[1] = RID2;

                            BestSINeighbor->TotalCost = Neighbor.TotalCost;
                            // AssignArray(Neighbor.Loads, BestSINeighbor->Loads);
                            memcpy(BestSINeighbor->Loads, Neighbor.Loads, sizeof(CurrSolution->Loads));
                            BestSINeighbor->TotalVioLoad = Neighbor.TotalVioLoad;
                            BestSINeighbor->Fitness = Neighbor.Fitness;
                        }
                    }
                }
            }
        }
    }

    if (Bestu < 0)
    {
        return count;
    }

    // Assign Sequence
    if (Bestu == 0)
    {
        // new routes
        BestSINeighbor->Sequence[0] = 1;
        for(k = 1; k < Positions[0]; k++)
        {
            if (k == Besti)
            {
                delete_element(Route[k], Bestj);

                BestSINeighbor->Sequence[0] --; // Route[1] = 0
                JoinArray(BestSINeighbor->Sequence, Route[k]);
            } else {
                BestSINeighbor->Sequence[0] --; // Route[1] = 0
                JoinArray(BestSINeighbor->Sequence, Route[k]);
            }
        }
        BestSINeighbor->Sequence[0] ++;
        BestSINeighbor->Sequence[BestSINeighbor->Sequence[0]] = MovedTasks[1];
        BestSINeighbor->Sequence[0] ++;
        BestSINeighbor->Sequence[BestSINeighbor->Sequence[0]] = 0;

    } else{

        BestSINeighbor->Sequence[0] = 1;
        for(k = 1; k < Positions[0]; k++)
        {
            if (k == Besti)
            {
                delete_element(Route[k], Bestj);
                if (Route[k][0] == 2)
                    continue;

                BestSINeighbor->Sequence[0] --; // Route[1] = 0
                JoinArray(BestSINeighbor->Sequence, Route[k]);
            } else if (k == Bestu){
                add_element(Route[k], MovedTasks[1], Bestv);
                BestSINeighbor->Sequence[0] --; // Route[1] = 0
                JoinArray(BestSINeighbor->Sequence, Route[k]);
            } else {
                BestSINeighbor->Sequence[0] --; // Route[1] = 0
                JoinArray(BestSINeighbor->Sequence, Route[k]);
            }
        }
    }

    return count;

}


int DoubleInsertion(CARPInd *BestDINeighbor, CARPInd *CurrSolution, int *MovedTasks, int *ChangedRoutesID, const Task *inst_tasks, int (*TabuList)[101],
                    double PenPrmt, double BestFitness, double BestFsbFitness)
{
    int i, j, k, u, v, z, w;

    int count = 0;  // record neighbourhood moves (number of fitness evaluation)
    int Besti, Bestj, Bestu, Bestv, RID1, RID2;
    int Positions[101], Route[101][MAX_TASK_SEQ_LENGTH];
    find_ele_positions(Positions, CurrSolution->Sequence, 0);

    Route[0][0] = Positions[0] - 1; // PA*
    for(i = 1; i < Positions[0]; i++)
    {
        AssignSubArray(CurrSolution->Sequence, Positions[i], Positions[i+1], Route[i]);
    }

    CARPInd Neighbor;

    MovedTasks[0] = 2;
    BestDINeighbor->Fitness = INF;

    Bestu = -1;


    for (i = 1; i < Positions[0]; i++)
    {
        if (Route[i][0] < 4)
            continue;
        RID1 = i;
        for (j = 2; j < Route[i][0]-1; j++) // Route[i]: 0, t1, t2, t3, ..., tn, 0
        {
            for (u = 0; u < Positions[0]; u++)
            {
                if (u == i)
                    continue;

                RID2 = u;

                if ( u==0 && Route[0][0] < vehicle_num && Route[i][0] > 4)
                {
                    // new routes
                    // AssignArray(CurrSolution->Loads, Neighbor.Loads);
                    memcpy(Neighbor.Loads, CurrSolution->Loads, sizeof(CurrSolution->Loads));
                    Neighbor.Loads[i] -= (inst_tasks[Route[i][j]].demand + inst_tasks[Route[i][j+1]].demand);
                    Neighbor.TotalVioLoad = CurrSolution->TotalVioLoad;

                    if (Neighbor.Loads[i] > capacity)
                    {
                        Neighbor.TotalVioLoad -= (inst_tasks[Route[i][j]].demand + inst_tasks[Route[i][j+1]].demand);
                    } else if ( Neighbor.Loads[i] > (capacity - inst_tasks[Route[i][j]].demand - inst_tasks[Route[i][j+1]].demand))
                    {
                        Neighbor.TotalVioLoad -= (Neighbor.Loads[i] - capacity + inst_tasks[Route[i][j]].demand + inst_tasks[Route[i][j+1]].demand);
                    }

                    Neighbor.Loads[0] ++;
                    Neighbor.Loads[Neighbor.Loads[0]] = inst_tasks[Route[i][j]].demand + inst_tasks[Route[i][j+1]].demand;

                    if (Neighbor.Loads[Neighbor.Loads[0]] > capacity)
                    {
                        Neighbor.TotalVioLoad += (Neighbor.Loads[Neighbor.Loads[0]] -capacity);
                    }

                    Neighbor.TotalCost = CurrSolution->TotalCost - min_cost[inst_tasks[Route[i][j-1]].tail_node][inst_tasks[Route[i][j]].head_node]
                                                                 - min_cost[inst_tasks[Route[i][j+1]].tail_node][inst_tasks[Route[i][j+2]].head_node]
                                                                 + min_cost[inst_tasks[Route[i][j-1]].tail_node][inst_tasks[Route[i][j+2]].head_node]
                                                                 + min_cost[DEPOT][inst_tasks[Route[i][j]].head_node]
                                                                 + min_cost[inst_tasks[Route[i][j+1]].tail_node][DEPOT];
                    count ++;
                    Neighbor.Fitness = Neighbor.TotalCost + PenPrmt * Neighbor.TotalVioLoad;

                    if (TabuList[Route[i][j]][u] == 0 || TabuList[Route[i][j+1]][u] == 0 || (Neighbor.TotalVioLoad > 0
                        && Neighbor.Fitness < BestFitness) || (Neighbor.TotalVioLoad == 0 && Neighbor.Fitness < BestFsbFitness))
                    {
                        if (Neighbor.Fitness < BestDINeighbor->Fitness)
                        {
                            MovedTasks[1] = Route[i][j];
                            MovedTasks[2] = Route[i][j+1];
                            Besti = i;
                            Bestj = j;
                            Bestu = u;
                            Bestv = 0;
                            ChangedRoutesID[0] = RID1;
                            ChangedRoutesID[1] = RID2;

                            BestDINeighbor->TotalCost = Neighbor.TotalCost;
                            // AssignArray(Neighbor.Loads, BestDINeighbor->Loads);
                            memcpy(BestDINeighbor->Loads, Neighbor.Loads, sizeof(CurrSolution->Loads));
                            BestDINeighbor->TotalVioLoad = Neighbor.TotalVioLoad;
                            BestDINeighbor->Fitness = Neighbor.Fitness;
                        }
                    }

                }


                if (u == 0)
                    continue;

                for (v = 2; v <= Route[u][0]; v++)
                {
                    
                    if (u == i && v == j )
                        continue;
                    // AssignArray(CurrSolution->Loads, Neighbor.Loads);
                    memcpy(Neighbor.Loads, CurrSolution->Loads, sizeof(CurrSolution->Loads));
                    Neighbor.Loads[i] -= (inst_tasks[Route[i][j]].demand + inst_tasks[Route[i][j+1]].demand);
                    Neighbor.Loads[u] += (inst_tasks[Route[i][j]].demand + inst_tasks[Route[i][j+1]].demand);
                    Neighbor.TotalVioLoad = CurrSolution->TotalVioLoad;

                    if ( Neighbor.Loads[i] > capacity)
                    {
                        Neighbor.TotalVioLoad -= (inst_tasks[Route[i][j]].demand + inst_tasks[Route[i][j+1]].demand);
                    } else if (Neighbor.Loads[i] > capacity - (inst_tasks[Route[i][j]].demand + inst_tasks[Route[i][j+1]].demand))
                    {
                        Neighbor.TotalVioLoad -= (Neighbor.Loads[i] - capacity + inst_tasks[Route[i][j]].demand + inst_tasks[Route[i][j+1]].demand);
                    }

                    if (Neighbor.Loads[u] > capacity + (inst_tasks[Route[i][j]].demand + inst_tasks[Route[i][j+1]].demand))
                    {
                        Neighbor.TotalVioLoad += inst_tasks[Route[i][j]].demand + inst_tasks[Route[i][j+1]].demand;
                    } else if (Neighbor.Loads[u] > capacity)
                    {
                        Neighbor.TotalVioLoad += (Neighbor.Loads[u] - capacity);
                    }
                    if (Neighbor.Loads[i] == 0)
                    {
                        RID1 = 0;
                        delete_element(Neighbor.Loads, i);
                    }
                    
                    Neighbor.TotalCost = CurrSolution->TotalCost - min_cost[inst_tasks[Route[i][j-1]].tail_node][inst_tasks[Route[i][j]].head_node]
                                         - min_cost[inst_tasks[Route[i][j+1]].tail_node][inst_tasks[Route[i][j+2]].head_node]
                                         + min_cost[inst_tasks[Route[i][j-1]].tail_node][inst_tasks[Route[i][j+2]].head_node]
                                         - min_cost[inst_tasks[Route[u][v-1]].tail_node][inst_tasks[Route[u][v]].head_node]
                                         + min_cost[inst_tasks[Route[u][v-1]].tail_node][inst_tasks[Route[i][j]].head_node]
                                         + min_cost[inst_tasks[Route[i][j+1]].tail_node][inst_tasks[Route[u][v]].head_node];
                    count ++;
                    Neighbor.Fitness = Neighbor.TotalCost + PenPrmt * Neighbor.TotalVioLoad;
                    if (TabuList[Route[i][j]][u] == 0 || TabuList[Route[i][j+1]][u] == 0 || (Neighbor.TotalVioLoad > 0
                       && Neighbor.Fitness < BestFitness) || (Neighbor.TotalVioLoad == 0 && Neighbor.Fitness < BestFsbFitness))
                    {
                        if (Neighbor.Fitness < BestDINeighbor->Fitness)
                        {
                            MovedTasks[1] = Route[i][j];
                            MovedTasks[2] = Route[i][j+1];
                            Besti = i;
                            Bestj = j;
                            Bestu = u;
                            Bestv = v;
                            ChangedRoutesID[0] = RID1;
                            ChangedRoutesID[1] = RID2;

                            BestDINeighbor->TotalCost = Neighbor.TotalCost;
                            // AssignArray(Neighbor.Loads, BestDINeighbor->Loads);
                            memcpy(BestDINeighbor->Loads, Neighbor.Loads, sizeof(CurrSolution->Loads));
                            BestDINeighbor->TotalVioLoad = Neighbor.TotalVioLoad;
                            BestDINeighbor->Fitness = Neighbor.Fitness;
                        }
                    }

                    w = inst_tasks[Route[i][j]].inverse;
                    Neighbor.TotalCost = CurrSolution->TotalCost - min_cost[inst_tasks[Route[i][j-1]].tail_node][inst_tasks[Route[i][j]].head_node]
                                         - min_cost[inst_tasks[Route[i][j+1]].tail_node][inst_tasks[Route[i][j+2]].head_node]
                                         - min_cost[inst_tasks[Route[i][j]].tail_node][inst_tasks[Route[i][j+1]].head_node]
                                         + min_cost[inst_tasks[Route[i][j-1]].tail_node][inst_tasks[Route[i][j+2]].head_node]
                                         - min_cost[inst_tasks[Route[u][v-1]].tail_node][inst_tasks[Route[u][v]].head_node]
                                         + min_cost[inst_tasks[Route[u][v-1]].tail_node][inst_tasks[w].head_node]
                                         + min_cost[inst_tasks[w].tail_node][inst_tasks[Route[i][j+1]].head_node]
                                         + min_cost[inst_tasks[Route[i][j+1]].tail_node][inst_tasks[Route[u][v]].head_node];
                    count ++;
                    Neighbor.Fitness = Neighbor.TotalCost + PenPrmt * Neighbor.TotalVioLoad;
                    if (TabuList[Route[i][j]][u] == 0 || TabuList[Route[i][j+1]][u] == 0 || (Neighbor.TotalVioLoad > 0
                                                                                             && Neighbor.Fitness < BestFitness) || (Neighbor.TotalVioLoad == 0 && Neighbor.Fitness < BestFsbFitness))
                    {
                        if (Neighbor.Fitness < BestDINeighbor->Fitness)
                        {
                            MovedTasks[1] = w;
                            MovedTasks[2] = Route[i][j+1];
                            Besti = i;
                            Bestj = j;
                            Bestu = u;
                            Bestv = v;
                            ChangedRoutesID[0] = RID1;
                            ChangedRoutesID[1] = RID2;

                            BestDINeighbor->TotalCost = Neighbor.TotalCost;
                            // AssignArray(Neighbor.Loads, BestDINeighbor->Loads);
                            memcpy(BestDINeighbor->Loads, Neighbor.Loads, sizeof(CurrSolution->Loads));
                            BestDINeighbor->TotalVioLoad = Neighbor.TotalVioLoad;
                            BestDINeighbor->Fitness = Neighbor.Fitness;
                        }
                    }

                    z = inst_tasks[Route[i][j+1]].inverse;
                    Neighbor.TotalCost = CurrSolution->TotalCost - min_cost[inst_tasks[Route[i][j-1]].tail_node][inst_tasks[Route[i][j]].head_node]
                                         - min_cost[inst_tasks[Route[i][j+1]].tail_node][inst_tasks[Route[i][j+2]].head_node]
                                         - min_cost[inst_tasks[Route[i][j]].tail_node][inst_tasks[Route[i][j+1]].head_node]
                                         + min_cost[inst_tasks[Route[i][j-1]].tail_node][inst_tasks[Route[i][j+2]].head_node]
                                         - min_cost[inst_tasks[Route[u][v-1]].tail_node][inst_tasks[Route[u][v]].head_node]
                                         + min_cost[inst_tasks[Route[u][v-1]].tail_node][inst_tasks[Route[i][j]].head_node]
                                         + min_cost[inst_tasks[Route[i][j]].tail_node][inst_tasks[z].head_node]
                                         + min_cost[inst_tasks[z].tail_node][inst_tasks[Route[u][v]].head_node];
                    count ++;
                    Neighbor.Fitness = Neighbor.TotalCost + PenPrmt * Neighbor.TotalVioLoad;
                    if (TabuList[Route[i][j]][u] == 0 || TabuList[Route[i][j+1]][u] == 0 || (Neighbor.TotalVioLoad > 0
                        && Neighbor.Fitness < BestFitness) || (Neighbor.TotalVioLoad == 0 && Neighbor.Fitness < BestFsbFitness))
                    {
                        if (Neighbor.Fitness < BestDINeighbor->Fitness)
                        {
                            MovedTasks[1] = Route[i][j];
                            MovedTasks[2] = z;
                            Besti = i;
                            Bestj = j;
                            Bestu = u;
                            Bestv = v;
                            ChangedRoutesID[0] = RID1;
                            ChangedRoutesID[1] = RID2;

                            BestDINeighbor->TotalCost = Neighbor.TotalCost;
                            // AssignArray(Neighbor.Loads, BestDINeighbor->Loads);
                            memcpy(BestDINeighbor->Loads, Neighbor.Loads, sizeof(CurrSolution->Loads));
                            BestDINeighbor->TotalVioLoad = Neighbor.TotalVioLoad;
                            BestDINeighbor->Fitness = Neighbor.Fitness;
                        }
                    }

                    Neighbor.TotalCost = CurrSolution->TotalCost - min_cost[inst_tasks[Route[i][j-1]].tail_node][inst_tasks[Route[i][j]].head_node]
                                         - min_cost[inst_tasks[Route[i][j+1]].tail_node][inst_tasks[Route[i][j+2]].head_node]
                                         - min_cost[inst_tasks[Route[i][j]].tail_node][inst_tasks[Route[i][j+1]].head_node]
                                         + min_cost[inst_tasks[Route[i][j-1]].tail_node][inst_tasks[Route[i][j+2]].head_node]
                                         - min_cost[inst_tasks[Route[u][v-1]].tail_node][inst_tasks[Route[u][v]].head_node]
                                         + min_cost[inst_tasks[Route[u][v-1]].tail_node][inst_tasks[w].head_node]
                                         + min_cost[inst_tasks[w].tail_node][inst_tasks[z].head_node]
                                         + min_cost[inst_tasks[z].tail_node][inst_tasks[Route[u][v]].head_node];
                    count ++;
                    Neighbor.Fitness = Neighbor.TotalCost + PenPrmt * Neighbor.TotalVioLoad;
                    if (TabuList[Route[i][j]][u] == 0 || TabuList[Route[i][j+1]][u] == 0 || (Neighbor.TotalVioLoad > 0
                                                                                             && Neighbor.Fitness < BestFitness) || (Neighbor.TotalVioLoad == 0 && Neighbor.Fitness < BestFsbFitness))
                    {
                        if (Neighbor.Fitness < BestDINeighbor->Fitness)
                        {
                            MovedTasks[1] = w;
                            MovedTasks[2] = z;
                            Besti = i;
                            Bestj = j;
                            Bestu = u;
                            Bestv = v;
                            
                            ChangedRoutesID[0] = RID1;
                            ChangedRoutesID[1] = RID2;

                            BestDINeighbor->TotalCost = Neighbor.TotalCost;
                            // AssignArray(Neighbor.Loads, BestDINeighbor->Loads);
                            memcpy(BestDINeighbor->Loads, Neighbor.Loads, sizeof(CurrSolution->Loads));
                            BestDINeighbor->TotalVioLoad = Neighbor.TotalVioLoad;
                            BestDINeighbor->Fitness = Neighbor.Fitness;
                        }
                    }
                }
            }
        }
    }

    if (Bestu < 0)
        return count;
    
    if (Bestu == 0)
    {
        BestDINeighbor->Sequence[0] = 1;
        for (k = 1; k < Positions[0]; k++)
        {
            if ( k == Besti )
            {
                delete_element(Route[k], Bestj+1);
                delete_element(Route[k], Bestj);
                BestDINeighbor->Sequence[0]--;
                JoinArray(BestDINeighbor->Sequence, Route[k]);
            } else {
                BestDINeighbor->Sequence[0]--;
                JoinArray(BestDINeighbor->Sequence, Route[k]);
            }
        }
        BestDINeighbor->Sequence[0] ++;
        BestDINeighbor->Sequence[BestDINeighbor->Sequence[0]] = MovedTasks[1];
        BestDINeighbor->Sequence[0] ++;
        BestDINeighbor->Sequence[BestDINeighbor->Sequence[0]] = MovedTasks[2];
        BestDINeighbor->Sequence[0] ++;
        BestDINeighbor->Sequence[BestDINeighbor->Sequence[0]] = 0;
    } else{

        BestDINeighbor->Sequence[0] = 1;
        for (k = 1; k < Positions[0]; k++)
        {
            if (k == Besti)
            {
                delete_element(Route[k], Bestj+1);
                delete_element(Route[k], Bestj);
                if (Route[k][0] == 2)
                    continue;

                BestDINeighbor->Sequence[0]--;
                JoinArray(BestDINeighbor->Sequence, Route[k]);
            } else if (k == Bestu)
            {
                add_element(Route[k], MovedTasks[2], Bestv);
                add_element(Route[k], MovedTasks[1], Bestv);
                BestDINeighbor->Sequence[0] --;
                JoinArray(BestDINeighbor->Sequence, Route[k]);
            } else{
                BestDINeighbor->Sequence[0] --;
                JoinArray(BestDINeighbor->Sequence, Route[k]);
            }
        }
    }

    return count;

}


int SWAP(CARPInd *BestSWAPNeighbor, CARPInd *CurrSolution, int *MovedTasks, int *ChangedRoutesID, const Task *inst_tasks, int (*TabuList)[101],
         double PenPrmt, double BestFitness, double BestFsbFitness)
{
    int i, j, k, u, v, z, w;

    int count = 0;  // record neighbourhood moves (number of fitness evaluation)
    int Besti, Bestj, Bestu, Bestv, RID1, RID2;
    int Positions[101], Route[101][MAX_TASK_SEQ_LENGTH];
    find_ele_positions(Positions, CurrSolution->Sequence, 0);

    Route[0][0] = Positions[0] - 1; // PA*
    for(i = 1; i < Positions[0]; i++)
    {
        AssignSubArray(CurrSolution->Sequence, Positions[i], Positions[i+1], Route[i]);
    }

    CARPInd Neighbor;

    MovedTasks[0] = 2;
    BestSWAPNeighbor->Fitness = INF;
    Bestu = -1;

    for (i = 1;  i < Route[0][0]; i++)
    {
        RID1 = i;
        for (j = 2; j < Route[i][0]; j++) // Route[i]: 0, t1, t2, t3, ..., tn, 0
        {
            for (u = i+1; u <= Route[0][0]; u++)
            {
                RID2 = u;
                for (v = 2; v < Route[u][0]; v++)
                {
                    // AssignArray(CurrSolution->Loads, Neighbor.Loads);
                    memcpy(Neighbor.Loads, CurrSolution->Loads, sizeof(CurrSolution->Loads));
                    Neighbor.Loads[i] -= inst_tasks[Route[i][j]].demand - inst_tasks[Route[u][v]].demand;
                    Neighbor.Loads[u] += inst_tasks[Route[i][j]].demand - inst_tasks[Route[u][v]].demand;
                    Neighbor.TotalVioLoad = CurrSolution->TotalVioLoad;

                    if (inst_tasks[Route[i][j]].demand > inst_tasks[Route[u][v]].demand)
                    {
                        if (Neighbor.Loads[i] > capacity)
                        {
                            Neighbor.TotalVioLoad -= inst_tasks[Route[i][j]].demand - inst_tasks[Route[u][v]].demand;
                        } else if (Neighbor.Loads[i] > capacity - inst_tasks[Route[i][j]].demand + inst_tasks[Route[u][v]].demand)
                        {
                            Neighbor.TotalVioLoad -= Neighbor.Loads[i] + inst_tasks[Route[i][j]].demand - inst_tasks[Route[u][v]].demand - capacity;
                        }

                        if (Neighbor.Loads[u] > capacity + inst_tasks[Route[i][j]].demand - inst_tasks[Route[u][v]].demand)
                        {
                            Neighbor.TotalVioLoad += inst_tasks[Route[i][j]].demand - inst_tasks[Route[u][v]].demand;
                        } else if (Neighbor.Loads[u] > capacity)
                        {
                            Neighbor.TotalVioLoad += Neighbor.Loads[u] - capacity;
                        }
                    } else{
                        if (Neighbor.Loads[u] > capacity)
                        {
                            Neighbor.TotalVioLoad -= inst_tasks[Route[u][v]].demand - inst_tasks[Route[i][j]].demand;
                        }
                        else if (Neighbor.Loads[u] > capacity - inst_tasks[Route[u][v]].demand + inst_tasks[Route[i][j]].demand)
                        {
                            Neighbor.TotalVioLoad -= Neighbor.Loads[u] + inst_tasks[Route[u][v]].demand - inst_tasks[Route[i][j]].demand - capacity;
                        }

                        if (Neighbor.Loads[i] > capacity + inst_tasks[Route[u][v]].demand - inst_tasks[Route[i][j]].demand)
                        {
                            Neighbor.TotalVioLoad += inst_tasks[Route[u][v]].demand - inst_tasks[Route[i][j]].demand;
                        }
                        else if (Neighbor.Loads[i] > capacity)
                        {
                            Neighbor.TotalVioLoad += Neighbor.Loads[i] - capacity;
                        }
                    }
                    Neighbor.TotalCost = CurrSolution->TotalCost - min_cost[inst_tasks[Route[i][j-1]].tail_node][inst_tasks[Route[i][j]].head_node]
                            - min_cost[inst_tasks[Route[i][j]].tail_node][inst_tasks[Route[i][j+1]].head_node]
                            - min_cost[inst_tasks[Route[u][v-1]].tail_node][inst_tasks[Route[u][v]].head_node]
                            - min_cost[inst_tasks[Route[u][v]].tail_node][inst_tasks[Route[u][v+1]].head_node]
                            + min_cost[inst_tasks[Route[u][v-1]].tail_node][inst_tasks[Route[i][j]].head_node]
                            + min_cost[inst_tasks[Route[i][j]].tail_node][inst_tasks[Route[u][v+1]].head_node]
                            + min_cost[inst_tasks[Route[i][j-1]].tail_node][inst_tasks[Route[u][v]].head_node]
                            + min_cost[inst_tasks[Route[u][v]].tail_node][inst_tasks[Route[i][j+1]].head_node];

                    count ++;
                    Neighbor.Fitness = Neighbor.TotalCost + PenPrmt * Neighbor.TotalVioLoad;
                    if (TabuList[Route[i][j]][u] == 0 || TabuList[Route[u][v]][i] == 0 || (Neighbor.TotalVioLoad > 0 && Neighbor.Fitness < BestFitness)	|| (Neighbor.TotalVioLoad == 0 && Neighbor.Fitness < BestFsbFitness))
                    {
                        if (Neighbor.Fitness < BestSWAPNeighbor->Fitness)
                        {
                            MovedTasks[1] = Route[i][j];
                            MovedTasks[2] = Route[u][v];
                            Besti = i;
                            Bestj = j;
                            Bestu = u;
                            Bestv = v;
                            ChangedRoutesID[0] = RID1;
                            ChangedRoutesID[1] = RID2;

                            BestSWAPNeighbor->TotalCost = Neighbor.TotalCost;
                            // AssignArray(Neighbor.Loads, BestSWAPNeighbor->Loads);
                            memcpy(BestSWAPNeighbor->Loads, Neighbor.Loads, sizeof(CurrSolution->Loads));
                            BestSWAPNeighbor->TotalVioLoad = Neighbor.TotalVioLoad;
                            BestSWAPNeighbor->Fitness = Neighbor.Fitness;
                        }
                    }

                    w = inst_tasks[Route[i][j]].inverse;
                    Neighbor.TotalCost = CurrSolution->TotalCost - min_cost[inst_tasks[Route[i][j-1]].tail_node][inst_tasks[Route[i][j]].head_node]
                                         - min_cost[inst_tasks[Route[i][j]].tail_node][inst_tasks[Route[i][j+1]].head_node]
                                         - min_cost[inst_tasks[Route[u][v-1]].tail_node][inst_tasks[Route[u][v]].head_node]
                                         - min_cost[inst_tasks[Route[u][v]].tail_node][inst_tasks[Route[u][v+1]].head_node]
                                         + min_cost[inst_tasks[Route[u][v-1]].tail_node][inst_tasks[w].head_node]
                                         + min_cost[inst_tasks[w].tail_node][inst_tasks[Route[u][v+1]].head_node]
                                         + min_cost[inst_tasks[Route[i][j-1]].tail_node][inst_tasks[Route[u][v]].head_node]
                                         + min_cost[inst_tasks[Route[u][v]].tail_node][inst_tasks[Route[i][j+1]].head_node];

                    count ++;
                    Neighbor.Fitness = Neighbor.TotalCost + PenPrmt * Neighbor.TotalVioLoad;
                    if (TabuList[Route[i][j]][u] == 0 || TabuList[Route[u][v]][i] == 0 || (Neighbor.TotalVioLoad > 0 && Neighbor.Fitness < BestFitness)	|| (Neighbor.TotalVioLoad == 0 && Neighbor.Fitness < BestFsbFitness))
                    {
                        if (Neighbor.Fitness < BestSWAPNeighbor->Fitness)
                        {
                            MovedTasks[1] = w;
                            MovedTasks[2] = Route[u][v];
                            Besti = i;
                            Bestj = j;
                            Bestu = u;
                            Bestv = v;
                            ChangedRoutesID[0] = RID1;
                            ChangedRoutesID[1] = RID2;

                            BestSWAPNeighbor->TotalCost = Neighbor.TotalCost;
                            // AssignArray(Neighbor.Loads, BestSWAPNeighbor->Loads);
                            memcpy(BestSWAPNeighbor->Loads, Neighbor.Loads, sizeof(CurrSolution->Loads));
                            BestSWAPNeighbor->TotalVioLoad = Neighbor.TotalVioLoad;
                            BestSWAPNeighbor->Fitness = Neighbor.Fitness;
                        }
                    }

                    z = inst_tasks[Route[u][v]].inverse;
                    Neighbor.TotalCost = CurrSolution->TotalCost - min_cost[inst_tasks[Route[i][j-1]].tail_node][inst_tasks[Route[i][j]].head_node]
                                         - min_cost[inst_tasks[Route[i][j]].tail_node][inst_tasks[Route[i][j+1]].head_node]
                                         - min_cost[inst_tasks[Route[u][v-1]].tail_node][inst_tasks[Route[u][v]].head_node]
                                         - min_cost[inst_tasks[Route[u][v]].tail_node][inst_tasks[Route[u][v+1]].head_node]
                                         + min_cost[inst_tasks[Route[u][v-1]].tail_node][inst_tasks[Route[i][j]].head_node]
                                         + min_cost[inst_tasks[Route[i][j]].tail_node][inst_tasks[Route[u][v+1]].head_node]
                                         + min_cost[inst_tasks[Route[i][j-1]].tail_node][inst_tasks[z].head_node]
                                         + min_cost[inst_tasks[z].tail_node][inst_tasks[Route[i][j+1]].head_node];

                    count ++;
                    Neighbor.Fitness = Neighbor.TotalCost + PenPrmt * Neighbor.TotalVioLoad;
                    if (TabuList[Route[i][j]][u] == 0 || TabuList[Route[u][v]][i] == 0 || (Neighbor.TotalVioLoad > 0 && Neighbor.Fitness < BestFitness)	|| (Neighbor.TotalVioLoad == 0 && Neighbor.Fitness < BestFsbFitness))
                    {
                        if (Neighbor.Fitness < BestSWAPNeighbor->Fitness)
                        {
                            MovedTasks[1] = Route[i][j];
                            MovedTasks[2] = z;
                            Besti = i;
                            Bestj = j;
                            Bestu = u;
                            Bestv = v;
                            ChangedRoutesID[0] = RID1;
                            ChangedRoutesID[1] = RID2;

                            BestSWAPNeighbor->TotalCost = Neighbor.TotalCost;
                            // AssignArray(Neighbor.Loads, BestSWAPNeighbor->Loads);
                            memcpy(BestSWAPNeighbor->Loads, Neighbor.Loads, sizeof(CurrSolution->Loads));
                            BestSWAPNeighbor->TotalVioLoad = Neighbor.TotalVioLoad;
                            BestSWAPNeighbor->Fitness = Neighbor.Fitness;
                        }
                    }

                    Neighbor.TotalCost = CurrSolution->TotalCost - min_cost[inst_tasks[Route[i][j-1]].tail_node][inst_tasks[Route[i][j]].head_node]
                                         - min_cost[inst_tasks[Route[i][j]].tail_node][inst_tasks[Route[i][j+1]].head_node]
                                         - min_cost[inst_tasks[Route[u][v-1]].tail_node][inst_tasks[Route[u][v]].head_node]
                                         - min_cost[inst_tasks[Route[u][v]].tail_node][inst_tasks[Route[u][v+1]].head_node]
                                         + min_cost[inst_tasks[Route[u][v-1]].tail_node][inst_tasks[w].head_node]
                                         + min_cost[inst_tasks[w].tail_node][inst_tasks[Route[u][v+1]].head_node]
                                         + min_cost[inst_tasks[Route[i][j-1]].tail_node][inst_tasks[z].head_node]
                                         + min_cost[inst_tasks[z].tail_node][inst_tasks[Route[i][j+1]].head_node];

                    count ++;
                    Neighbor.Fitness = Neighbor.TotalCost + PenPrmt * Neighbor.TotalVioLoad;
                    if (TabuList[Route[i][j]][u] == 0 || TabuList[Route[u][v]][i] == 0 || (Neighbor.TotalVioLoad > 0 && Neighbor.Fitness < BestFitness)	|| (Neighbor.TotalVioLoad == 0 && Neighbor.Fitness < BestFsbFitness))
                    {
                        if (Neighbor.Fitness < BestSWAPNeighbor->Fitness)
                        {
                            MovedTasks[1] = w;
                            MovedTasks[2] = z;
                            Besti = i;
                            Bestj = j;
                            Bestu = u;
                            Bestv = v;
                            ChangedRoutesID[0] = RID1;
                            ChangedRoutesID[1] = RID2;

                            BestSWAPNeighbor->TotalCost = Neighbor.TotalCost;
                            // AssignArray(Neighbor.Loads, BestSWAPNeighbor->Loads);
                            memcpy(BestSWAPNeighbor->Loads, Neighbor.Loads, sizeof(CurrSolution->Loads));
                            BestSWAPNeighbor->TotalVioLoad = Neighbor.TotalVioLoad;
                            BestSWAPNeighbor->Fitness = Neighbor.Fitness;
                        }
                    }
                }
            }
        }
    }

    if(Bestu < 0)
    {
        return count;
    }
    BestSWAPNeighbor->Sequence[0] = 1;
    for (k = 1; k < Positions[0]; k++)
    {
        if (k == Besti)
        {
            delete_element(Route[k], Bestj);
            add_element(Route[k], MovedTasks[2], Bestj);
            BestSWAPNeighbor->Sequence[0] --;
            JoinArray(BestSWAPNeighbor->Sequence, Route[k]);
        } else if (k == Bestu)
        {
            delete_element(Route[k], Bestv);
            add_element(Route[k], MovedTasks[1], Bestv);
            BestSWAPNeighbor->Sequence[0] --;
            JoinArray(BestSWAPNeighbor->Sequence, Route[k]);
        } else{
            BestSWAPNeighbor->Sequence[0] --;
            JoinArray(BestSWAPNeighbor->Sequence, Route[k]);
        }
    }
    return count;

}

void RepairInfeasibility (CARPInd *Indi, const Task *inst_tasks)
{
    int NRE = req_edge_num, NRA = req_arc_num;
    int i, j, k, RoutesNum;
    int NodeRoutes[101][MAX_TASK_SEQ_LENGTH], TaskRoutes[101][MAX_TASK_SEQ_LENGTH];
    int InRoutes[NRE+NRA + 1][101], CheckMark[NRE+NRA+1];
    int NO_Task = 2*NRE + NRA;

    // InRoutes: the row TID save all routes ID where task TID located
    // e.g. InRoutes[1] = {1, 2, 3}, task 1 exists in route 1, 2, 3.
    // for (i = 1; i <= NRE+NRA; i++)
    // {
    //     InRoutes[i][0] = 0;
    // }
    memset(InRoutes, 0, sizeof(InRoutes));
    memset(NodeRoutes, 0, sizeof(NodeRoutes));
    memset(TaskRoutes, 0, sizeof(TaskRoutes));

    // TaskRoutes: each row is the task sequence of each sub-route
    // RoutesNum: the number of routes
    RoutesNum = 0;
    for (i = 1; i < Indi->Sequence[0]; i++)
    {
        if (Indi->Sequence[i] == 0)
        {
            TaskRoutes[RoutesNum][0]++;
            TaskRoutes[RoutesNum][TaskRoutes[RoutesNum][0]] = 0;
            RoutesNum ++;
            TaskRoutes[RoutesNum][0] = 1;
            TaskRoutes[RoutesNum][1] = 0;
        } else{
            TaskRoutes[RoutesNum][0] ++;
            TaskRoutes[RoutesNum][TaskRoutes[RoutesNum][0]] = Indi->Sequence[i];
        }
    }
    TaskRoutes[RoutesNum][0]++;
    TaskRoutes[RoutesNum][TaskRoutes[RoutesNum][0]] = 0;

    // NodeRoutes: each row is the actual node sequence of each sub-route
    for (i = 1; i <= RoutesNum; i++)
    {
        NodeRoutes[i][0] = 0;
        for (j = 1; j < TaskRoutes[i][0]; j++)
        {
            if (inst_tasks[TaskRoutes[i][j+1]].vt > 0)
            {
                int TID = TaskRoutes[i][j+1];
                if (TID > NRE)
                    TID -= NRE;
                InRoutes[TID][0] ++;
                InRoutes[TID][InRoutes[TID][0]] = i;
                continue;
            }
            for (k = 1; k <= shortest_path[inst_tasks[TaskRoutes[i][j]].tail_node][inst_tasks[TaskRoutes[i][j+1]].head_node][0]; k++)
            {
                NodeRoutes[i][0] ++;
                NodeRoutes[i][NodeRoutes[i][0]] = shortest_path[inst_tasks[TaskRoutes[i][j]].tail_node][inst_tasks[TaskRoutes[i][j+1]].head_node][k];
            }
        }
    }

    // InRoutes: the row TID save all routes ID where task TID located
    // e.g. InRoutes[1] = {1, 2, 3}, task 1 exists in route 1, 2, 3.
    for (i=1; i<=RoutesNum; i++)
    {
        memset(CheckMark, 0, sizeof(CheckMark));
        for (j=1; j < NodeRoutes[i][0]; j++)
        {
            int TID = FindTask(NodeRoutes[i][j], NodeRoutes[i][j+1], inst_tasks, NO_Task);
            if (TID > NRE)
                TID -= NRE;
            if (TID == 0) // if TID==0, it is not a task
                continue;

            if (!CheckMark[TID])
            {
                CheckMark[TID] = 1;
                InRoutes[TID][0] ++;
                InRoutes[TID][InRoutes[TID][0]] = i;
            }
        }
    }

    // FreeTasks: tasks that exists in over one routes
    // tasks only exist in one route is ignored because it can't be assigned into different routes.
    int FreeTasks[NRE + NRA + 1];
    FreeTasks[0] = 0;
    for (i = 1; i <= NRE + NRA; i++)
    {
        if (InRoutes[i][0] == 1) // tasks only exist in one route;
            continue;
        FreeTasks[0] ++;
        FreeTasks[FreeTasks[0]] = i;
    }


    struct TaskAssignment
    {
        int Assignment[300]; // which route is assigned for each task.
        int Loads[101];
        int TotalVioLoad;
    };

    struct TaskAssignment CurrTA, NeighTA, NextTA, BestTA;

    memset(CurrTA.Assignment, 0, sizeof(CurrTA.Assignment));
    memset(CurrTA.Loads, 0, sizeof(CurrTA.Loads));
    CurrTA.Loads[0] = RoutesNum;

    // assign tasks which only exist in one route
    for (i = 1; i <= NRE + NRA; i++)
    {
        // e.g. InRoutes[1] = {1, 2, 3}, task 1 exists in route 1, 2, 3.
        if (InRoutes[i][0] == 1)
        {
            CurrTA.Assignment[i] = InRoutes[i][1];
            CurrTA.Loads[InRoutes[i][1]] += inst_tasks[i].demand;
        }
    }

    int LeftTasks[NRE+NRA+1], RouteCandDemands[RoutesNum+1], CandRoutes[NRE+NRA+1][101];
    AssignArray(FreeTasks, LeftTasks);
    memset(RouteCandDemands, 0, sizeof(RouteCandDemands));
    for(i = 1; i <= NRE+NRA; i++)
    {
        CandRoutes[i][0] = 0; // The routes which task i can be assigned.
        for (j = 1; j <= InRoutes[i][0]; j++)
        {
            if (CurrTA.Loads[InRoutes[i][j]] >= capacity)
                continue;

            CandRoutes[i][0] ++;
            CandRoutes[i][CandRoutes[i][0]] = InRoutes[i][j];
        }
    }

    // RouteCandDemands[i] denotes Route[i].
    // The value of RouteCandDemands[i] represents the total demand of all possible assigned tasks.
    // LeftTasks = FreeTasks： tasks that exists in over one routes
    for (i = 1; i <= LeftTasks[0]; i++)
    {
        for (j = 1; j <= InRoutes[LeftTasks[i]][0]; j++)
        {
            RouteCandDemands[InRoutes[LeftTasks[i]][j]] += inst_tasks[LeftTasks[i]].demand;
        }
    }

    int SelID, AssignedRouteID;
    int n;
    for (n = 1; n <= FreeTasks[0]; n++)
    {
        SelID = 0; // select item
        for (i = 1; i <= LeftTasks[0]; i++)
        {
            if (SelID == 0)
            {
                SelID = i;
            } else if (CandRoutes[LeftTasks[i]][0] < CandRoutes[LeftTasks[SelID]][0]) // select smallest _|^|_
            {
                SelID = i;
            } else if (CandRoutes[LeftTasks[i]][0] == CandRoutes[LeftTasks[SelID]][0])
            {
                if (inst_tasks[LeftTasks[i]].demand > inst_tasks[LeftTasks[SelID]].demand) // select the max demand
                {
                    SelID = i;
                }
            }
        }

        AssignedRouteID = 0; // select bin
        for (i = 1; i <= InRoutes[LeftTasks[SelID]][0]; i++)
        {
            if (AssignedRouteID == 0)
            {
                AssignedRouteID = i;
            }
            else if (CurrTA.Loads[InRoutes[LeftTasks[SelID]][i]]+inst_tasks[LeftTasks[SelID]].demand <
                     CurrTA.Loads[InRoutes[LeftTasks[SelID]][AssignedRouteID]]+inst_tasks[LeftTasks[SelID]].demand)
            {
                AssignedRouteID = i;
            }
            else if (CurrTA.Loads[InRoutes[LeftTasks[SelID]][i]]+inst_tasks[LeftTasks[SelID]].demand ==
                     CurrTA.Loads[InRoutes[LeftTasks[SelID]][AssignedRouteID]]+inst_tasks[LeftTasks[SelID]].demand)
            {
                if (RouteCandDemands[InRoutes[LeftTasks[SelID]][i]]+CurrTA.Loads[InRoutes[LeftTasks[SelID]][i]] >
                    RouteCandDemands[InRoutes[LeftTasks[SelID]][AssignedRouteID]]+
                    CurrTA.Loads[InRoutes[LeftTasks[SelID]][AssignedRouteID]])
                {
                    AssignedRouteID = i;
                }
            }
        }

        CurrTA.Assignment[LeftTasks[SelID]] = InRoutes[LeftTasks[SelID]][AssignedRouteID];
        CurrTA.Loads[CurrTA.Assignment[LeftTasks[SelID]]] += inst_tasks[LeftTasks[SelID]].demand;
        for (i = 1; i <= InRoutes[LeftTasks[SelID]][0]; i++)
        {
            RouteCandDemands[InRoutes[LeftTasks[SelID]][i]] -= inst_tasks[LeftTasks[SelID]].demand;
        }

        if (CurrTA.Loads[CurrTA.Assignment[LeftTasks[SelID]]] >= capacity)
        {
            for (i = 1; i <= NRE+NRA; i++)
            {
                for (j = 1; j <= CandRoutes[i][0]; j++)
                {
                    if (CandRoutes[i][j] == CurrTA.Assignment[LeftTasks[SelID]])
                    {
                        delete_element(CandRoutes[i], j);
                        break;
                    }
                }
            }
        }
        delete_element(LeftTasks, SelID);
    }
    CurrTA.TotalVioLoad = 0;
    for (i = 1; i <= RoutesNum; i++)
    {
        if (CurrTA.Loads[i] > capacity)
            CurrTA.TotalVioLoad += CurrTA.Loads[i]-capacity;
    }

    BestTA = CurrTA;

    // tabu search for repairing
    int TabuList[NRE+NRA+1];
    int TabuTenure = FreeTasks[0]/2;
    int TabuTask, Tabu;
    memset(TabuList, 0, sizeof(TabuList));

    int count = 0;
    int stlcount = 0;
    while (BestTA.TotalVioLoad > 0)
    {
        count ++;
        stlcount ++;
        NextTA.TotalVioLoad = INF;
        for (i = 1; i <= FreeTasks[0]; i++)
        {
            for (j = 1; j <= InRoutes[FreeTasks[i]][0]; j++)
            {
                if (InRoutes[FreeTasks[i]][j] == CurrTA.Assignment[FreeTasks[i]])
                    continue;

                NeighTA = CurrTA;
                NeighTA.Assignment[FreeTasks[i]] = InRoutes[FreeTasks[i]][j];
                NeighTA.Loads[CurrTA.Assignment[FreeTasks[i]]] -= inst_tasks[FreeTasks[i]].demand;
                NeighTA.Loads[NeighTA.Assignment[FreeTasks[i]]] += inst_tasks[FreeTasks[i]].demand;

                if (CurrTA.Loads[NeighTA.Assignment[FreeTasks[i]]] >= capacity)
                {
                    NeighTA.TotalVioLoad += inst_tasks[FreeTasks[i]].demand;
                }
                else if (NeighTA.Loads[NeighTA.Assignment[FreeTasks[i]]] > capacity)
                {
                    NeighTA.TotalVioLoad += NeighTA.Loads[NeighTA.Assignment[FreeTasks[i]]]-capacity;
                }

                if (NeighTA.Loads[CurrTA.Assignment[FreeTasks[i]]] >= capacity)
                {
                    NeighTA.TotalVioLoad -= inst_tasks[FreeTasks[i]].demand;
                }
                else if (CurrTA.Loads[CurrTA.Assignment[FreeTasks[i]]] > capacity)
                {
                    NeighTA.TotalVioLoad -= CurrTA.Loads[CurrTA.Assignment[FreeTasks[i]]]-capacity;
                }

                Tabu = 0;
                if (TabuList[FreeTasks[i]] > 0 && NeighTA.TotalVioLoad >= BestTA.TotalVioLoad)
                    Tabu = 1;

                if (Tabu)
                    continue;

                if (NeighTA.TotalVioLoad < NextTA.TotalVioLoad)
                {
                    NextTA = NeighTA;
                    TabuTask = FreeTasks[i];
                }
            }
        }
        for (i = 1; i <= NRE + NRA; i++)
        {
            if (TabuList[i] > 0)
                TabuList[i] --;
        }

        TabuList[TabuTask] = TabuTenure;
        CurrTA = NextTA;
        if (CurrTA.TotalVioLoad < BestTA.TotalVioLoad)
        {
            stlcount = 0;
            BestTA = CurrTA;
        }

        if (count == 2*(NRA+NRA) || stlcount == (NRE + NRA) / 2)
            break;
    }
    if (BestTA.TotalVioLoad > Indi->TotalVioLoad)
        return;

    int Served[NRA + NRE + 1];
    memset(Served, 0, sizeof(Served));

    Indi->Sequence[0] = 1;
    Indi->Sequence[1] = 0;

    for (i=1; i <= RoutesNum; i++)
    {
        int vtFlag = 0;
        for (j = 1; j < NodeRoutes[i][0]; j++)
        {
            int TID;
            if (NodeRoutes[i][j] != 1 && vtFlag == 0)
            {
                for (k=req_edge_num; inst_tasks[k].vt > 0; k--)
                {
                    if(InRoutes[k][1] == i)
                    {
                        TID = k;
                        break;
                    }
                }
                // for (k = NO_Task/2; k > 0; k--)
                // {
                //     if (inst_tasks[k].tail_node == NodeRoutes[i][j])
                //     {
                //         TID = k;
                //         break;
                //     }
                // }
                vtFlag = 1;
                j --;
            } else{
                TID = FindTask(NodeRoutes[i][j], NodeRoutes[i][j+1], inst_tasks, NO_Task);
            }
            int TmpTID = TID;

            if (TmpTID > NRE)
                TmpTID -= NRE;

            if (TmpTID == 0)
                continue;

            if (BestTA.Assignment[TmpTID] == i && !Served[TmpTID])
            {
                Served[TmpTID] = 1;
                Indi->Sequence[0] ++;
                Indi->Sequence[Indi->Sequence[0]] = TID;
            }

        }
        if (Indi->Sequence[Indi->Sequence[0]] != 0)
        {
            Indi->Sequence[0] ++;
            Indi->Sequence[Indi->Sequence[0]] = 0;
        }
    }

    for(i = BestTA.Loads[0]; i > 0; i--)
    {
        if (BestTA.Loads[i] == 0)
            delete_element(BestTA.Loads, i);
    }

    Indi->TotalCost = get_task_seq_total_cost(Indi->Sequence, inst_tasks);
    AssignArray(BestTA.Loads, Indi->Loads);
    Indi->TotalVioLoad = BestTA.TotalVioLoad;

}







void GetConnectivePiece(int Root, int CurrentPiece, int *PieceMark, int (*Neighbors)[MAX_NODE_TAG_LENGTH]);
void GetMinCostTree(int (*CPMCTree)[MAX_NODE_TAG_LENGTH], int (*CPMinCost)[MAX_NODE_TAG_LENGTH]);
void GetEulerRoute(int *EulerRoute, int (*AdMatrix)[MAX_NODE_TAG_LENGTH], int *Nodes);
void FindCircuit(int *EulerRoute, int CurrentNode, int (*AdMatrix)[MAX_NODE_TAG_LENGTH], int *Nodes);
void EvenAllOddNodes(int (*AdMatrix)[MAX_NODE_TAG_LENGTH], int *OddNodes);

void FredericksonHeuristic(int *FHRoute, int *Route, const Task *inst_tasks)
{
    // min_cost, task_num, vertex_num are extern variables
    int i, j, k, u, v;

    int EulerRoute[3*(Route[0]-1)];

    int Degree[MAX_NODE_TAG_LENGTH];
    int OddNodes[MAX_NODE_TAG_LENGTH];

    int Nodes[2*Route[0]+1];
    Nodes[0] = 1;
    Nodes[1] = 1;

    int flag[MAX_NODE_TAG_LENGTH];
    memset(flag, 0, sizeof(flag));
    flag[1] = 1;

    int AdMatrix[MAX_NODE_TAG_LENGTH][MAX_NODE_TAG_LENGTH];
    int TaskMatrix[MAX_TASK_SEQ_LENGTH][MAX_TASK_SEQ_LENGTH];

    memset(AdMatrix, 0, sizeof(AdMatrix));
    memset(TaskMatrix, 0, sizeof(TaskMatrix));

    int Neighbors[MAX_NODE_TAG_LENGTH][MAX_NODE_TAG_LENGTH];
    for (i=1; i<=vertex_num; i++)
    {
        Neighbors[i][0] = 0;
    }

//    int ConnectivePieceNodes[2*Route[0]+2][2*Route[0]+2];
    int ConnectivePieceNodes[MAX_NODE_TAG_LENGTH][MAX_NODE_TAG_LENGTH];
    memset(ConnectivePieceNodes, 0, sizeof(ConnectivePieceNodes));
    ConnectivePieceNodes[0][0] = 0;

    int PieceMark[MAX_NODE_TAG_LENGTH];
    memset(PieceMark, 0, sizeof(PieceMark));
    PieceMark[0] = vertex_num;

    int CPMinCost[MAX_NODE_TAG_LENGTH][MAX_NODE_TAG_LENGTH];
    for (i = 1; i<=vertex_num; i++)
    {
        for(j = 1; j<=vertex_num; j++)
        {
            CPMinCost[i][j] = INF;
        }
    }

    int CPMCTree[MAX_NODE_TAG_LENGTH][MAX_NODE_TAG_LENGTH];
    int CurrentPiece = 0;

    int virtualTask[101];
    memset(virtualTask, 0, sizeof(virtualTask));

    for (i = 2; i < Route[0]; i++)
    {
        // ignore the virtual task
        if (inst_tasks[Route[i]].vt)
        {
            virtualTask[0] ++;
            virtualTask[virtualTask[0]] = Route[i];
            continue;
        }
        Neighbors[inst_tasks[Route[i]].tail_node][0]++;
        Neighbors[inst_tasks[Route[i]].tail_node][Neighbors[inst_tasks[Route[i]].tail_node][0]] = inst_tasks[Route[i]].head_node;
        Neighbors[inst_tasks[Route[i]].head_node][0]++;
        Neighbors[inst_tasks[Route[i]].head_node][Neighbors[inst_tasks[Route[i]].head_node][0]] = inst_tasks[Route[i]].tail_node;

        if (! flag[inst_tasks[Route[i]].tail_node])
        {
            Nodes[0] ++;
            Nodes[Nodes[0]] = inst_tasks[Route[i]].tail_node;
            flag[inst_tasks[Route[i]].tail_node] = 1;
        }
        if (! flag[inst_tasks[Route[i]].head_node])
        {
            Nodes[0] ++;
            Nodes[Nodes[0]] = inst_tasks[Route[i]].head_node;
            flag[inst_tasks[Route[i]].head_node] = 1;
        }

        AdMatrix[inst_tasks[Route[i]].head_node][inst_tasks[Route[i]].tail_node] ++;
        AdMatrix[inst_tasks[Route[i]].tail_node][inst_tasks[Route[i]].head_node] ++;
        TaskMatrix[inst_tasks[Route[i]].head_node][inst_tasks[Route[i]].tail_node] ++;
        TaskMatrix[inst_tasks[Route[i]].tail_node][inst_tasks[Route[i]].head_node] ++;
    }

    for (i=1; i<=Nodes[0]; i++)
    {
        if (PieceMark[Nodes[i]] > 0)
            continue;
        CurrentPiece ++;
        GetConnectivePiece(Nodes[i], CurrentPiece, PieceMark, Neighbors);
    }

    ConnectivePieceNodes[0][0] = max(PieceMark);
    for(i=1; i<=Nodes[0]; i++)
    {
        ConnectivePieceNodes[PieceMark[Nodes[i]]][0] ++;
        ConnectivePieceNodes[PieceMark[Nodes[i]]][ConnectivePieceNodes[PieceMark[Nodes[i]]][0]] = Nodes[i];
    }
    if (ConnectivePieceNodes[0][0] > 1)
    {
        CPMinCost[0][0] = ConnectivePieceNodes[0][0];
        CPMCTree[0][0] = ConnectivePieceNodes[0][0];

        int BNode[CPMinCost[0][0]+1][CPMinCost[0][0]+1];
        int ENode[CPMinCost[0][0]+1][CPMinCost[0][0]+1];

        for (i=1; i < ConnectivePieceNodes[0][0]; i++)
        {
            for (j=i+1; j <= ConnectivePieceNodes[0][0]; j++)
            {
                if (i==j)
                    continue;
                for(u = 1; u <= ConnectivePieceNodes[i][0]; u++)
                {
                    for(v = 1; v <= ConnectivePieceNodes[j][0]; v++)
                    {
                        if(min_cost[ConnectivePieceNodes[i][u]][ConnectivePieceNodes[j][v]] < CPMinCost[i][j])
                        {
                            CPMinCost[i][j] = min_cost[ConnectivePieceNodes[i][u]][ConnectivePieceNodes[j][v]];
                            CPMinCost[j][i] = CPMinCost[i][j];
                            BNode[i][j] = ConnectivePieceNodes[i][u];
                            ENode[i][j] = ConnectivePieceNodes[j][v];

                            BNode[j][i] = ENode[i][j];
                            ENode[j][i] = BNode[i][j];
                        }
                    }
                }
            }
        }

        GetMinCostTree(CPMCTree, CPMinCost);

        for (i=1;  i< CPMCTree[0][0]; i++)
        {
            for (j=i+1; j <= CPMCTree[0][0]; j++)
            {
                if (CPMCTree[i][j])
                {
                    AdMatrix[BNode[i][j]][ENode[i][j]] ++;
                    AdMatrix[ENode[i][j]][BNode[i][j]] ++;

                    if (CPMinCost[i][j] == INF)
                    {
                        CPMinCost[i][j] = min_cost[BNode[i][j]][ENode[i][j]];
                        CPMinCost[j][i] = CPMinCost[i][j];
                    }
                }
            }
        }
    }
    memset(Degree, 0, sizeof(Degree));
    OddNodes[0] = 0;
    for (i = 1; i <= Nodes[0]; i++)
    {
        for (j = 1; j <= Nodes[0]; j++ )
        {
            Degree[Nodes[i]] += AdMatrix[Nodes[i]][Nodes[j]];
        }
        if (Degree[Nodes[i]] % 2 == 1)
        {
            OddNodes[0] ++;
            OddNodes[OddNodes[0]] = Nodes[i];
        }
    }

    // EvenAllNodes
    EvenAllOddNodes(AdMatrix, OddNodes);

    // GetEulerRoute
    GetEulerRoute(EulerRoute, AdMatrix, Nodes);

    int used_task[task_num+2];
    memset(used_task, 0, sizeof(used_task));
    for (i = 1; i <= Route[0]; i++)
    {
        used_task[Route[i]] = 1;
        used_task[inst_tasks[Route[i]].inverse] = 1;
    }

    FHRoute[0] = 1;
    FHRoute[1] = 0;

    for (i = 1; i < EulerRoute[0]; i++)
    {
        if (TaskMatrix[EulerRoute[i]][EulerRoute[i+1]])
        {
            TaskMatrix[EulerRoute[i]][EulerRoute[i+1]] --;
            TaskMatrix[EulerRoute[i+1]][EulerRoute[i]] --;
            FHRoute[0] ++;
            // FHRoute[FHRoute[0]] = FindTask(EulerRoute[i], EulerRoute[i+1], inst_tasks, task_num);

            int task_id;
            for (int k = 1; k <= task_num; k++)
            {
                if (inst_tasks[k].head_node == EulerRoute[i] && inst_tasks[k].tail_node == EulerRoute[i+1])
                {
                    if (used_task[k])
                    {
                        task_id = k;
                        break;
                    }
                }
            }
            FHRoute[FHRoute[0]] = task_id;
        }
    }

    FHRoute[0] ++;
    FHRoute[FHRoute[0]] = 0;

    // ignore the virtual task, and insert them in the position where the outside vehicle is closest.
    if (virtualTask[0] > 0)
    {
        int insert_flag[FHRoute[0]];
        memset(insert_flag, 0, sizeof(insert_flag));
        int tmp;
        for (i = 1; i < virtualTask[0]; i++)
        {
            for (j = i+1; j <= virtualTask[0]; j++)
            {
                if (inst_tasks[virtualTask[i]].serv_cost < inst_tasks[virtualTask[j]].serv_cost)
                {
                    tmp = virtualTask[j];
                    virtualTask[j] = virtualTask[i];
                    virtualTask[i] = tmp;
                }
            }
        }

        for (i = 1; i <= virtualTask[0]; i++)
        {
            int currTask = virtualTask[i];
            int min_reduce_cost = INF, reduce_cost, insert_position;
            for (j = 1; j < FHRoute[0]; j++)
            {
                reduce_cost = min_cost[inst_tasks[FHRoute[j]].tail_node][inst_tasks[virtualTask[i]].head_node]
                 + min_cost[inst_tasks[virtualTask[i]].tail_node][inst_tasks[FHRoute[j+1]].head_node]
                 - min_cost[inst_tasks[FHRoute[j]].tail_node][inst_tasks[FHRoute[j+1]].head_node];
                if (reduce_cost < min_reduce_cost)
                {
                    min_reduce_cost = reduce_cost;
                    insert_position = j+1;
                }
            }
            add_element(FHRoute, virtualTask[i], insert_position);
        }
    }

    // if (virtualTask[0] == 1)
    // {
    //     int min_reduce_cost = INF, reduce_cost, insert_position;
    //     for (j = 1; j < FHRoute[0]; j++)
    //     {
    //         reduce_cost = min_cost[inst_tasks[FHRoute[j]].tail_node][inst_tasks[virtualTask[1]].head_node]
    //                       + min_cost[inst_tasks[virtualTask[1]].tail_node][inst_tasks[FHRoute[j+1]].head_node]
    //                       - min_cost[inst_tasks[FHRoute[j]].tail_node][inst_tasks[FHRoute[j+1]].head_node];
    //         if (reduce_cost < min_reduce_cost)
    //         {
    //             min_reduce_cost = reduce_cost;
    //             insert_position = j+1;
    //         }
    //     }
    //     int tmp_route[MAX_TASK_SEQ_LENGTH];
    //     memset(tmp_route, 0, sizeof(tmp_route));
    //     tmp_route[0] = 2;
    //     tmp_route[1] = 0;
    //     tmp_route[2] = virtualTask[1];

    //     for (i = insert_position+1; i < FHRoute[0]; i++)
    //     {
    //         tmp_route[0] ++;
    //         tmp_route[tmp_route[0]] = FHRoute[i];
    //     }
    //     for (i = 2; i <= insert_position; i++)
    //     {
    //         tmp_route[0] ++;
    //         tmp_route[tmp_route[0]] = FHRoute[i];
    //     }
    //     tmp_route[0] ++;
    //     tmp_route[tmp_route[0]] = 0;
    //     memcpy(FHRoute, tmp_route, sizeof(tmp_route));
    // }
}




void GetConnectivePiece(int Root, int CurrentPiece, int *PieceMark, int (*Neighbors)[MAX_NODE_TAG_LENGTH])
{
    int i, j;

    PieceMark[Root] = CurrentPiece;

    for (i = 1; i <= Neighbors[Root][0]; i++)
    {
        if (PieceMark[Neighbors[Root][i]] > 0)
            continue;

        GetConnectivePiece(Neighbors[Root][i], CurrentPiece, PieceMark, Neighbors);
    }
}

void GetMinCostTree( int (*CPMCTree)[MAX_NODE_TAG_LENGTH], int (*CPMinCost)[MAX_NODE_TAG_LENGTH])
{
    int i, j, k;
    int u, v;
    int MCost;

    for (i = 1; i <= vertex_num; i++)
    {
        for (j = 1; j <= vertex_num; j++)
        {
            CPMCTree[i][j] = 0;
        }
    }

    int ColoredArcs[CPMinCost[0][0]+1][CPMinCost[0][0]+1];
    int Visited[CPMinCost[0][0]+1];

    memset(ColoredArcs, 0, sizeof(ColoredArcs));
    memset(Visited, 0, sizeof(Visited));

    Visited[1] = 1;

    for (i = 1; i <= CPMinCost[0][0]; i++)
    {
        if (CPMinCost[1][i] < INF)
        {
            ColoredArcs[1][i] = 1;
        }

        if (CPMinCost[i][1] < INF)
        {
            ColoredArcs[i][1] = 1;
        }
    }

    for (k = 1; k < CPMinCost[0][0]; k++)
    {
        MCost = INF;

        for (i = 1; i <= CPMinCost[0][0]; i++)
        {
            for (j = 1; j <= CPMinCost[0][0]; j++)
            {
                if (ColoredArcs[i][j] == 1)
                {
                    if (CPMinCost[i][j] < MCost)
                    {
                        MCost = CPMinCost[i][j];
                        u = i;
                        v = j;
                    }
                }
            }
        }
        if (u < 0)
        {
            int a = 0;
            printf("Error in GetMinCostTree of FH");
            exit(0);
        }

        if (!Visited[u])
        {
            Visited[u] = 1;

            for (i = 1; i <= CPMinCost[0][0]; i++)
            {
                if (CPMinCost[u][i] < INF && !Visited[i])
                {
                    ColoredArcs[u][i] = 1;
                }

                if (CPMinCost[i][u] < INF && !Visited[i])
                {
                    ColoredArcs[i][u] = 1;
                }
            }
        }
        else
        {
            Visited[v] = 1;

            for (i = 1; i <= CPMinCost[0][0]; i++)
            {
                if (CPMinCost[v][i] < INF && !Visited[i])
                {
                    ColoredArcs[v][i] = 1;
                }

                if (CPMinCost[i][v] < INF && !Visited[i])
                {
                    ColoredArcs[i][v] = 1;
                }
            }
        }

        CPMCTree[u][v] = 1;
        CPMCTree[v][u] = 1;
        ColoredArcs[u][v] = 2;
        ColoredArcs[v][u] = 2;

        for (i = 1; i <= CPMinCost[0][0]; i++)
        {
            if (Visited[i])
                continue;

            MCost = INF;

            for (j = 1; j <= CPMinCost[0][0]; j++)
            {
                if (CPMinCost[i][j] < INF && Visited[j])
                {
                    ColoredArcs[i][j] = 0;

                    if (CPMinCost[i][j] < MCost)
                    {
                        MCost = CPMinCost[i][j];
                        u = i;
                        v = j;
                    }
                }
            }

            for (j = 1; j <= CPMinCost[0][0]; j++)
            {
                if (CPMinCost[j][i] < INF && Visited[j])
                {
                    ColoredArcs[j][i] = 0;

                    if (CPMinCost[j][i] < MCost)
                    {
                        MCost = CPMinCost[j][i];
                        u = j;
                        v = i;
                    }
                }
            }

            ColoredArcs[u][v] = 1;
            ColoredArcs[v][u] = 1;
        }
    }
}

void GetEulerRoute(int *EulerRoute, int (*AdMatrix)[MAX_NODE_TAG_LENGTH], int *Nodes)
{
    int i, j;
    int backplace;

    EulerRoute[0] = 0;

    FindCircuit(EulerRoute, 1, AdMatrix, Nodes);

    for (backplace = 1; backplace <= EulerRoute[0]; backplace++)
    {
        if (EulerRoute[backplace] == 1)
            break;
    }

    backplace --;

    int TmpER[EulerRoute[0]+1];
    AssignArray(EulerRoute, TmpER);

    for (i = 1; i <= EulerRoute[0]-backplace; i++)
    {
        EulerRoute[i] = TmpER[i+backplace];
    }

    for (i = 1; i <= backplace; i++)
    {
        EulerRoute[EulerRoute[0]-backplace+i] = TmpER[i+1];
    }
}

void FindCircuit(int *EulerRoute, int CurrentNode, int (*AdMatrix)[MAX_NODE_TAG_LENGTH], int *Nodes)
{
    int i, j, k;
    int Neighbors[Nodes[0]];
    Neighbors[0] = 0;

    for (i = 1; i <= Nodes[0]; i++)
    {
        if (AdMatrix[CurrentNode][Nodes[i]])
        {
            Neighbors[0] ++;
            Neighbors[Neighbors[0]] = Nodes[i];
        }
    }

    if (Neighbors[0] == 0)
    {
        EulerRoute[0] ++;
        EulerRoute[EulerRoute[0]] = CurrentNode;
    }
    else
    {
        while(Neighbors[0] > 0)
        {
            k = rand_choose(Neighbors[0]);

            AdMatrix[CurrentNode][Neighbors[k]] --;
            AdMatrix[Neighbors[k]][CurrentNode] --;

            FindCircuit(EulerRoute, Neighbors[k], AdMatrix, Nodes);

            Neighbors[0] = 0;
            for (i = 1; i <= Nodes[0]; i++)
            {
                if (AdMatrix[CurrentNode][Nodes[i]])
                {
                    Neighbors[0] ++;
                    Neighbors[Neighbors[0]] = Nodes[i];
                }
            }
        }

        EulerRoute[0] ++;
        EulerRoute[EulerRoute[0]] = CurrentNode;
    }
}

void EvenAllOddNodes(int (*AdMatrix)[MAX_NODE_TAG_LENGTH], int *OddNodes)
{
    int i, j, k;
    int LeftNodes[vertex_num+1];
    memset(LeftNodes, 0, sizeof(LeftNodes));
    AssignArray(OddNodes, LeftNodes);
//    for (i = 1; i <= OddNodes[0]; i++)
//    {
//        printf("%d \t", OddNodes[i]);
//    }
//    printf("1: %d, %d \n", OddNodes[0], LeftNodes[0]);

    struct Link
    {
        int LinkNode1Pos;
        int LinkNode2Pos;
        int LinkCost;
    };

    struct Link TmpLink, BestLink;

    for (i = 1; i <= OddNodes[0]/2; i++)
    {
        BestLink.LinkCost = INF;
        for (j = 1; j < LeftNodes[0]; j++)
        {
            for (k = j+1; k <= LeftNodes[0]; k++)
            {
                TmpLink.LinkNode1Pos = j;
                TmpLink.LinkNode2Pos = k;
                TmpLink.LinkCost = min_cost[LeftNodes[j]][LeftNodes[k]];
//                printf("%d %d %d\n", LeftNodes[j], LeftNodes[k], TmpLink.LinkCost);

                if (TmpLink.LinkCost < BestLink.LinkCost)
                    BestLink = TmpLink;
            }
        }

        //printf("Link %d and %d\n", LeftNodes[BestLink.LinkNode1Pos], LeftNodes[BestLink.LinkNode2Pos]);

        AdMatrix[LeftNodes[BestLink.LinkNode1Pos]][LeftNodes[BestLink.LinkNode2Pos]] ++;
        AdMatrix[LeftNodes[BestLink.LinkNode2Pos]][LeftNodes[BestLink.LinkNode1Pos]] ++;

//        printf("2: %d, %d, %d, %d \n", OddNodes[0], LeftNodes[0], BestLink.LinkNode1Pos, BestLink.LinkNode2Pos);
        if (BestLink.LinkNode1Pos < BestLink.LinkNode2Pos)
        {
            delete_element(LeftNodes, BestLink.LinkNode2Pos);
            delete_element(LeftNodes, BestLink.LinkNode1Pos);
        }
        else
        {
            delete_element(LeftNodes, BestLink.LinkNode1Pos);
            delete_element(LeftNodes, BestLink.LinkNode2Pos);
        }
    }
//    printf("End...\n");
}



















