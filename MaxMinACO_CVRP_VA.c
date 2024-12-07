#include <stdio.h> 
#include <stdlib.h> 
#include <math.h> 
#include <limits.h> 
#include <time.h> 
#include <string.h> 

#define MAX_NODES 100 // max number of cities
#define MAX_ANTS 300 // max number of ants


typedef struct {
    int tour[MAX_NODES]; // cities visited by the ant
    float cost; // distance of the ant's tour
} Ant;

typedef struct {
    int num_ants; // number of ants
    int num_iterations; // number of iterations
    float alpha; // influence of pheromone
    float beta; // influence of heuristic desirability
    float rho; // pheromone evaporation rate
    float pheromone_min; // minimum pheromone level
    float pheromone_max; // maximum pheromone level
    float pheromone[MAX_NODES][MAX_NODES]; // pheromone levels on the edges between nodes
    int graph[MAX_NODES][MAX_NODES]; // distance matrix representing the problem graph
    int num_nodes; // number of cities
    int vehicle_capacity; // maximum load a vehicle can carry
    int demand[MAX_NODES]; // demand for each city
} MaxMinACO;

// initialize the pheromone levels between nodes to the maximum value
void initialize_pheromone(MaxMinACO *aco) {
    for (int i = 0; i < aco->num_nodes; i++) {
        for (int j = 0; j < aco->num_nodes; j++) {
            aco->pheromone[i][j] = aco->pheromone_max;
        }
    }
}

// construct a solution for an ant
void construct_solution(MaxMinACO *aco, Ant *ant) {
    int unvisited[MAX_NODES]; // track whether a city has been visited
    int num_unvisited = aco->num_nodes - 1; // number of nodes to be visited minus the depot
    for (int i = 1; i < aco->num_nodes; i++) {
        unvisited[i] = 1; // all cities are unvisited
    }

    int start_node = 0; // start from the depot
    ant->tour[0] = start_node;
    int tour_index = 1; // track the position in the tour
    int current_node = start_node;
    ant->cost = 0.0; // initialize the cost of the tour
    int load = 0; // current load of the vehicle

    while (num_unvisited > 0) {
        int next_node = -1;
        float probabilities[MAX_NODES]; // probabilities of choosing the next node
        float total_prob = 0.0; // total probability for normalization

        // calculate probabilities of visiting each unvisited node
        for (int j = 1; j < aco->num_nodes; j++) {
            if (unvisited[j] && (load + aco->demand[j] <= aco->vehicle_capacity)) {
                float tau = powf(aco->pheromone[current_node][j], aco->alpha); // influence of pheromone level
                float eta = powf(1.0 / aco->graph[current_node][j], aco->beta); // influence of heuristic (1/distance)
                probabilities[j] = tau * eta; // probability of choosing the next node based on pheromone (tau) and heuristic (eta)
                total_prob += probabilities[j];
            } else {
                probabilities[j] = 0.0; // node is either visited or exceeds capacity
            }
        }

        // select nect node (roulette wheel selection)
        if (total_prob > 0.0) {
            float r = ((float)rand() / (RAND_MAX)) * total_prob; // random value for selection
            float cum_prob = 0.0; 
            for (int j = 1; j < aco->num_nodes; j++) {
                if (unvisited[j]) {
                    cum_prob += probabilities[j];
                    if (r <= cum_prob) {
                        next_node = j;
                        break;
                    }
                }
            }
        }

        // if no suitable node was found, return to the depot
        if (next_node == -1) {
            ant->tour[tour_index++] = start_node; // add depot to the tour
            ant->cost += aco->graph[current_node][start_node]; // update the cost
            current_node = start_node; // reset to depot
            load = 0; // reset load after returning to the depot
            continue;
        }

        // update ant's tour and cost
        ant->tour[tour_index++] = next_node; // add next node to the tour
        ant->cost += aco->graph[current_node][next_node]; // update the cost and load
        load += aco->demand[next_node]; 
        unvisited[next_node] = 0; // mark node as visited
        num_unvisited--; 
        current_node = next_node; 
    }

    // return to the depot after the last customer
    ant->tour[tour_index++] = start_node; 
    ant->cost += aco->graph[current_node][start_node]; 
}

void update_pheromones(MaxMinACO *aco, Ant *ants) {
    // evaporate the pheromone 
    for (int i = 0; i < aco->num_nodes; i++) {
        for (int j = 0; j < aco->num_nodes; j++) {
            // decrease pheromone levels with factor (1-rho)[evaporation]
            aco->pheromone[i][j] *= (1 - aco->rho);

            // keep pheromones above pheromone_min
            if (aco->pheromone[i][j] < aco->pheromone_min) {
                aco->pheromone[i][j] = aco->pheromone_min;
            }
        }
    }

    // deposit pheromone based on ant tours
    for (int k = 0; k < aco->num_ants; k++) {
        for (int i = 0; i < aco->num_nodes - 1; i++) {
            int from = ants[k].tour[i];
            int to = ants[k].tour[i + 1];
            // add pheromone inversely proportional to cost
            aco->pheromone[from][to] += 1.0 / ants[k].cost; 
            // limit pheromone levels to pheromone_max
            if (aco->pheromone[from][to] > aco->pheromone_max) {
                aco->pheromone[from][to] = aco->pheromone_max; 
            }
        }
    }
}

// parse the TSPLIB file to get data
void parse_tsplib_file(const char *filename, MaxMinACO *aco) {
    FILE *file = fopen(filename, "r"); 
    if (file == NULL) {
        perror("Error opening file"); 
        return;
    }

    char line[256]; // buffer
    while (fgets(line, sizeof(line), file)) { 
        if (strstr(line, "DIMENSION")) {
            sscanf(line, "DIMENSION : %d", &aco->num_nodes); 
        } else if (strstr(line, "CAPACITY")) {
            sscanf(line, "CAPACITY : %d", &aco->vehicle_capacity); 
        } else if (strstr(line, "EDGE_WEIGHT_SECTION")) {
            for (int i = 0; i < aco->num_nodes; i++) { 
                for (int j = 0; j <= i; j++) {
                    fscanf(file, "%d", &aco->graph[i][j]); 
                    aco->graph[j][i] = aco->graph[i][j]; 
                }
            }
        } else if (strstr(line, "DEMAND_SECTION")) {
            for (int i = 0; i < aco->num_nodes; i++) { 
                int node, demand;
                fscanf(file, "%d %d", &node, &demand); 
                aco->demand[node - 1] = demand; // store demand (adjusting for 0-based index)
            }
        }
    }

    fclose(file); 
}

void max_min_aco(MaxMinACO *aco) {
    Ant ants[MAX_ANTS];
    Ant best_ant;
    best_ant.cost = INT_MAX;

    initialize_pheromone(aco); // initialize pheromone levels

    // iterate over num_iterations
    for (int iter = 0; iter < aco->num_iterations; iter++) {
        // each ant constructs a solution, and the best solution (best_ant) is tracked
        for (int i = 0; i < aco->num_ants; i++) {
            construct_solution(aco, &ants[i]); 
            if (ants[i].cost < best_ant.cost) { 
                best_ant = ants[i];
            }
        }

        // update pheromone
        update_pheromones(aco, ants); 
    }

    // print the best tour and its cost
    printf("Best tour: ");
    for (int i = 0; i <= aco->num_nodes; i++) {
        printf("%d ", best_ant.tour[i]);
    }
    printf("\nBest cost: %f\n", best_ant.cost);
}


int main(int argc, char *argv[]) {
    if (argc != 2) { 
        fprintf(stderr, "Usage: %s <fileName.vrp>\n", argv[0]);
        return 1; 
    }

    srand(time(NULL)); 
    MaxMinACO aco; 
    aco.num_ants = 10; 
    aco.num_iterations = 100; 
    aco.alpha = 1.0; 
    aco.beta = 5.0; 
    aco.rho = 0.5; // set pheromone evaporation rate
    aco.pheromone_min = 0.1; // set minimum pheromone level
    aco.pheromone_max = 10.0; // set maximum pheromone level

    parse_tsplib_file(argv[1], &aco); 
    max_min_aco(&aco); 

    return 0;
}
