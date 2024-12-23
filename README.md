### Max-Min Ant Colony Optimization for CVRP

#### Overview
This project implements a Max-Min Ant Colony Optimization (ACO) algorithm in C to solve the Capacitated Vehicle Routing Problem (CVRP). The CVRP is a combinatorial optimization problem where vehicles must deliver goods to customers while adhering to capacity constraints. The algorithm uses ant colony principles to find efficient routes that minimize the total cost (distance).

#### Highlights
- Customizable parameters such as the number of ants, iterations, pheromone influence, and evaporation rates.
- Advanced pheromone management for efficient convergence to optimal solutions.
- Standardized input using TSPLIB-formatted files, ensuring compatibility with recognized problem specifications.
- Comprehensive outputs including the best tour and its cost.

#### Requirements
- C compiler (e.g., GCC)
- TSPLIB problem files (.vrp format)

#### Installation
1. Clone this repository:
   ```bash
   git clone https://github.com/Vamin-co/maxMin-aco-cvrp.git
   cd maxMin-aco-cvrp
   ```
2. Compile the code:
   ```bash
   gcc -o maxmin_aco MaxMinACO_CVRP_VA.c -lm
   ```

#### Usage
Run the program with a `.vrp` file as input:
```bash
./maxmin_aco example.vrp
```
Replace `example.vrp` with the path to your TSPLIB problem file.

#### Configuration
Customize the algorithm by modifying parameters in the `main` function:
- **Number of Ants (`aco.num_ants`)**
- **Number of Iterations (`aco.num_iterations`)**
- **Pheromone Influence (`aco.alpha`)**
- **Heuristic Influence (`aco.beta`)**
- **Pheromone Evaporation Rate (`aco.rho`)**
- **Pheromone Min/Max (`aco.pheromone_min` / `aco.pheromone_max`)**

#### Results
The program outputs the best tour and its cost after execution. Example:
```
Best tour: 0 3 2 5 1 0
Best cost: 123.45
```

#### File Structure
- `MaxMinACO_CVRP_VA.c`: Main implementation file.

#### License
This project is licensed under the [GPL 3.0 License](LICENSE).

#### Contributions
Contributions are welcome! Submit issues or pull requests to improve the project.

#### Acknowledgments
- **Ant Colony Optimization**: Inspired by Marco Dorigo’s pioneering work on ACO algorithms.
- **TSPLIB**: Used as a standard for vehicle routing problem definitions.

