import sys
import math
import heapq
import random
import itertools

# ---------------------------------------------------------
# 1. Map and Environment Logic
# ---------------------------------------------------------

class MapEnv:
    def __init__(self, grid_lines):
        self.rows = len(grid_lines)
        self.cols = len(grid_lines[0])
        self.grid = grid_lines
        self.ambulances = []  # List of (r, c, id_str)
        self.incidents = []   # List of (r, c, id_str)
        self._parse_objects()

    def _parse_objects(self):
        """
        Identifies S (Ambulances) and G (Incidents) based on the
        reading order specified: Left->Right, Top->Bottom.
        """
        s_count = 0
        g_count = 0
        
        # Determine coordinates and assign IDs based on reading order
        for r in range(self.rows):
            for c in range(self.cols):
                cell = self.grid[r][c]
                if cell == 'S':
                    s_count += 1
                    self.ambulances.append((r, c, f"S{s_count}"))
                elif cell == 'G':
                    g_count += 1
                    self.incidents.append((r, c, f"I{g_count}"))

    def get_movement_cost(self, r, c, current_time):
        """
        Calculates cost to ENTER cell (r, c) at current_time.
        """
        cell = self.grid[r][c]
        
        # S and G always cost 1
        if cell in ['S', 'G']:
            return 1
        
        # Traffic Light Logic
        if cell == 'L':
            # Cycle: 20 mins (0-9 Green, 10-19 Red)
            cycle_time = current_time % 20
            if cycle_time < 10:
                return 1  # Green
            else:
                return 10 # Red
        
        # Numeric traffic cost
        if cell.isdigit():
            return int(cell)
            
        # Default fallback (should not happen based on problem desc)
        return 1

    def is_valid(self, r, c):
        return 0 <= r < self.rows and 0 <= c < self.cols

# ---------------------------------------------------------
# 2. Time-Dependent A* Search (Helper for Fitness)
# ---------------------------------------------------------

def heuristic(a, b):
    """Manhattan distance heuristic."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def get_travel_time(start_pos, end_pos, start_time, map_env):
    """
    Runs A* with Time-Dependent States.
    State is defined by (row, col, time % 20) to allow 'waiting' for green lights.
    """
    # Priority Queue: (f_score, current_time, r, c)
    pq = []
    h_start = heuristic(start_pos, end_pos)
    heapq.heappush(pq, (start_time + h_start, start_time, start_pos[0], start_pos[1]))
    
    # Visited set needs to account for Traffic Light Cycles (20 mins)
    # Key: (r, c, time % 20), Value: min_time_reached
    min_time_at_state = {} 
    
    start_state = (start_pos[0], start_pos[1], start_time % 20)
    min_time_at_state[start_state] = start_time

    while pq:
        f, curr_time, r, c = heapq.heappop(pq)

        # Goal check
        if (r, c) == end_pos:
            return curr_time - start_time

        # State Pruning: Check if we found a faster way to this (cell + cycle_phase)
        cycle_phase = curr_time % 20
        state_key = (r, c, cycle_phase)
        
        if min_time_at_state.get(state_key, float('inf')) < curr_time:
            continue
        
        # Actions: Up, Down, Left, Right, Stay
        moves = [(-1, 0), (1, 0), (0, -1), (0, 1), (0, 0)] 

        for dr, dc in moves:
            nr, nc = r + dr, c + dc
            
            if map_env.is_valid(nr, nc):
                # Cost Logic
                if dr == 0 and dc == 0: # STAY
                    move_cost = 1
                else:
                    move_cost = map_env.get_movement_cost(nr, nc, curr_time)
                
                new_time = curr_time + move_cost
                new_cycle_phase = new_time % 20
                new_state_key = (nr, nc, new_cycle_phase)
                
                # Optimization: Only add if this is the best time for this specific cycle phase
                if new_time < min_time_at_state.get(new_state_key, float('inf')):
                    min_time_at_state[new_state_key] = new_time
                    h = heuristic((nr, nc), end_pos)
                    # Optimization: Prioritize staying if f-scores are equal but time is less? 
                    # Standard A* relies on f.
                    heapq.heappush(pq, (new_time + h, new_time, nr, nc))
                    
    return float('inf')
# ---------------------------------------------------------
# 3. Genetic Algorithm Logic
# ---------------------------------------------------------

def evaluate_chromosome(chromosome, map_env, cache):
    """
    Calculates fitness (makespan).
    Chromosome: List where index i is the Ambulance Index assigned to Incident i.
    Example: [0, 1, 0] means I1->S1, I2->S2, I3->S1.
    """
    num_ambulances = len(map_env.ambulances)
    num_incidents = len(map_env.incidents)
    
    # Group incidents by ambulance
    assignments = [[] for _ in range(num_ambulances)]
    for inc_idx, amb_idx in enumerate(chromosome):
        assignments[amb_idx].append(inc_idx)
    
    max_route_time = 0
    
    # Calculate route time for each ambulance
    for amb_idx, assigned_inc_indices in enumerate(assignments):
        if not assigned_inc_indices:
            continue
            
        start_node = map_env.ambulances[amb_idx][:2] # (r, c)
        current_time = 0
        current_pos = start_node
        
        # TSP Solver: Find best order for these incidents
        # Since N is usually small in this project, we use Permutations
        # If N > 6, we could switch to Nearest Neighbor to save time
        
        best_perm_time = float('inf')
        
        # Get coordinates of assigned incidents
        targets = [map_env.incidents[i][:2] for i in assigned_inc_indices]
        
        # Try all permutations of targets to find optimal route for this ambulance
        for perm in itertools.permutations(targets):
            sim_time = 0
            sim_pos = start_node
            possible = True
            
            for target in perm:
                # Check cache first
                cache_key = (sim_pos, target, sim_time)
                if cache_key in cache:
                    duration = cache[cache_key]
                else:
                    duration = get_travel_time(sim_pos, target, sim_time, map_env)
                    cache[cache_key] = duration
                
                if duration == float('inf'):
                    possible = False
                    break
                
                sim_time += duration
                sim_pos = target
            
            if possible:
                if sim_time < best_perm_time:
                    best_perm_time = sim_time
        
        if best_perm_time == float('inf'):
            return float('inf') # Invalid solution
            
        if best_perm_time > max_route_time:
            max_route_time = best_perm_time
            
    return max_route_time

def run_genetic_algorithm(map_env):
    num_incidents = len(map_env.incidents)
    num_ambulances = len(map_env.ambulances)
    
    if num_incidents == 0:
        return 0, [], {}

    # GA Parameters
    POPULATION_SIZE = 50
    GENERATIONS = 100
    MUTATION_RATE = 0.1
    ELITISM_COUNT = 2
    
    # Cache for A* results to speed up: {(start, end, start_time): duration}
    path_cache = {}

    # 1. Initial Population
    population = []
    for _ in range(POPULATION_SIZE):
        chrom = [random.randint(0, num_ambulances - 1) for _ in range(num_incidents)]
        population.append(chrom)

    for gen in range(GENERATIONS):
        # 2. Evaluate Fitness
        fitness_scores = []
        for chrom in population:
            fit = evaluate_chromosome(chrom, map_env, path_cache)
            fitness_scores.append(fit)
        
        # Sort population by fitness (Ascending because we want to minimize time)
        pop_sorted = sorted(zip(population, fitness_scores), key=lambda x: x[1])
        population = [x[0] for x in pop_sorted]
        fitness_scores = [x[1] for x in pop_sorted]
        
        best_fitness = fitness_scores[0]
        
        # Optional: Stop early if converged or perfect score (unlikely to know perfect score here)
        
        # 3. Selection & Crossover & Mutation
        new_population = []
        
        # Elitism
        new_population.extend(population[:ELITISM_COUNT])
        
        while len(new_population) < POPULATION_SIZE:
            # Tournament Selection
            parent1 = population[random.randint(0, POPULATION_SIZE//2)] # Bias towards better half
            parent2 = population[random.randint(0, POPULATION_SIZE//2)]
            
            # Crossover (Single Point)
            if num_incidents > 1:
                cut = random.randint(1, num_incidents - 1)
                child = parent1[:cut] + parent2[cut:]
            else:
                child = parent1[:]
            
            # Mutation
            if random.random() < MUTATION_RATE:
                idx_to_mutate = random.randint(0, num_incidents - 1)
                child[idx_to_mutate] = random.randint(0, num_ambulances - 1)
            
            new_population.append(child)
            
        population = new_population

    # Final Evaluation to get the best result details
    best_chrom = population[0]
    best_makespan = evaluate_chromosome(best_chrom, map_env, path_cache)
    
    return best_makespan, best_chrom, path_cache

# ---------------------------------------------------------
# 4. Output Formatting & Main Execution
# ---------------------------------------------------------

def print_detailed_result(map_env, best_chrom, best_makespan):
    print(f"Best makespan (minutes): {best_makespan:.2f}")
    
    num_ambulances = len(map_env.ambulances)
    assignments = [[] for _ in range(num_ambulances)]
    for inc_idx, amb_idx in enumerate(best_chrom):
        assignments[amb_idx].append(inc_idx)
    
    # For calculating individual routes again to print logic
    # We need to re-run the TSP logic to find the specific order that yielded the best time
    
    for amb_idx in range(num_ambulances):
        assigned_indices = assignments[amb_idx]
        assigned_ids = [map_env.incidents[i][2] for i in assigned_indices]
        assigned_ids.sort() # Just for display, though actual path order matters
        
        # Re-calculate best order to display coords
        start_node = map_env.ambulances[amb_idx][:2]
        targets_info = [(i, map_env.incidents[i][:2]) for i in assigned_indices] # (index, coord)
        
        best_order_coords = []
        min_time = float('inf')
        
        if not targets_info:
            route_time = 0.0
        else:
            # Try perms to find which one gave the optimal time
            for perm in itertools.permutations(targets_info):
                sim_time = 0
                sim_pos = start_node
                current_coords = []
                
                for idx, target_pos in perm:
                    duration = get_travel_time(sim_pos, target_pos, sim_time, map_env)
                    sim_time += duration
                    sim_pos = target_pos
                    current_coords.append(target_pos)
                
                if sim_time < min_time:
                    min_time = sim_time
                    best_order_coords = current_coords
            
            route_time = min_time

        # Formatting output string
        # S1 at (0, 0) assigned incidents: ['I3'] -> coords: [(2, 0)]
        s_pos = map_env.ambulances[amb_idx][:2]
        s_id = map_env.ambulances[amb_idx][2]
        
        # Format lists properly
        ids_str = str(assigned_ids).replace('"', "'")
        coords_str = str(best_order_coords)
        
        print(f"{s_id} at {s_pos} assigned incidents: {ids_str} -> coords: {coords_str}")
    
    # Print route times
    for amb_idx in range(num_ambulances):
        assigned_indices = assignments[amb_idx]
        # Recalculate time (redundant but cleaner code-wise than passing variable)
        start_node = map_env.ambulances[amb_idx][:2]
        targets_info = [(i, map_env.incidents[i][:2]) for i in assigned_indices]
        min_time = 0.0
        if targets_info:
            min_time = float('inf')
            for perm in itertools.permutations(targets_info):
                sim_time = 0
                sim_pos = start_node
                for idx, target_pos in perm:
                    duration = get_travel_time(sim_pos, target_pos, sim_time, map_env)
                    sim_time += duration
                    sim_pos = target_pos
                if sim_time < min_time:
                    min_time = sim_time
        
        s_id = map_env.ambulances[amb_idx][2]
        print(f"{s_id} route time = {float(min_time):.1f} minutes (visiting {len(assigned_indices)} incidents)")


def solve():
    # Reading input from Standard Input
    try:
        input_line = sys.stdin.read().split()
    except Exception:
        return

    if not input_line:
        return

    iterator = iter(input_line)
    try:
        rows = int(next(iterator))
        cols = int(next(iterator))
    except StopIteration:
        return

    grid = []
    for r in range(rows):
        row_str = next(iterator)
        grid.append(row_str)

    # Initialize Environment
    env = MapEnv(grid)

    # Run Genetic Algorithm
    makespan, best_chromosome, _ = run_genetic_algorithm(env)

    # Output Results
    print_detailed_result(env, best_chromosome, makespan)

if __name__ == "__main__":
    solve()