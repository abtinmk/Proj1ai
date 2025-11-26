import sys
import heapq

# ---------------------------------------------------------
# 1. Map and Environment Logic
# ---------------------------------------------------------
class MapEnv:
    def __init__(self, grid_lines):
        self.rows = len(grid_lines)
        self.cols = len(grid_lines[0])
        self.grid = grid_lines
        self.ambulances = []
        self.incidents = []
        self._parse_objects()

    def _parse_objects(self):
        for r in range(self.rows):
            for c in range(self.cols):
                cell = self.grid[r][c]
                if cell == 'S':
                    self.ambulances.append((r, c))
                elif cell == 'G':
                    self.incidents.append((r, c))

    def get_movement_cost(self, r, c, current_time):
        """
        Calculates cost to ENTER cell (r, c) at current_time.
        """
        cell = self.grid[r][c]
        
        if cell in ['S', 'G']:
            return 1
        
        if cell == 'L':
            # Cycle: 20 mins (0-9 Green, 10-19 Red)
            if (current_time % 20) < 10:
                return 1  # Green
            else:
                return 10 # Red
        
        if cell.isdigit():
            return int(cell)
            
        return 1

    def is_valid(self, r, c):
        return 0 <= r < self.rows and 0 <= c < self.cols

# ---------------------------------------------------------
# 2. UCS Algorithm Logic
# ---------------------------------------------------------
class UCSSolver:
    def __init__(self, env):
        self.env = env

    def solve(self):
        if not self.env.ambulances:
            return float('inf'), []
            
        start_pos = self.env.ambulances[0]
        goals = tuple(sorted(self.env.incidents))
        
        # Priority Queue: (cost, state, path)
        # State: (r, c, remaining_goals)
        initial_state = (start_pos[0], start_pos[1], goals)
        pq = [(0, initial_state, [])]
        
        # Visited: (r, c, goals, cycle_phase) -> min_cost
        visited = {} 

        while pq:
            cost, (r, c, rem_goals), path = heapq.heappop(pq)

            # 1. Check Goal on current node
            if (r, c) in rem_goals:
                rem_goals = tuple(g for g in rem_goals if g != (r, c))
            
            # 2. Termination
            if not rem_goals:
                return cost, path

            # 3. Visited Check (Pruning with Cycle Phase)
            cycle_phase = cost % 20
            state_key = (r, c, rem_goals, cycle_phase)
            
            if state_key in visited and visited[state_key] <= cost:
                continue
            visited[state_key] = cost

            # 4. Expand Neighbors
            moves = [
                ("UP", -1, 0), ("DOWN", 1, 0), 
                ("LEFT", 0, -1), ("RIGHT", 0, 1), 
                ("STAY", 0, 0)
            ]

            for action, dr, dc in moves:
                nr, nc = r + dr, c + dc
                
                if self.env.is_valid(nr, nc):
                    step_cost = 0
                    if action == "STAY":
                        step_cost = 1
                    else:
                        step_cost = self.env.get_movement_cost(nr, nc, cost)
                    
                    new_cost = cost + step_cost
                    new_path = path + [action]
                    
                    # Optimization: Check visited before push
                    new_cycle = new_cost % 20
                    new_state_key = (nr, nc, rem_goals, new_cycle)
                    
                    if new_state_key not in visited or visited[new_state_key] > new_cost:
                        heapq.heappush(pq, (new_cost, (nr, nc, rem_goals), new_path))

        return float('inf'), []

# ---------------------------------------------------------
# 3. Main Execution
# ---------------------------------------------------------
def solve():
    print("--- Scenario 1: Uninformed Search (UCS) ---")
    print("Enter Input (Dimensions first, then map):")
    
    try:
        line = sys.stdin.readline()
        while line and not line.strip():
            line = sys.stdin.readline()
        if not line: return
        
        rows, cols = map(int, line.split())
        
        grid = []
        for _ in range(rows):
            row_str = sys.stdin.readline().strip()
            while not row_str:
                row_str = sys.stdin.readline().strip()
            grid.append(row_str)

        env = MapEnv(grid)
        solver = UCSSolver(env)
        
        cost, actions = solver.solve()
        
        print(f"Cost: {cost}")
        print(f"Actions: {actions}")

    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    solve()