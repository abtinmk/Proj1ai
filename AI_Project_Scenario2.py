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
        cell = self.grid[r][c]
        if cell in ['S', 'G']: return 1
        if cell == 'L':
            return 1 if (current_time % 20) < 10 else 10
        if cell.isdigit(): return int(cell)
        return 1

    def is_valid(self, r, c):
        return 0 <= r < self.rows and 0 <= c < self.cols

# ---------------------------------------------------------
# 2. A* Algorithm Logic
# ---------------------------------------------------------
class AStarSolver:
    def __init__(self, env):
        self.env = env

    def manhattan(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def mst_cost(self, goals):
        """Calculates MST cost for goals using Prim's Algorithm."""
        if len(goals) <= 1:
            return 0
        
        points = list(goals)
        unvisited = set(points)
        start_node = points[0]
        unvisited.remove(start_node)
        visited = {start_node}
        mst_len = 0

        while unvisited:
            min_dist = float('inf')
            next_node = None
            
            for u in visited:
                for v in unvisited:
                    d = self.manhattan(u, v)
                    if d < min_dist:
                        min_dist = d
                        next_node = v
            
            if next_node:
                mst_len += min_dist
                visited.add(next_node)
                unvisited.remove(next_node)
            else:
                break
        return mst_len

    def heuristic(self, curr_pos, goals):
        """h(n) = Dist to nearest goal + MST of remaining goals"""
        if not goals:
            return 0
        min_dist = min([self.manhattan(curr_pos, g) for g in goals])
        return min_dist + self.mst_cost(goals)

    def solve(self):
        if not self.env.ambulances:
            return float('inf'), []

        start_pos = self.env.ambulances[0]
        initial_goals = tuple(sorted(self.env.incidents))
        initial_state = (start_pos[0], start_pos[1], initial_goals)
        
        # PQ: (f_score, g_score, state, path)
        pq = [(0, 0, initial_state, [])]
        
        # Visited: (r, c, goals, cycle_phase) -> min_g
        visited = {}

        while pq:
            f, g, (r, c, rem_goals), path = heapq.heappop(pq)

            if (r, c) in rem_goals:
                rem_goals = tuple(tgt for tgt in rem_goals if tgt != (r, c))
            
            if not rem_goals:
                return g, path

            cycle_phase = g % 20
            state_key = (r, c, rem_goals, cycle_phase)
            
            if state_key in visited and visited[state_key] <= g:
                continue
            visited[state_key] = g

            moves = [("UP", -1, 0), ("DOWN", 1, 0), ("LEFT", 0, -1), ("RIGHT", 0, 1), ("STAY", 0, 0)]

            for action, dr, dc in moves:
                nr, nc = r + dr, c + dc
                
                if self.env.is_valid(nr, nc):
                    step_cost = 0
                    if action == "STAY":
                        step_cost = 1
                    else:
                        step_cost = self.env.get_movement_cost(nr, nc, g)
                    
                    new_g = g + step_cost
                    h = self.heuristic((nr, nc), rem_goals)
                    new_f = new_g + h
                    
                    new_cycle = new_g % 20
                    new_state_key = (nr, nc, rem_goals, new_cycle)

                    if new_state_key not in visited or visited[new_state_key] > new_g:
                        heapq.heappush(pq, (new_f, new_g, (nr, nc, rem_goals), path + [action]))

        return float('inf'), []

# ---------------------------------------------------------
# 3. Main Execution
# ---------------------------------------------------------
def solve():
    print("--- Scenario 2: Informed Search (A*) ---")
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
        solver = AStarSolver(env)
        
        cost, actions = solver.solve()
        
        print(f"Cost: {cost}")
        print(f"Actions: {actions}")

    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    solve()