def cost_to_enter(cell,current_time):
    if cell=='S' or cell== 'G':
        return 1
    if cell=='L':
        if current_time %20<10:
            return 1
        else:
            return 10
    if cell.isdigit():
        return int(cell)
    return float('inf')
#Custom Priority Queue
class PriorityQueue:
    def __init__(self):
        self.data = []

    def push(self, item):
        self.data.append(item)
    def pop(self):
        #minimum
        best_i = 0
        best_cost = self.data[0][0]
        for i in range(1, len(self.data)):
            if self.data[i][0] < best_cost:
                best_i = i
                best_cost = self.data[i][0]
        return self.data.pop(best_i)

    def empty(self):
        return len(self.data) == 0
#UCS
def solve(grid, R, C):
    G_positions = []
    start = None

    for r in range(R):
        for c in range(C):
            if grid[r][c] == 'S':
                start = (r, c)
            if grid[r][c] == 'G':
                G_positions.append((r, c))

    g_count = len(G_positions)
    g_index = {G_positions[i]: i for i in range(g_count)}
    ALL_VISITED = (1 << g_count) - 1

    start_mask = 0
    if start in g_index:
        start_mask = 1 << g_index[start]

    pq = PriorityQueue()
    pq.push((0, start, start_mask, []))

    visited_best_cost = {}
    states_expanded = 0

    MOVES = [
        (0, 1, 'RIGHT'),
        (0, -1, 'LEFT'),
        (-1, 0, 'UP'),
        (1, 0, 'DOWN'),
        (0, 0, 'STAY')
    ]

    while not pq.empty():
        cost, (r, c), mask, actions = pq.pop()
        states_expanded += 1

        if mask == ALL_VISITED:
            print("Cost:",cost)
            print("Actions:", actions)
            print("States Expanded:", states_expanded)
            return

        key = (r, c, mask)
        if key in visited_best_cost and visited_best_cost[key] <= cost:
            continue
        visited_best_cost[key] = cost

        for dr, dc, name in MOVES:
            nr, nc = r + dr, c + dc

            if name == 'STAY':
                new_cost = cost + 1
                pq.push((new_cost, (r, c), mask, actions + [name]))
                continue

            if 0<= nr< R and 0 <= nc < C:
                cell = grid[nr][nc]
                move_cost = cost_to_enter(cell, cost)
                new_cost = cost + move_cost
                new_mask = mask
                if (nr, nc) in g_index:
                    new_mask |=(1<<g_index[(nr, nc)])
                pq.push((new_cost,(nr, nc), new_mask,actions+[name]))
if __name__ == "__main__":
    R, C = map(int, input().split())
    grid = [list(input().strip()) for _ in range(R)]
    solve(grid, R, C)