import heapq
from itertools import combinations
def is_green(t):
    return (t % 20) < 10
def wait_time_for_green(t):
    cycle = t % 20
    if cycle < 10:
        return 0
    return 20 - cycle

def cost_to_enter(cell, t):
    if cell in ['S', 'G']:
        return 1
    if cell == 'L':
        return 1 if is_green(t) else 10
    return int(cell)
def manhattan(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])
def mst_cost(points):
    if len(points) <= 1:
        return 0

    #Prim’s MST
    unvisited = set(points)
    current = unvisited.pop()
    total = 0

    while unvisited:
        next_point = min(unvisited, key=lambda p: manhattan(current, p))
        total += manhattan(current, next_point)
        current = next_point
        unvisited.remove(current)

    return total
#h function

def heuristic(curr, goals):
    if not goals:
        return 0
    goals = list(goals)
    h1 = min(manhattan(curr, g) for g in goals)
    h2 = mst_cost(goals)

    return h1 + h2
def astar_multi_goal(grid, start, goals):
    n, m = len(grid), len(grid[0])

    initial_state = (start[0], start[1], tuple(goals))

    pq = []
    heapq.heappush(pq, (0, 0, initial_state, []))  
    # (f = g+h, g, state, path)
    visited = {}
    while pq:
        f, g, (r, c, rem_goals), path = heapq.heappop(pq)

        #end
        if not rem_goals:
            return g, path

        key = (r, c, rem_goals)
        if key in visited and visited[key] <= g:
            continue
        visited[key] = g

        new_goals = tuple(gp for gp in rem_goals if gp != (r, c))

        moves = {
            "UP": (-1, 0),
            "DOWN": (1, 0),
            "LEFT": (0, -1),
            "RIGHT": (0, 1),
            "STAY": (0, 0),
        }
        for act, (dr, dc) in moves.items():
            if act == "STAY":
                gn = g + 1
                hn = heuristic((r, c), new_goals)
                heapq.heappush(pq, (gn + hn, gn, (r, c, new_goals), path + ["STAY"]))
                continue

            nr, nc = r + dr, c + dc
            if 0 <= nr < n and 0 <= nc < m:
                cell = grid[nr][nc]
                # اگر L و قرمز → STAY لازم
                if cell == 'L' and not is_green(g):
                    w = wait_time_for_green(g)
                    gn = g + w + 1
                    new_path = path + ["STAY"] * w + [act]
                else:
                    enter_cost = cost_to_enter(cell, g)
                    gn = g + enter_cost
                    new_path = path + [act]

                hn = heuristic((nr, nc), new_goals)
                heapq.heappush(pq, (gn + hn, gn, (nr, nc, new_goals), new_path))

    return None
def scenario2():
    n, m = map(int, input().split())
    grid = [list(input().strip()) for _ in range(n)]

    start = None
    goals = []
    for i in range(n):
        for j in range(m):
            if grid[i][j] == 'S':
                start = (i, j)
            if grid[i][j] == 'G':
                goals.append((i, j))

    cost, actions = astar_multi_goal(grid, start, goals)
    print("Cost:", cost-1)
    print("Actions:", actions)

if __name__ == "__main__":
    scenario2()
