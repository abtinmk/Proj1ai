# scenario2_informed_fixed.py
# A* for multi-goal ambulance routing with a stronger admissible heuristic.
# Input format:
# First line: r c
# Next r lines: each line is a string of length c containing chars S, G, L, or digits 1-9
# Example:
# 3 5
# S9679
# 917LG
# 19999

from heapq import heappush, heappop
import math
import sys

# ---------- Input ----------
def read_input():
    try:
        r, c = map(int, input().split())
    except Exception:
        print("Invalid dimensions line. Expected: r c")
        sys.exit(1)
    grid = []
    for _ in range(r):
        line = input().rstrip("\n")
        if len(line) != c:
            print("Invalid row length. Expected", c, "chars but got:", len(line))
            sys.exit(1)
        grid.append(list(line))
    return grid

grid = read_input()
R = len(grid)
C = len(grid[0]) if R>0 else 0

# ---------- locate start and goals ----------
start = None
goals = []
for i in range(R):
    for j in range(C):
        if grid[i][j] == 'S':
            start = (i,j)
        elif grid[i][j] == 'G':
            goals.append((i,j))

if start is None:
    print("Start 'S' not found.")
    sys.exit(1)
if len(goals) == 0:
    print("No goals 'G' found.")
    sys.exit(1)

goal_index = {g:i for i,g in enumerate(goals)}
ALL_GOALS_MASK = (1 << len(goals)) - 1

MOVES = {"UP":(-1,0),"DOWN":(1,0),"LEFT":(0,-1),"RIGHT":(0,1),"STAY":(0,0)}
def in_bounds(r,c): return 0 <= r < R and 0 <= c < C

# ---------- costs ----------
def real_cell_entry_cost(cell, current_time):
    if cell == 'L':
        return 1 if (current_time % 20) < 10 else 10
    if cell in ('S','G'):
        return 1
    return int(cell)

def optimistic_cell_cost(cell):
    # optimistic (best-case) cost used for heuristic precomputation
    if cell == 'L':
        return 1
    if cell in ('S','G'):
        return 1
    return int(cell)

# ---------- Dijkstra (optimistic) ----------
def dijkstra_from(source):
    sr, sc = source
    INF = 10**12
    dist = [[INF]*C for _ in range(R)]
    dist[sr][sc] = 0
    heap = [(0, sr, sc)]
    while heap:
        d, r, c = heappop(heap)
        if d != dist[r][c]:
            continue
        for dr, dc in ((1,0),(-1,0),(0,1),(0,-1)):
            nr, nc = r+dr, c+dc
            if not in_bounds(nr,nc): 
                continue
            nd = d + optimistic_cell_cost(grid[nr][nc])
            if nd < dist[nr][nc]:
                dist[nr][nc] = nd
                heappush(heap, (nd, nr, nc))
    return dist

# Precompute optimistic distances TO each goal (so opt_dist(cell, goal) is available)
pre_to_goal = {}   # pre_to_goal[goal] = dist_matrix (R x C)
for g in goals:
    pre_to_goal[g] = dijkstra_from(g)

def opt_dist(cell, goal):
    # optimistic minimal cost from cell -> goal (using precomputed map from goal)
    # if unreachable in optimistic graph, return a very large number
    d = pre_to_goal[goal][cell[0]][cell[1]]
    return d

# Pairwise optimistic distances between goals (for MST)
pairwise_goal = [[0]*len(goals) for _ in range(len(goals))]
for i,g1 in enumerate(goals):
    for j,g2 in enumerate(goals):
        # distance from g1 to g2 (optimistic): look up pre_to_goal[g2][g1]
        pairwise_goal[i][j] = pre_to_goal[g2][g1[0]][g1[1]] if False else pre_to_goal[g2][g1[0]][g1[1]]  # fallback corrected below

# The above line used a placeholder; correct construction:
for i,g1 in enumerate(goals):
    for j,g2 in enumerate(goals):
        pairwise_goal[i][j] = pre_to_goal[g1][g2[0]][g2[1]]  # optimistic distance g1 -> g2

# ---------- MST (Prim) over remaining goals using pairwise_goal ----------
def mst_cost_of_goal_indices(indices):
    # indices: list of goal indices
    if len(indices) <= 1:
        return 0
    n = len(indices)
    used = [False]*n
    dist = [math.inf]*n
    dist[0] = 0
    total = 0
    for _ in range(n):
        # pick min unused
        mn = math.inf; idx = -1
        for k in range(n):
            if not used[k] and dist[k] < mn:
                mn = dist[k]; idx = k
        if idx == -1:
            break
        used[idx] = True
        if idx != 0:
            total += dist[idx]
        # update
        for j in range(n):
            if not used[j]:
                a = indices[idx]
                b = indices[j]
                cand = pairwise_goal[a][b]
                if cand < dist[j]:
                    dist[j] = cand
    return total

# ---------- Heuristic ----------
def heuristic(r, c, mask):
    # remaining goals
    rem_indices = [idx for pos, idx in goal_index.items() if not (mask & (1<<idx))]
    if not rem_indices:
        return 0
    # min optimistic distance from current cell to any remaining goal
    min_to_goal = math.inf
    for idx in rem_indices:
        gpos = goals[idx]
        d = opt_dist((r,c), gpos)
        if d < min_to_goal:
            min_to_goal = d
    # MST cost among remaining goals (use their indices)
    mst = mst_cost_of_goal_indices(rem_indices) if len(rem_indices) > 1 else 0
    # if any optimistic distance is INF -> unreachable in optimistic graph -> return large number
    if min_to_goal >= 10**12:
        return 10**12
    return min_to_goal + mst

# ---------- A* search ----------
start_mask = 0
if start in goal_index:
    start_mask = 1 << goal_index[start]
start_state = (start[0], start[1], start_mask)

open_heap = []
counter = 0
g0 = 0
h0 = heuristic(start[0], start[1], start_mask)
heappush(open_heap, (g0 + h0, counter, g0, start_state, None, None))
counter += 1

best_g = {start_state: 0}
parents = {}   # state -> (parent_state, action, g_at_state)
log_lines = []
push_count = 1
pop_count = 0
expanded = 0
solution = None
MAX_ITERS = 10**7

log_path = "scenario2_informed_log.txt"

while open_heap and expanded < MAX_ITERS:
    f, _, g, state, parent_state, action = heappop(open_heap)
    r, c, mask = state
    pop_count += 1
    log_lines.append(f"POP: state={state} g={g} f={f}")
    # stale check
    if g != best_g.get(state, float('inf')):
        continue
    if parent_state is not None:
        parents[state] = (parent_state, action, g)
    expanded += 1
    if mask == ALL_GOALS_MASK:
        solution = (state, g)
        break
    # successors
    for mv, (dr, dc) in MOVES.items():
        nr, nc = r + dr, c + dc
        if mv != "STAY" and not in_bounds(nr, nc):
            continue
        if mv == "STAY":
            enter = 1
            nr, nc = r, c
        else:
            enter = real_cell_entry_cost(grid[nr][nc], g)
        ng = g + enter
        nmask = mask
        if (nr, nc) in goal_index:
            nmask |= 1 << goal_index[(nr, nc)]
        new_state = (nr, nc, nmask)
        if ng < best_g.get(new_state, float('inf')):
            best_g[new_state] = ng
            h = heuristic(nr, nc, nmask)
            nf = ng + h
            heappush(open_heap, (nf, counter, ng, new_state, state, mv))
            counter += 1
            push_count += 1
            log_lines.append(f"PUSH: state={new_state} g={ng} f={nf} action={mv} h={h}")

# save log
with open(log_path, "w", encoding="utf8") as f:
    for L in log_lines:
        f.write(L + "\n")

# reconstruct and print solution
if solution:
    st, total_time = solution
    path = []
    cur = st
    while cur != start_state:
        p_state, act, g_at = parents[cur]
        path.append((act, cur, g_at))
        cur = p_state
    path.reverse()
    print("Cost", total_time)
    print("Steps (action, state_after_action, time_after_move):")
    for step in path:
        print(step)
    print(f"\nLog saved to: {log_path}")
else:
    print("No solution found within iteration limit.")
    print(f"Expanded: {expanded}, Pops: {pop_count}, Pushes: {push_count}")
