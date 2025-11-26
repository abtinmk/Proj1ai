import heapq

MOVES = {'UP': (-1,0), 'DOWN': (1,0), 'LEFT': (0,-1), 'RIGHT': (0,1), 'STAY': (0,0)}

def parse_input():
    R, C = map(int, input().split())
    grid = [list(input().strip()) for _ in range(R)]
    start = None
    goals = set()
    for r,row in enumerate(grid):
        for c,val in enumerate(row):
            if val=='S':
                start = (r,c)
            elif val=='G':
                goals.add((r,c))
    return grid, start, goals, R, C

def cell_cost(cell, current_time):
    if cell in ('S','G'):
        return 1
    elif cell=='L':
        cycle = current_time % 20
        if cycle<10:
            return 1
        else:
            return 10
    else:
        return int(cell)

def in_bounds(r,c,R,C):
    return 0<=r<R and 0<=c<C

def manhattan(p1,p2):
    return abs(p1[0]-p2[0]) + abs(p1[1]-p2[1])

def A_star(grid, start, goals, R, C):
    pq = []
    visited = dict()
    initial_f = min(manhattan(start,g) for g in goals)
    heapq.heappush(pq, (initial_f, 0, start, frozenset(goals), []))  # f, g, pos, remaining_goals, actions

    while pq:
        f, g, (r,c), rem_goals, actions = heapq.heappop(pq)

        if (r,c) in rem_goals:
            rem_goals = rem_goals - {(r,c)}

        if not rem_goals:
            return g, actions

        key = (r,c,rem_goals)
        if key in visited and visited[key]<=g:
            continue
        visited[key]=g

        for move_name,(dr,dc) in MOVES.items():
            nr,nc = r+dr, c+dc
            if in_bounds(nr,nc,R,C):
                cost = cell_cost(grid[nr][nc], g)
                if grid[nr][nc]=='L':
                    cycle = g%20
                    if cycle>=10:
                        wait = 20-cycle
                        new_g = g+wait+1
                        new_f = new_g + (min(manhattan((nr,nc),g2) for g2 in rem_goals) if rem_goals else 0)
                        heapq.heappush(pq, (new_f, new_g, (nr,nc), rem_goals, actions+['STAY']*wait+[move_name]))
                        continue
                new_g = g+cost
                new_f = new_g + (min(manhattan((nr,nc),g2) for g2 in rem_goals) if rem_goals else 0)
                heapq.heappush(pq, (new_f, new_g, (nr,nc), rem_goals, actions+[move_name]))
            elif move_name=='STAY':
                heapq.heappush(pq, (f+1, g+1, (r,c), rem_goals, actions+['STAY']))
    return None,None

# اجرای برنامه
grid, start, goals, R, C = parse_input()
cost, actions = A_star(grid, start, goals, R, C)
print("Cost:", cost)
print("Actions:", actions)
