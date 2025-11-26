import heapq

# ------------------------
# توابع کمکی
# ------------------------

def is_green(t):
    """چرخه ۲۰ دقیقه‌ای چراغ راهنمایی L"""
    return (t % 20) < 10

def wait_time_for_green(t):
    """اگر قرمز باشد، چند دقیقه صبر کنیم تا سبز شود"""
    cycle = t % 20
    if cycle < 10:
        return 0
    return 20 - cycle  # زمان تا شروع سبز

def cost_to_enter(cell, t):
    """محاسبه هزینه ورود به سلول با توجه به زمان"""
    if cell in ['S', 'G']:
        return 1
    if cell == 'L':
        if is_green(t):
            return 1
        else:
            return 10
    return int(cell)

# ------------------------
# UCS برای چند هدف
# ------------------------

def ucs_multi_goal(grid, start, goals):
    n, m = len(grid), len(grid[0])
    initial_state = (start[0], start[1], tuple(goals))  # r, c, remaining goals

    pq = []
    heapq.heappush(pq, (0, initial_state, []))  # total_cost, state, path
    visited = {}

    while pq:
        cost_so_far, (r, c, rem_goals), path = heapq.heappop(pq)

        # اگر همه اهداف رسیدیم
        if not rem_goals:
            return cost_so_far, path

        # جلوگیری از revisit غیرضروری
        key = (r, c, rem_goals)
        if key in visited and visited[key] <= cost_so_far:
            continue
        visited[key] = cost_so_far

        # اهداف جدید
        new_goals = tuple(g for g in rem_goals if g != (r, c))

        # حرکات مجاز
        moves = {
            "UP": (-1, 0),
            "DOWN": (1, 0),
            "LEFT": (0, -1),
            "RIGHT": (0, 1),
            "STAY": (0, 0)
        }

        for act, (dr, dc) in moves.items():
            nr, nc = r + dr, c + dc

            if act == "STAY":
                heapq.heappush(pq, (cost_so_far + 1, (r, c, new_goals), path + ["STAY"]))
                continue

            # داخل محدوده
            if 0 <= nr < n and 0 <= nc < m:
                cell = grid[nr][nc]

                # اگر L است و قرمز است، قبل از ورود باید صبر کنیم
                if cell == 'L' and not is_green(cost_so_far):
                    wait = wait_time_for_green(cost_so_far)
                    total_cost = cost_so_far + wait + 1
                    new_path = path + ["STAY"] * wait + [act]
                else:
                    enter_cost = cost_to_enter(cell, cost_so_far)
                    total_cost = cost_so_far + enter_cost
                    new_path = path + [act]

                heapq.heappush(pq, (total_cost, (nr, nc, new_goals), new_path))

    return None

# ------------------------
# ورودی و اجرای برنامه
# ------------------------

def scenario1():
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

    total_cost, actions = ucs_multi_goal(grid, start, goals)
    print("Cost:", total_cost-1)
    print("Actions:", actions)


if __name__ == "__main__":
    scenario1()
