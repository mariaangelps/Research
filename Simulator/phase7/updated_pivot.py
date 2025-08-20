import pygame
import random
import math
from collections import deque, defaultdict
from class_robot import Robot
from class_source_and_demand import Source, Demand

ARENA_WIDTH, ARENA_HEIGHT = 600, 300
ROBOT_RADIUS = 5
N_ROBOTS = 60
N_DEMANDS = 2
CONNECTION_DISTANCE = 80

obstacles = []  # intentionally unused

# -------------------------
# Basic node wrapper
# -------------------------
class Node:
    def __init__(self, name, x, y, color):
        self.name = name
        self.x = x
        self.y = y
        self.color = color

    def draw(self, screen):
        pygame.draw.circle(screen, self.color, (int(self.x), int(self.y)), 12)
        font = pygame.font.Font(None, 24)
        label = font.render(self.name, True, (0, 0, 0))
        screen.blit(label, (self.x + 10, self.y - 10))

def distance(a, b):
    return math.hypot(a.x - b.x, a.y - b.y)

def neighbors_of(node, connections):
    # undirected graph adjacency over explicit edge list
    for a, b in connections:
        if a == node:
            yield b
        elif b == node:
            yield a

# -------------------------
# Connectivity / BFS helpers
# -------------------------
def graph_reaches_all_demands(source, demands, connections):
    """True only if every demand is reachable from the source in the graph."""
    target = set(demands)
    reached = set()
    seen = {source}
    q = deque([source])

    while q:
        cur = q.popleft()
        if cur in target:
            reached.add(cur)
            if len(reached) == len(target):
                return True
        for nxt in neighbors_of(cur, connections):
            if nxt not in seen:
                seen.add(nxt)
                q.append(nxt)
    return False

def has_physical_path(a, b, connections):
    seen = {a}
    q = deque([a])
    while q:
        cur = q.popleft()
        if cur is b:
            return True
        for nxt in neighbors_of(cur, connections):
            if nxt not in seen:
                seen.add(nxt)
                q.append(nxt)
    return False

def bfs_shortest_path(start, goal, connections):
    """Shortest-hop path (list of nodes, includes start & goal). Empty if none."""
    if start == goal:
        return [start]
    q = deque([start])
    parent = {start: None}
    while q:
        cur = q.popleft()
        for nxt in neighbors_of(cur, connections):
            if nxt not in parent:
                parent[nxt] = cur
                if nxt == goal:
                    # reconstruct
                    path = [goal]
                    while parent[path[-1]] is not None:
                        path.append(parent[path[-1]])
                    path.append(start)
                    path.reverse()
                    return path
                q.append(nxt)
    return []

# -------------------------
# Hop computations
# -------------------------
def bfs_hops_from(start_node, robots, connections):
    """Return dict {robot: hops} from start_node to each reachable robot (None if unreachable)."""
    INF = None
    hops = {r: INF for r in robots}
    seen = {start_node}
    q = deque([(start_node, 0)])

    while q:
        cur, d = q.popleft()
        if isinstance(cur, Robot) and (hops[cur] is None or d < hops[cur]):
            hops[cur] = d
        for nxt in neighbors_of(cur, connections):
            if nxt not in seen:
                seen.add(nxt)
                q.append((nxt, d + 1))
    return hops

def compute_all_hops_and_totals(source, demands, robots, connections):
    """
    For each robot r:
        r.hop_from_source
        r.demand_hops[ demand_name ]  (per-demand hop)
        r.total_overall = hop_from_source + sum(hop_from_each_demand)  (None if any piece is None)
    """
    hop_src = bfs_hops_from(source, robots, connections)

    hop_demands = {}
    for d in demands:
        hop_demands[d.name] = bfs_hops_from(d, robots, connections)

    for r in robots:
        r.hop_from_source = hop_src[r]
        r.demand_hops = {dn: hop_demands[dn][r] for dn in hop_demands}

        if r.hop_from_source is None or any(v is None for v in r.demand_hops.values()):
            r.total_overall = None
        else:
            r.total_overall = r.hop_from_source + sum(r.demand_hops.values())

# -------------------------
# Pivot selection
# -------------------------
def robot_neighbors(rb, connections):
    ns = set()
    for a, b in connections:
        if a is rb and isinstance(b, Robot):
            ns.add(b)
        elif b is rb and isinstance(a, Robot):
            ns.add(a)
    return ns

def robot_degree(rb, connections):
    return len(robot_neighbors(rb, connections))

def choose_global_pivot(robots, connections):
    """
    Global pivot = robot with minimal total_overall (ignore None).
    Tie-break: smaller degree (more central for branching).
    """
    candidates = [r for r in robots if getattr(r, "total_overall", None) is not None]
    if not candidates:
        return None
    return min(candidates, key=lambda r: (r.total_overall, robot_degree(r, connections)))

# -------------------------
# Faster edge building (spatial hash)
# -------------------------
def build_connections_fast(robots, source, demands, r):
    """
    Edges within CONNECTION_DISTANCE r. Uses a simple grid for O(n)-ish build.
    Creates robot-robot, robot-source, robot-demand undirected edges.
    """
    cell = r
    grid = defaultdict(list)

    for i, rb in enumerate(robots):
        cx, cy = int(rb.x // cell), int(rb.y // cell)
        grid[(cx, cy)].append(i)

    connections = []

    # robot-robot edges
    for (cx, cy), idxs in grid.items():
        neigh_cells = [(cx+dx, cy+dy) for dx in (-1,0,1) for dy in (-1,0,1)]
        cand_idxs = []
        for nc in neigh_cells:
            cand_idxs.extend(grid.get(nc, []))
        cand_set = set(cand_idxs)
        for i in idxs:
            ri = robots[i]
            for j in cand_set:
                if j <= i:
                    continue
                rj = robots[j]
                if (ri.x - rj.x)**2 + (ri.y - rj.y)**2 <= r*r:
                    connections.append((ri, rj))

    # robot-source edges
    for rb in robots:
        if (rb.x - source.x)**2 + (rb.y - source.y)**2 <= r*r:
            connections.append((rb, source))

    # robot-demand edges
    for d in demands:
        for rb in robots:
            if (rb.x - d.x)**2 + (rb.y - d.y)**2 <= r*r:
                connections.append((rb, d))

    return connections

# -------------------------
# Pretty hop table (optional)
# -------------------------
def dump_hop_table(robots, demand_names, max_rows=None, path=None):
    lines = []
    header = "Robot | Src | " + " | ".join(demand_names) + " | Total"
    sep = "-" * len(header)
    lines += ["\n--- HOP TABLE (Source & Demands & Total Overall) ---", header, sep]
    for r in robots:
        dhops = [str(r.demand_hops.get(dn)) for dn in demand_names]
        total = str(r.total_overall) if r.total_overall is not None else "∞"
        lines.append(f"{r.robot_id:>5} | {str(r.hop_from_source):>3} | " +
                     " | ".join(f"{x:>3}" for x in dhops) + f" | {total}")

    if max_rows is not None and len(lines) > 3 + max_rows:
        body = lines[3:3+max_rows]
        lines = lines[:3] + body + [f"... ({len(lines) - 3 - max_rows} more rows)"]

    print("\n".join(lines))

    if path:
        try:
            with open(path, "w", encoding="utf-8") as f:
                f.write("\n".join(lines) + "\n")
            print(f"[hop table saved to {path}]")
        except Exception as e:
            print(f"[warn] could not save hop table to {path}: {e}")

# -------------------------
# Main: 1) hops 2) global pivot 3) paths
# -------------------------
def main():
    pygame.init()
    clock = pygame.time.Clock()
    screen = pygame.display.set_mode((ARENA_WIDTH, ARENA_HEIGHT))
    pygame.display.set_caption("Source → Pivot → Demands (global-hops pivot)")

    # --- nodes ---
    source = Node("Source", 50, ARENA_HEIGHT // 2, (255, 0, 0))
    demands = [
        Node("D1", ARENA_WIDTH - 50, 50, (0, 128, 0)),
        Node("D2", ARENA_WIDTH - 50, ARENA_HEIGHT - 50, (0, 128, 0)),
    ]

    # --- robots ---
    robots = []
    for i in range(N_ROBOTS):
        x = random.randint(80, ARENA_WIDTH - 80)
        y = random.randint(60, ARENA_HEIGHT - 60)
        robots.append(Robot(i, x, y, ROBOT_RADIUS))

    # --- build edges; retry a few times for connected layout ---
    connections = build_connections_fast(robots, source, demands, CONNECTION_DISTANCE)
    attempts, MAX_ATTEMPTS = 1, 3
    while not graph_reaches_all_demands(source, demands, connections) and attempts < MAX_ATTEMPTS:
        attempts += 1
        robots = []
        for i in range(N_ROBOTS):
            x = random.randint(80, ARENA_WIDTH - 80)
            y = random.randint(60, ARENA_HEIGHT - 60)
            robots.append(Robot(i, x, y, ROBOT_RADIUS))
        connections = build_connections_fast(robots, source, demands, CONNECTION_DISTANCE)

    # 1) compute hops (source & all demands) and totals
    compute_all_hops_and_totals(source, demands, robots, connections)

    # 2) pick global pivot
    pivot = choose_global_pivot(robots, connections)

    if pivot:
        print(f"Pivot robot: {pivot.robot_id}, total_overall={pivot.total_overall}, "
            f"src={pivot.hop_from_source}, demands={pivot.demand_hops}")
    else:
        print("No global pivot (disconnected or unreachable).")

    # 3) build paths: Source→Pivot and Demand_i→Pivot
    if pivot is not None:
        path_S = bfs_shortest_path(source, pivot, connections)
        demand_paths = {d: bfs_shortest_path(d, pivot, connections) for d in demands}
    else:
        path_S = []
        demand_paths = {d: [] for d in demands}

    # debug table (optional)
    dump_hop_table(robots, [d.name for d in demands], max_rows=120, path="hop_table.txt")

    # ---- draw helpers ----
    def draw_poly(path, width=3):
        for i in range(len(path) - 1):
            pygame.draw.line(screen, (255, 0, 0), (path[i].x, path[i].y), (path[i+1].x, path[i+1].y), width)

    # ---- first frame ----
    screen.fill((255, 255, 255))
    source.draw(screen)
    for d in demands:
        d.draw(screen)

    # draw only the union tree for clarity
    draw_poly(path_S, 3)
    for d, p in demand_paths.items():
        draw_poly(p, 3)

    robots_in_union = {n for n in path_S + sum(demand_paths.values(), []) if isinstance(n, Robot)}
    for rb in robots:
        if rb in robots_in_union:
            rb.draw(screen, color=(0, 200, 0))
        else:
            rb.draw(screen, color=(0, 100, 255))

    pygame.display.flip()
    clock.tick(30)

    # ---- idle loop ----
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        screen.fill((255, 255, 255))
        source.draw(screen)
        for d in demands:
            d.draw(screen)
        draw_poly(path_S, 3)
        for d, p in demand_paths.items():
            draw_poly(p, 3)
        for rb in robots:
            if rb in robots_in_union:
                rb.draw(screen, color=(0, 200, 0))
            else:
                rb.draw(screen, color=(0, 100, 255))
        pygame.display.flip()
        clock.tick(30)

    pygame.quit()

if __name__ == "__main__":
    main()