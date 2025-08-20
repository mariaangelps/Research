import pygame
import random
import math
from collections import deque
from class_robot import Robot
from class_source_and_demand import Source, Demand
# from class_obs import Obstacle  

ARENA_WIDTH, ARENA_HEIGHT = 600,300
ROBOT_RADIUS = 5
N_ROBOTS = 80
N_DEMANDS = 2
CONNECTION_DISTANCE = 60

# obstacles removed entirely
obstacles = []  # keep empty so nothing references it

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

def connect(a, b, connections):
    if (a, b) not in connections and (b, a) not in connections:
        connections.append((a, b))

def propagate_local_hop_count(source, robots, connections, attr_hop, attr_parent):
    for robot in robots:
        setattr(robot, attr_hop, None)
        setattr(robot, attr_parent, None)

    queue = []
    visited = set()

    for a, b in connections:
        if a == source and isinstance(b, Robot):
            queue.append((b, 1, source))
        elif b == source and isinstance(a, Robot):
            queue.append((a, 1, source))

    visited.add(source)

    while queue:
        current, hops, parent = queue.pop(0)

        if isinstance(current, Robot):
            existing_hop = getattr(current, attr_hop)
            if existing_hop is not None and hops >= existing_hop:
                continue
            setattr(current, attr_hop, hops)
            setattr(current, attr_parent, parent if isinstance(parent, Robot) else None)
        visited.add(current)

        neighbors = []
        for a, b in connections:
            if a == current and b not in visited:
                neighbors.append(b)
            elif b == current and a not in visited:
                neighbors.append(a)

        for neighbor in neighbors:
            if isinstance(neighbor, Robot):
                queue.append((neighbor, hops + 1, current))

def is_path_exists(source, demands, robots, connections):
    """True solo si desde Source se alcanza a TODAS las demandas."""
    target = set(demands)
    reached = set()
    seen = {source}
    q = deque([source])

    while q:
        cur = q.popleft()
        if cur in target:
            reached.add(cur)
            if len(reached) == len(target):
                return True  # Ya alcanzamos todas
        # expandir vecinos en grafo no dirigido
        for a, b in connections:
            nxt = None
            if a == cur:
                nxt = b
            elif b == cur:
                nxt = a
            if nxt is not None and nxt not in seen:
                seen.add(nxt)
                q.append(nxt)
    return False


def existe_ruta_fisica(a, b, conexiones_fisicas):
    visitado = set()
    cola = deque([a])
    visitado.add(a)
    while cola:
        actual = cola.popleft()
        if actual == b:
            return True
        vecinos = [r2 for (r1, r2) in conexiones_fisicas if r1 == actual] + \
                  [r1 for (r1, r2) in conexiones_fisicas if r2 == actual]
        for vecino in vecinos:
            if vecino not in visitado:
                visitado.add(vecino)
                cola.append(vecino)
    return False

def get_node_name(n):
    return n.name if isinstance(n, Node) else f"Robot {n.robot_id}"
def neighbors_of(node, connections):
    for a, b in connections:
        if a == node:
            yield b
        elif b == node:
            yield a

def bfs_shortest_path(start, goal, connections):
    """Camino más corto (en hops) en el grafo no dirigido; incluye start y goal."""
    if start == goal:
        return [start]
    from collections import deque
    q = deque([start])
    parent = {start: None}
    while q:
        cur = q.popleft()
        for nxt in neighbors_of(cur, connections):
            if nxt not in parent:
                parent[nxt] = cur
                if nxt == goal:
                    # reconstruir
                    path = [goal]
                    while path[-1] is not None:
                        prev = parent[path[-1]]
                        if prev is None:
                            break
                        path.append(prev)
                    path.reverse()
                    return path
                q.append(nxt)
    return []  # no hay camino


# ❌ obstacle-aware helpers removed
def is_best_path_valid(path, connections):
    for i in range(len(path) - 1):
        a = path[i]
        b = path[i + 1]
        if a == b:
            continue
        if (a, b) not in connections and (b, a) not in connections:
            return False
""""
def choose_best_demand(source, demands, robots, connections):
    best = {"demand": None, "path_src": [], "path_dst": [], "score": float("inf"), "tie_len": float("inf")}
    propagate_local_hop_count(source, robots, connections, 'hop_from_source', 'parent_from_source')

    for r in robots:
        r.demand_hops = {}

    for d in demands:
        propagate_local_hop_count(d, robots, connections, 'hop_from_demand', 'parent_from_demand')
        for r in robots:
            r.demand_hops[d.name] = r.hop_from_demand

        path_src = build_optimal_path(source, d, robots, connections, 'hop_from_source')
        path_dst = build_optimal_path(d, source, robots, connections, 'hop_from_demand')

        elig = [r.hop_from_source + r.hop_from_demand for r in robots
                if r.hop_from_source is not None and r.hop_from_demand is not None]
        min_total = min(elig) if elig else float("inf")
        tie_len = len(path_src)

        if (min_total < best["score"]) or (min_total == best["score"] and tie_len < best["tie_len"]):
            best.update({"demand": d, "path_src": path_src, "path_dst": path_dst,
                         "score": min_total, "tie_len": tie_len})
    return best["demand"], best["path_src"], best["path_dst"]
"""
def build_optimal_path(start, end, robots, connections, hop_attr):
    path = []
    visited = set()
    current = start
    current_hop = 0 if not isinstance(start, Robot) else getattr(start, hop_attr)

    while current != end:
        visited.add(current)
        if isinstance(end, Node) and distance(current, end) <= CONNECTION_DISTANCE:
            if not path or path[-1] != current:
                path.append(current)
            path.append(end)
            break

        candidates = []
        for r in robots:
            if r in visited:
                continue
            if getattr(r, hop_attr) == current_hop + 1:
                if existe_ruta_fisica(current, r, connections):
                    sh = getattr(r, 'hop_from_source') or float('inf')
                    dh = getattr(r, 'hop_from_demand') or float('inf')
                    candidates.append((sh + dh, r))

        if not candidates:
            break

        def connection_score(robot, connections):
            vecinos = [r2 for (r1, r2) in connections if r1 == robot and isinstance(r2, Robot)] + \
                      [r1 for (r1, r2) in connections if r2 == robot and isinstance(r1, Robot)]
            return -len(vecinos)

        _, best_next = min(candidates, key=lambda x: (x[0], connection_score(x[1], connections)))
        path.append(best_next)
        visited.add(best_next)
        current = best_next
        current_hop += 1

    if isinstance(start, Node):
        first = path[0] if path else None
        if first and first != start and distance(start, first) <= CONNECTION_DISTANCE:
            path.insert(0, start)
    return path

def build_path_after_repulsion(start, end, robots, connections, hop_attr):
    # identical to build_optimal_path but restricted to neighbors of current
    path = []
    visited = set()
    current = start
    current_hop = 0 if not isinstance(start, Robot) else getattr(start, hop_attr)

    while current != end:
        visited.add(current)
        if isinstance(end, Node) and distance(current, end) <= CONNECTION_DISTANCE:
            if not path or path[-1] != current:
                path.append(current)
            path.append(end)
            break

        candidates = []
        neighbors = [r2 for (r1, r2) in connections if r1 == current and isinstance(r2, Robot)] + \
                    [r1 for (r1, r2) in connections if r2 == current and isinstance(r1, Robot)]

        for r in neighbors:
            if r in visited:
                continue
            if getattr(r, hop_attr) == current_hop + 1:
                sh = getattr(r, 'hop_from_source') or float('inf')
                dh = getattr(r, 'hop_from_demand') or float('inf')
                candidates.append((sh + dh, r))

        if not candidates:
            break

        def connection_score(robot, connections):
            vecinos = [r2 for (r1, r2) in connections if r1 == robot and isinstance(r2, Robot)] + \
                      [r1 for (r1, r2) in connections if r2 == robot and isinstance(r1, Robot)]
            return -len(vecinos)

        candidates.sort(key=lambda x: (x[0], connection_score(x[1], connections)))
        best_next = candidates[0][1]
        path.append(best_next)
        visited.add(best_next)
        current = best_next
        current_hop += 1

    if isinstance(start, Node):
        first = path[0] if path else None
        if first and distance(start, first) <= CONNECTION_DISTANCE:
            path.insert(0, start)
    return path

def debug_global_choice(source, demands, robots, connections, best_demand):
    propagate_local_hop_count(source, robots, connections, 'hop_from_source', 'parent_from_source')
    rows = []
    for d in demands:
        propagate_local_hop_count(d, robots, connections, 'hop_from_demand', 'parent_from_demand')
        path_src = build_optimal_path(source, d, robots, connections, 'hop_from_source')
        tie_len = len(path_src)

        eligibles = [(r.robot_id, r.hop_from_source, r.hop_from_demand,
                      r.hop_from_source + r.hop_from_demand)
                     for r in robots
                     if r.hop_from_source is not None and r.hop_from_demand is not None]
        if eligibles:
            best_robot = min(eligibles, key=lambda t: t[3])
            min_total = best_robot[3]
        else:
            min_total = float('inf')
        rows.append((d.name, min_total, tie_len, None))

    
        
def bfs_hops_from(start_node, robots, connections):
    """Devuelve dict {robot: hops} desde start_node a cada robot alcanzable."""
    INF = None
    hops = {r: INF for r in robots}
    visited = set([start_node])
    q = deque([(start_node, 0)])

    while q:
        cur, d = q.popleft()
        # Si llegamos a un robot, registra hop si es el primero o mejora
        if isinstance(cur, Robot):
            if hops[cur] is None or d < hops[cur]:
                hops[cur] = d
        # Expandir vecinos
        for a, b in connections:
            nxt = None
            if a == cur and b not in visited:
                nxt = b
            elif b == cur and a not in visited:
                nxt = a
            if nxt is not None:
                visited.add(nxt)
                q.append((nxt, d + 1))
    return hops

def compute_all_hops_and_totals(source, demands, robots, connections):
    # Hops desde la fuente
    hop_src = bfs_hops_from(source, robots, connections)

    # Hops desde cada demanda
    hop_demands = {}  # name -> {robot: hops}
    for d in demands:
        hop_demands[d.name] = bfs_hops_from(d, robots, connections)

    # Adjuntar a cada robot y calcular el "overall"
    for r in robots:
        r.hop_from_source = hop_src[r]
        r.demand_hops = {dn: hop_demands[dn][r] for dn in hop_demands}

        # TOTAL OVERALL = Source→r + sum(Di→r)  (esto es lo que tu profe llama “total number of hop counts”)
        # Si alguno es None (inaccesible), el total se considera infinito (None).
        parts = []
        if r.hop_from_source is not None:
            parts.append(r.hop_from_source)
        else:
            r.total_overall = None
            continue
        for dn, hv in r.demand_hops.items():
            if hv is None:
                r.total_overall = None
                break
            parts.append(hv)
        else:
            r.total_overall = sum(parts)

def print_hop_table(robots, demand_names):
    print("\n--- HOP TABLE (Source & Demands & Total Overall) ---")
    header = "Robot | Src | " + " | ".join([f"{dn}" for dn in demand_names]) + " | Total"
    print(header)
    print("-" * len(header))
    for r in robots:
        dhops = [str(r.demand_hops.get(dn)) for dn in demand_names]
        total = str(r.total_overall) if r.total_overall is not None else "∞"
        print(f"{r.robot_id:>5} | {str(r.hop_from_source):>3} | " +
              " | ".join([f"{x:>3}" for x in dhops]) + f" | {total}")
def choose_pivot_robot(robots):
    # Elige el robot con menor r.total_overall (ignorando None). Desempate: menor grado (más “centralidad” util).
    candidates = [r for r in robots if r.total_overall is not None]
    if not candidates:
        return None
    # Puedes afinar el desempate con “grado” si lo calculas; por ahora, sólo total.
    return min(candidates, key=lambda r: r.total_overall)
from collections import defaultdict

def build_connections_fast(robots, source, demands, r):
    """Build edges using a spatial hash grid: O(n)–ish instead of O(n^2)."""
    cell = r  # cell size = connection distance
    grid = defaultdict(list)

    # put robots in grid
    for i, rb in enumerate(robots):
        cx, cy = int(rb.x // cell), int(rb.y // cell)
        grid[(cx, cy)].append(i)

    connections = []

    # robot-robot edges (only check 3x3 neighbor cells)
    n = len(robots)
    for (cx, cy), idxs in grid.items():
        neigh_cells = [(cx+dx, cy+dy) for dx in (-1, 0, 1) for dy in (-1, 0, 1)]
        cand_idxs = []
        for nc in neigh_cells:
            cand_idxs.extend(grid.get(nc, []))
        cand_set = set(cand_idxs)

        for i in idxs:
            ri = robots[i]
            # only j > i to avoid dup checking
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

def dump_hop_table_once(robots, demand_names, max_rows=None, save_path=None):
    """
    Prints the hop table once (optionally only first N rows) and/or saves full table to a file.
    """
    lines = []
    header = "Robot | Src | " + " | ".join([f"{dn}" for dn in demand_names]) + " | Total"
    sep = "-" * len(header)
    lines.append("\n--- HOP TABLE (Source & Demands & Total Overall) ---")
    lines.append(header)
    lines.append(sep)
    for r in robots:
        dhops = [str(r.demand_hops.get(dn)) for dn in demand_names]
        total = str(r.total_overall) if r.total_overall is not None else "∞"
        lines.append(f"{r.robot_id:>5} | {str(r.hop_from_source):>3} | " +
                     " | ".join([f"{x:>3}" for x in dhops]) + f" | {total}")

    # Print (maybe truncated)
    to_print = lines
    if max_rows is not None:
        # keep header + first max_rows robot lines
        head = lines[:3]
        body = lines[3:3+max_rows]
        tail = []
        if len(lines) > 3 + max_rows:
            tail = [f"... ({len(lines) - 3 - max_rows} more rows not shown)"]
        to_print = head + body + tail

    print("\n".join(to_print))

    # Save full table if requested
    if save_path:
        try:
            with open(save_path, "w", encoding="utf-8") as f:
                f.write("\n".join(lines) + "\n")
            print(f"\n[hop table saved to {save_path}]")
        except Exception as e:
            print(f"[warn] could not save hop table to {save_path}: {e}")
def robot_neighbors(rb, connections):
    """Return the set of robot neighbors directly connected to rb."""
    neigh = set()
    for a, b in connections:
        if a is rb and isinstance(b, Robot):
            neigh.add(b)
        elif b is rb and isinstance(a, Robot):
            neigh.add(a)
    return neigh

def robot_metric(rb, metric="total"):
    """
    Metric selector:
        - "total": r.total_overall (sum of source + all demands hops)  [DEFAULT]
        - "source": r.hop_from_source
        - "max_demand": max over demands, useful for balancing
    Returns None if undefined.
    """
    if metric == "total":
        return getattr(rb, "total_overall", None)
    elif metric == "source":
        return getattr(rb, "hop_from_source", None)
    elif metric == "max_demand":
        if not hasattr(rb, "demand_hops") or not rb.demand_hops:
            return None
        vals = [v for v in rb.demand_hops.values() if v is not None]
        return max(vals) if vals else None
    else:
        return getattr(rb, "total_overall", None)  # fallback

def is_local_minimum(rb, connections, metric="total"):
    """
    True if rb's metric is strictly less than all neighbor robots' metrics.
    If rb.metric is None, returns False.
    If neighbor has None metric, that neighbor is ignored.
    """
    mv = robot_metric(rb, metric)
    if mv is None:
        return False
    for nb in robot_neighbors(rb, connections):
        nv = robot_metric(nb, metric)
        if nv is None:
            continue
        if mv > nv:  # must be strictly smaller than every neighbor
            return False
    return True

def find_local_minima(robots, connections, metric="total"):
    """Return a list of robots that are local minima under the chosen metric."""
    return [r for r in robots if is_local_minimum(r, connections, metric=metric)]

def main():
    pygame.init()
    clock = pygame.time.Clock()
    screen = pygame.display.set_mode((ARENA_WIDTH, ARENA_HEIGHT))
    pygame.display.set_caption("Fast joint network (Source ↔ Pivot ↔ Demands)")

    # ---- Nodes ----
    source = Node("Source", 50, ARENA_HEIGHT // 2, (255, 0, 0))
    demands = [Node("D1", ARENA_WIDTH - 50, 50, (0, 128, 0)),
               Node("D2", ARENA_WIDTH - 50, ARENA_HEIGHT - 50, (0, 128, 0))]

    # ---- Robots (random once) ----
    robots = []
    for i in range(N_ROBOTS):
        x = random.randint(80, ARENA_WIDTH - 80)
        y = random.randint(60, ARENA_HEIGHT - 60)
        robots.append(Robot(i, x, y, ROBOT_RADIUS))

    # ---- Build connections ONCE (fast) ----
    connections = build_connections_fast(robots, source, demands, CONNECTION_DISTANCE)

    # If graph doesn’t connect everything, do a couple of reseeds (cap attempts)
    attempts = 1
    MAX_ATTEMPTS = 3
    while not is_path_exists(source, demands, robots, connections) and attempts < MAX_ATTEMPTS:
        attempts += 1
        robots = []
        for i in range(N_ROBOTS):
            x = random.randint(80, ARENA_WIDTH - 80)
            y = random.randint(60, ARENA_HEIGHT - 60)
            robots.append(Robot(i, x, y, ROBOT_RADIUS))
        connections = build_connections_fast(robots, source, demands, CONNECTION_DISTANCE)

    # ---- Heavy math ONCE ----
    compute_all_hops_and_totals(source, demands, robots, connections)
    print("Local minima (by total):", [r.robot_id for r in find_local_minima(robots, connections, "total")])

    # Choose by local criterion first; fallback to global if none.
    local_mins = find_local_minima(robots, connections, metric="total")  # or "source"
    if local_mins:
        # Among locals, pick the best (e.g., smallest total_overall)
        pivot = min(local_mins, key=lambda r: robot_metric(r, "total"))
    else:
        # Fallback: global minimum (what you had before)
        pivot = choose_pivot_robot(robots)

    # Print first 30 rows to console, save full table to a file
    demand_names = [d.name for d in demands]
    dump_hop_table_once(robots, demand_names, max_rows=121, save_path="hop_table.txt")


    if pivot:
        path_S = bfs_shortest_path(source, pivot, connections)
        demand_paths = {d: bfs_shortest_path(d, pivot, connections) for d in demands}
    else:
        path_S = []
        demand_paths = {d: [] for d in demands}

    # ---- Draw helpers ----
    def draw_poly(path, color=(255, 0, 0), width=3):
        for i in range(len(path) - 1):
            pygame.draw.line(screen, color, (path[i].x, path[i].y), (path[i+1].x, path[i+1].y), width)

    # ---- First frame ----
    screen.fill((255, 255, 255))
    source.draw(screen)
    for d in demands:
        d.draw(screen)

    # Optional: comment the next block if too many edges to draw fast
    # for a,b in connections:
    #     pygame.draw.line(screen, (220,220,220), (a.x,a.y), (b.x,b.y), 1)

    # Draw only the joint tree (much faster)
    draw_poly(path_S, (255, 0, 0), 3)
    for d, p in demand_paths.items():
        draw_poly(p, (255, 0, 0), 3)

    robots_in_union = set(n for n in path_S + sum(demand_paths.values(), []) if isinstance(n, Robot))
    for rb in robots:
        if rb in robots_in_union:
            rb.draw(screen, color=(0, 200, 0))
        else:
            rb.draw(screen, color=(0, 100, 255))

    pygame.display.flip()
    clock.tick(30)

    # ---- Idle loop: JUST draw (no recompute) ----
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # If nothing moves, we can skip redrawing entirely,
        # but we’ll keep a light redraw for window events:
        screen.fill((255, 255, 255))
        source.draw(screen)
        for d in demands:
            d.draw(screen)

        # Optional (comment if slow): draw the full graph
        # for a,b in connections:
        #     pygame.draw.line(screen, (220,220,220), (a.x,a.y), (b.x,b.y), 1)

        draw_poly(path_S, (255, 0, 0), 3)
        for d, p in demand_paths.items():
            draw_poly(p, (255, 0, 0), 3)

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