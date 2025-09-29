import pygame
import random
import math
from collections import deque
from class_robot import Robot
from class_source_and_demand import Source, Demand
import time
# from class_obs import Obstacle  

ARENA_WIDTH, ARENA_HEIGHT = 800,300
ROBOT_RADIUS = 6 
N_ROBOTS = 120
N_DEMANDS = 25
CONNECTION_DISTANCE = 120
SENSE_RADIUS_R = 200        # big sensing radius (R)
CONNECT_RADIUS_r = CONNECTION_DISTANCE  # connection radius (r)
STEP_MAX = 1.6              # max movement per frame 
K_ATTR_ONPATH = 0.60        # attraction gain if robot is on the golden network
K_ATTR_OFFPATH = 1.00       # weaker attraction if robot is outside
RECOMPUTE_EVERY = 10        # recompute pivot + paths every N frames
SOURCE_NODE = None  # se setea en main() para uso dentro de apply_sink_attraction5
K_LADDER = 0.8
# === Debug reach logs ===
DEMAND_FIRST_REACH = {}   # nombre_demand -> (robot_id, frame) first robot arrived
DEMAND_REACHERS   = {}    # nombre_demand -> set(robot_id) of all reached

ALIGN_DOT_THRESHOLD = 0.2      # >0 ⇒ direcciones ~alineadas ⇒ find_branch
NETWORK_LEASH = int(CONNECTION_DISTANCE * 1.15)  # no despegarse de la red
BRANCH_K = 0.9                  # empuje al seguir la red buscando rama
DIRECT_K = 1.0                  # empuje al ir directo a la demand


# obstacles removed entirely
obstacles = []  # keep empty so nothing references it

class Node:
    def __init__(self, name, x, y, color, radius=9):
        self.name = name
        self.x = x
        self.y = y
        self.color = color
        self.radius = radius

    def draw(self, screen):
        pygame.draw.circle(screen, self.color, (int(self.x), int(self.y)), self.radius)
        font = pygame.font.Font(None, 24)
        label = font.render(self.name, True, (0, 0, 0))
        screen.blit(label, (self.x + 10, self.y - 10))

def check_and_color_robots(robots, demands, frame=None):
    """Marca at_demand=True y loguea SOLO cuando realmente toca la demand."""
    global DEMAND_FIRST_REACH, DEMAND_REACHERS

    for r in robots:
        if getattr(r, "at_demand", False):
            continue
        for d in demands:
            d_rad = getattr(d, "radius", 12)
            if math.hypot(r.x - d.x, r.y - d.y) <= d_rad + ROBOT_RADIUS:
                # Se vuelve morado aquí
                r.at_demand = True

                # Logging consistente con el morado
                if frame is not None:
                    nm = d.name
                    DEMAND_REACHERS.setdefault(nm, set())
                    DEMAND_FIRST_REACH.setdefault(nm, None)

                    if r.robot_id not in DEMAND_REACHERS[nm]:
                        DEMAND_REACHERS[nm].add(r.robot_id)
                        if DEMAND_FIRST_REACH[nm] is None:
                            DEMAND_FIRST_REACH[nm] = (r.robot_id, frame)
                            print(f"[REACHED-FIRST] {nm} reached by Robot {r.robot_id} at frame {frame}")
                        else:
                            print(f"[REACHED] {nm} also reached by Robot {r.robot_id} at frame {frame}")

                        # (opcional) resumen cuando todas tengan primero
                        if all(DEMAND_FIRST_REACH[k] is not None for k in DEMAND_FIRST_REACH):
                            summary = {k: f"robot {v[0]} @frame {v[1]}" for k, v in DEMAND_FIRST_REACH.items()}
                            print("[SUMMARY] First reach per demand:", summary)
                break  # no sigas chequeando más demands para este robot


def distance(a, b):
    return math.hypot(a.x - b.x, a.y - b.y)

def connect(a, b, connections):
    if (a, b) not in connections and (b, a) not in connections:
        connections.append((a, b))

def propagate_local_hop_count(start_node, robots, connections, attr_hop, attr_parent, debug=False):

    """
    Writes on each robot the minimum hop count from start_node.
    Rule: a robot neighbor of a robot with hop=h will get hop=h+1.
    Non-robot Node objects (Source/Demands) just relay without increasing hop.
    BFS is used over the undirected graph represented by 'connections'.
    """

    from collections import deque

    # Reset hop/parent attributes on robots
    for r in robots:
        setattr(r, attr_hop, None)
        setattr(r, attr_parent, None)

   # Standard BFS
    q = deque()
    seen = set([start_node])

     # Seed: neighbors of the start node
    for nb in neighbors_of(start_node, connections):
        if isinstance(nb, Robot):
             # Robots directly reachable from the start get hop = 1
            setattr(nb, attr_hop, 1)
            setattr(nb, attr_parent, start_node if isinstance(start_node, Robot) else None)
            q.append(nb)
            seen.add(nb)
        else:
            # Non-robot intermediate nodes are enqueued to continue BFS
            q.append(nb)
            seen.add(nb)

    while q:
        cur = q.popleft()

        # Current hop value (only robots have hop values)

        cur_hop = getattr(cur, attr_hop, None) if isinstance(cur, Robot) else None

        for nxt in neighbors_of(cur, connections):
            if nxt not in seen:
                seen.add(nxt)
                q.append(nxt)

            # Relaxation for robot-to-robot edges: next hop should be c`ur_hop + 1
            if isinstance(cur, Robot) and isinstance(nxt, Robot):
                candidate = (cur_hop if cur_hop is not None else 0) + 1
                prev = getattr(nxt, attr_hop)
                if prev is None or candidate < prev:
                    setattr(nxt, attr_hop, candidate)
                    setattr(nxt, attr_parent, cur)

def unconnected_demands(demands, demand_paths):
    """Lista de demands que aún no tienen camino dibujado."""
    out = []
    for d in demands:
        p = demand_paths.get(d, [])
        if not p:
            out.append(d)
    return out

def nearest_on_network(rb, robots_in_union):
    """Nodo de la red (Robot o Node) más cercano al robot."""
    best, bd = None, float('inf')
    for n in robots_in_union:
        d = math.hypot(rb.x - n.x, rb.y - n.y)
        if d < bd:
            best, bd = n, d
    return best, bd

def decide_branch_or_direct(rb, robots_in_union, demands, demand_paths):
    """
    Devuelve:
      ("find_branch", net_target, dem_target)  ó
      ("connect_direct", None, dem_target)     ó
      None si no hay demands sin conectar.
    Criterio: dot( dir→network , dir→demand ) > ALIGN_DOT_THRESHOLD ⇒ find_branch.
    """
    u = unconnected_demands(demands, demand_paths)
    if not u:
        return None

    # demand no-conectada más cercana
    dem, _ = nearest_demand_and_dist(rb, u)
    # nodo de red más cercano
    net, _ = nearest_on_network(rb, robots_in_union)

    if dem is None or net is None:
        return ("connect_direct", None, dem)

    # unit vectors
    v_net = (net.x - rb.x, net.y - rb.y)
    v_dem = (dem.x - rb.x, dem.y - rb.y)
    mag_n = math.hypot(*v_net); mag_d = math.hypot(*v_dem)
    if mag_n < 1e-9 or mag_d < 1e-9:
        return ("connect_direct", None, dem)

    u_net = (v_net[0]/mag_n, v_net[1]/mag_n)
    u_dem = (v_dem[0]/mag_d, v_dem[1]/mag_d)
    dot = u_net[0]*u_dem[0] + u_net[1]*u_dem[1]

    if dot > ALIGN_DOT_THRESHOLD:
        return ("find_branch", net, dem)
    else:
        return ("connect_direct", None, dem)

def leash_to_network(rb, robots_in_union, max_dist=NETWORK_LEASH):
    """Vector correctivo hacia la red si rb se aleja más de max_dist."""
    net, d = nearest_on_network(rb, robots_in_union)
    if net is None or d <= max_dist:
        return (0.0, 0.0)
    dx, dy = (net.x - rb.x), (net.y - rb.y)
    if d < 1e-9: 
        return (0.0, 0.0)
    # fuerza suave proporcional al exceso
    excess = d - max_dist
    fx = (dx / d) * min(1.0, excess / max_dist)
    fy = (dy / d) * min(1.0, excess / max_dist)
    return (fx, fy)

def spawn_demands_random(n, arena_w, arena_h, source, min_sep=45, margin=40, max_tries=5000):
    """
    Genera n demands en posiciones aleatorias del mapa, separadas al menos 'min_sep'
    entre sí y con un margen de 'margin' a los bordes. Evita spawnear muy pegado al Source.
    """
    pts = []
    tries = 0
    while len(pts) < n and tries < max_tries:
        tries += 1
        x = random.randint(margin, arena_w - margin)
        y = random.randint(margin, arena_h - margin)

        # evita quedar demasiado cerca del source (para que no colisione visualmente)
        if math.hypot(x - source.x, y - source.y) < min_sep * 1.5:
            continue

        # respeta separación mínima entre demands
        ok = True
        for (px, py) in pts:
            if math.hypot(x - px, y - py) < min_sep:
                ok = False
                break
        if ok:
            pts.append((x, y))

    # Si no alcanzó a separar todas (mapa muy lleno), rellena lo que falte sin separación estricta
    while len(pts) < n:
        x = random.randint(margin, arena_w - margin)
        y = random.randint(margin, arena_h - margin)
        pts.append((x, y))

    return pts

def is_path_exists(source, demands, robots, connections):
    """Return True if ALL demand nodes are reachable from Source."""
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
        # expand neighbors
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

#function to prove there is a physical route between two nodes (a and b)
def existe_ruta_fisica(a, b, conexiones_fisicas):
    visitado = set()
    #queue
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

# Generator over neighbors of 'node' in the undirected 'connections' list
#Each (a, b) pair represents a bidirectional link, so both a→b and b→a are valid.

def neighbors_of(node, connections):
    for a, b in connections:
        if a == node:
            yield b
        elif b == node:
            yield a

def bfs_shortest_path(start, goal, connections):

    #Unweighted shortest path (by hops) in the undirected graph; includes start and goal.

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
                    # reconstruct golden path by backtracking parents
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


# obstacle-aware helpers removed
def is_best_path_valid(path, connections):
    for i in range(len(path) - 1):
        #ensure each consecutive pair in 'path' has an edge in 'connections'
        a = path[i]
        b = path[i + 1]
        if a == b:
            continue
        if (a, b) not in connections and (b, a) not in connections:
            return False


def build_optimal_path(start, end, robots, connections, hop_attr):
    """
    Greedy path builder that advances through robots with hop = current_hop + 1,
    preferring robots that minimize (hop_from_source + hop_from_demand) and with
    a mild tie-break on fewer robot neighbors (to reduce branching).
    """
    path = []
    visited = set()
    current = start
    current_hop = 0 if not isinstance(start, Robot) else getattr(start, hop_attr)

    while current != end:
        visited.add(current)
        # If the end is a Node and it's physically within connection distance, close the path
        if isinstance(end, Node) and distance(current, end) <= CONNECTION_DISTANCE:
            if not path or path[-1] != current:
                path.append(current)
            path.append(end)
            break

        candidates = []
        for r in robots:
            if r in visited:
                continue
            # Only consider robots in the next hop layer (curr+1)
            if getattr(r, hop_attr) == current_hop + 1:
                if existe_ruta_fisica(current, r, connections):
                    # Combine source/demand hop info for scoring
                    sh = getattr(r, 'hop_from_source', None)
                    sh = sh if sh is not None else float('inf')

                    dh = getattr(r, 'hop_from_demand', None)
                    dh = dh if dh is not None else float('inf')

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

    # If start is a Node and the first hop is within range, make connection
    if isinstance(start, Node):
        first = path[0] if path else None
        if first and first != start and distance(start, first) <= CONNECTION_DISTANCE:
            path.insert(0, start)
    return path
"""
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
    return pat

# Helper to inspect min total hops per demand; prints tie lengths and best robot scores
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

 """  
 
        
def bfs_hops_from(start_node, robots, connections):
    """
    Return a dict {robot: hops} representing the hop distance from 'start_node'
    to each reachable robot using BFS over undirected edges.
    """
    INF = None
    hops = {r: INF for r in robots}
    visited = set([start_node])
    q = deque([(start_node, 0)])

    while q:
        cur, d = q.popleft()
         #Record hop for robots the first time they are seen or if we find a better d
        if isinstance(cur, Robot):
            if hops[cur] is None or d < hops[cur]:
                hops[cur] = d
        # Expand neighbors
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
    """
    Compute per-robot:
      - hop_from_source
      - demand_hops (per demand)
      - total_overall = hop_from_source + sum(hops from each demand)
    Robots unreachable from either source or any demand get total_overall = None (∞).
    """
     
    # Hops desde la fuente
    hop_src = bfs_hops_from(source, robots, connections)

    # Hops desde cada demanda
    hop_demands = {}  # name -> {robot: hops}
    for d in demands:
        hop_demands[d.name] = bfs_hops_from(d, robots, connections)

    # Attach values on each robot and compute totals
    for r in robots:
        r.hop_from_source = hop_src[r]
        r.demand_hops = {dn: hop_demands[dn][r] for dn in hop_demands}

        # TOTAL OVERALL = Source→r + sum(Di→r)  “total number of hop counts”
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
    """
    Fallback global pivot selection (when no local minima are found).
    Uses the same tie-breaking logic as in main():
      - lowest total_overall
      - if tie: closest to source
      - if still tie: lowest robot_id
    """
    candidates = [r for r in robots if getattr(r, "total_overall", None) is not None]
    if not candidates:
        return None
    return min(candidates, key=pivot_key)

from collections import defaultdict


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
    #Return a list of robots that are local minima under the chosen metric.
    return [r for r in robots if is_local_minimum(r, connections, metric=metric)]
def pivot_key(r):
    """
    Key function for pivot selection.
    Priority order:
      1) Lowest total_overall (sum of source + all demands hop counts)
      2) Closest to the source (lowest hop_from_source)
      3) Lowest robot_id as a final deterministic tie-breaker

    This ensures that when there is a tie in total_overall,
    the pivot chosen will be the one physically closer to the source.
    """
    total = robot_metric(r, "total")
    src = getattr(r, "hop_from_source", None)
    if total is None:
        total = float("inf")
    if src is None:
        src = 10**9 # Large number so they lose tie-break if unreachable
    return (total, src, r.robot_id)

def nearest_demand_and_dist(p, demands):
    best = (None, float('inf'))
    for d in demands:
        dist = math.hypot(p.x - d.x, p.y - d.y)
        if dist < best[1]:
            best = (d, dist)
    return best  # (demand, distance)

def clamp_step(dx, dy, max_step):
    mag = math.hypot(dx, dy)
    if mag == 0:
        return 0.0, 0.0
    if mag <= max_step:
        return dx, dy
    s = max_step / mag
    return dx * s, dy * s

def rebuild_connections(robots, source, demands):
    connections = []
    # robot–robot
    for i in range(len(robots)):
        for j in range(i + 1, len(robots)):
            if distance(robots[i], robots[j]) <= CONNECTION_DISTANCE:
                connect(robots[i], robots[j], connections)
    # robot–source / robot–demands
    for rb in robots:
        if distance(rb, source) <= CONNECTION_DISTANCE:
            connect(rb, source, connections)
        for d in demands:
            if distance(rb, d) <= CONNECTION_DISTANCE:
                connect(rb, d, connections)
    return connections

def apply_sink_attraction(robots, demands, robots_in_union, connections, current_frame=None):
    """
    Mueve SOLO a robots que aún no llegaron a una demanda (no morados).
    - ON-PATH: demanda correcta por hops + escalera + branch/direct + rienda.
    - OFF-PATH: atraídos al nodo de red más cercano (fallback: Source).
    Usa donut (r,R) y clamp STEP_MAX.
    """
    global SOURCE_NODE, demand_paths
    if SOURCE_NODE is None:
        return

    for rb in robots:
        # --- Si ya tocó una demanda (morado), no se mueve ---
        if getattr(rb, "at_demand", False):
            continue

        # === 1) Objetivo principal ===
        if rb in robots_in_union:
            target_d, _ = best_demand_for_robot(rb, demands)
            if target_d is None:
                target_d, _ = nearest_demand_and_dist(rb, demands)
            target = target_d
            k_main = K_ATTR_ONPATH
        else:
            nt, _ = nearest_on_network(rb, robots_in_union)
            target = nt if nt is not None else SOURCE_NODE
            k_main = K_ATTR_OFFPATH

        # === 2) Fuerza principal con donut ===
        dist = math.hypot(target.x - rb.x, target.y - rb.y)
        if CONNECT_RADIUS_r < dist < SENSE_RADIUS_R:
            vx, vy = (target.x - rb.x), (target.y - rb.y)
            fx = k_main * vx / (dist + 1e-6)
            fy = k_main * vy / (dist + 1e-6)
        else:
            fx = fy = 0.0

        # NUDGE solo OFF-PATH si ya está dentro de r (evita quedarse congelado)
        if rb not in robots_in_union and dist <= CONNECT_RADIUS_r:
            vx, vy = (target.x - rb.x), (target.y - rb.y)
            fx += 0.18 * vx / (dist + 1e-6)
            fy += 0.18 * vy / (dist + 1e-6)

        # === 3) ON-PATH: escalera + decisión branch/direct + rienda ===
        if rb in robots_in_union:
            d_best, _ = best_demand_for_robot(rb, demands)
            dname = d_best.name if d_best else None
            nxt = neighbor_with_lower_hop_to_demand(rb, dname, connections)

            if nxt is not None:
                dx, dy = (nxt.x - rb.x), (nxt.y - rb.y)
                mag = math.hypot(dx, dy)
                if mag > 1e-9:
                    fx += K_LADDER * dx / mag
                    fy += K_LADDER * dy / mag
            else:
                if d_best is not None:
                    vx2, vy2 = (d_best.x - rb.x), (d_best.y - rb.y)
                    dist2 = math.hypot(vx2, vy2)
                    if dist2 > 1e-6:
                        fx += K_LADDER * vx2 / dist2
                        fy += K_LADDER * vy2 / dist2

            try:
                decision = decide_branch_or_direct(rb, robots_in_union, demands, demand_paths)
            except NameError:
                decision = None

            if decision is not None:
                mode, net_target, dem_target = decision

                if mode == "find_branch" and net_target is not None and dem_target is not None:
                    dxn, dyn = net_target.x - rb.x, net_target.y - rb.y
                    dn = math.hypot(dxn, dyn)
                    if dn > 1e-9:
                        fx += BRANCH_K * dxn / dn
                        fy += BRANCH_K * dyn / dn
                    dxd, dyd = dem_target.x - rb.x, dem_target.y - rb.y
                    dd = math.hypot(dxd, dyd)
                    if dd > 1e-9:
                        fx += 0.35 * dxd / dd
                        fy += 0.35 * dyd / dd

                elif mode == "connect_direct" and dem_target is not None:
                    dxd, dyd = dem_target.x - rb.x, dem_target.y - rb.y
                    dd = math.hypot(dxd, dyd)
                    if dd > 1e-9:
                        fx += DIRECT_K * dxd / dd
                        fy += DIRECT_K * dyd / dd

                lx, ly = leash_to_network(rb, robots_in_union, NETWORK_LEASH)
                fx += lx
                fy += ly

        # === 4) Clamp y mover ===
        dx, dy = clamp_step(fx, fy, STEP_MAX)
        rb.x = max(0, min(ARENA_WIDTH,  rb.x + dx))
        rb.y = max(0, min(ARENA_HEIGHT, rb.y + dy))


def best_demand_for_robot(rb, demands):
    """
    Devuelve (demand, hops) donde 'demand' minimiza los hops desde esa demanda al robot.
    Si el robot no tiene hops válidos a ninguna demanda, retorna (None, None).
    """
    if not hasattr(rb, "demand_hops") or not rb.demand_hops:
        return (None, None)
    best_nm, best_h = None, None
    for d in demands:
        hv = rb.demand_hops.get(d.name, None)
        if hv is None:
            continue
        if best_h is None or hv < best_h:
            best_nm, best_h = d.name, hv
    if best_nm is None:
        return (None, None)
    # encuentra el objeto demanda por nombre
    for d in demands:
        if d.name == best_nm:
            return (d, best_h)
    return (None, None)

def neighbor_with_lower_hop_to_demand(rb, demand_name, connections):
    if demand_name is None:
        return None
    my_h = rb.demand_hops.get(demand_name, None)
    if my_h is None:
        return None

    cand = []
    for nb in robot_neighbors(rb, connections):
        dh = getattr(nb, "demand_hops", {}).get(demand_name, None)
        if dh is not None and dh < my_h:
            dgeom = math.hypot(nb.x - rb.x, nb.y - rb.y)
            cand.append((dh, dgeom, nb.robot_id, nb))
    if not cand:
        return None
    # menor hop → más cercano → menor id
    cand.sort(key=lambda t: (t[0], t[1], t[2]))
    return cand[0][3]



def fmt_inf(v):
    return "∞" if v is None else str(v)

def summarize_robot(r):
    if r is None:
        return "None"
    parts = [f"id={r.robot_id}",
             f"total={fmt_inf(getattr(r,'total_overall',None))}",
             f"src={fmt_inf(getattr(r,'hop_from_source',None))}"]
    # include per-demand hops if available
    if hasattr(r, "demand_hops") and r.demand_hops:
        dsum = ", ".join([f"{k}:{fmt_inf(v)}" for k, v in sorted(r.demand_hops.items())])
        parts.append(f"demand_hops={{ {dsum} }}")
    return " | ".join(parts)

def top_pivot_candidates(robots, local_mins, k=5):
    """
    Returns up to k best pivot candidates (list of robots) according to pivot_key,
    preferring local minima if available, else global candidates with finite totals.
    """
    if local_mins:
        pool = list(local_mins)
    else:
        pool = [r for r in robots if getattr(r, "total_overall", None) is not None]
    pool_sorted = sorted(pool, key=pivot_key)
    return pool_sorted[:k]

def log_pivot_change(prev_pivot, new_pivot, robots, local_mins):
    changed = (prev_pivot is None and new_pivot is not None) or \
              (prev_pivot is not None and new_pivot is None) or \
              (prev_pivot is not None and new_pivot is not None and prev_pivot.robot_id != new_pivot.robot_id)
    if not changed:
        return

    print("\n=== PIVOT CHANGED ===")
    print("Prev:", summarize_robot(prev_pivot))
    print("New :", summarize_robot(new_pivot))

    # Show top ranked candidates and why they rank that way (tie-break visibility)
    print("\nTop candidates by pivot_key (showing up to 5):")
    cands = top_pivot_candidates(robots, local_mins, k=5)
    for idx, r in enumerate(cands, 1):
        total = getattr(r, "total_overall", None)
        src   = getattr(r, "hop_from_source", None)
        key_tup = pivot_key(r)   # (total, src, id)
        print(f"  {idx}. id={r.robot_id} key={key_tup} total={fmt_inf(total)} src={fmt_inf(src)}")

def draw_pivot_badge(screen, pivot, radius=12):
    if pivot is None:
        return
    # halo around the pivot (gold ring)
    pygame.draw.circle(screen, (255, 215, 0), (int(pivot.x), int(pivot.y)), radius + 6, 3)
    # small label
    font = pygame.font.Font(None, 20)
    txt = font.render(f"PIVOT {pivot.robot_id}", True, (120, 90, 0))
    screen.blit(txt, (pivot.x + 10, pivot.y - 18))

def main():
    
    pygame.init()
    clock = pygame.time.Clock()
    screen = pygame.display.set_mode((ARENA_WIDTH, ARENA_HEIGHT))
    pygame.display.set_caption("Fast joint network (Source ↔ Pivot ↔ Demands)")

    # ---- Nodes ----
    source = Node("Source", 50, ARENA_HEIGHT // 2, (255, 0, 0))
    global SOURCE_NODE
    SOURCE_NODE = source
    # ---- Demands (uses N_DEMANDS) ----
    demands = []
    demand_positions = spawn_demands_random(
        N_DEMANDS, ARENA_WIDTH, ARENA_HEIGHT, source,
        min_sep=45,   # más grande = más separados
        margin=40     # margen a los bordes
    )
    for i, (dx, dy) in enumerate(demand_positions):
        name = f"D{i+1}"
        demands.append(Node(name, dx, dy, (0, 128, 0)))

      # init reach logs
    global DEMAND_FIRST_REACH, DEMAND_REACHERS
    DEMAND_FIRST_REACH = {d.name: None for d in demands}
    DEMAND_REACHERS   = {d.name: set()  for d in demands}


    # ---- Robots (random once) ----
    robots = []
    for i in range(N_ROBOTS):
        x = random.randint(80, ARENA_WIDTH - 80)
        y = random.randint(60, ARENA_HEIGHT - 60)
        robots.append(Robot(i, x, y, ROBOT_RADIUS))
    #TRYING
    
    # ---- Build connections ONCE (simple O(n²)) ----
    connections = []
    # robot–robot
    for i in range(len(robots)):
        for j in range(i + 1, len(robots)):
            if distance(robots[i], robots[j]) <= CONNECTION_DISTANCE:
                connect(robots[i], robots[j], connections)
    # robot–source / robot–demands
    for rb in robots:
        if distance(rb, source) <= CONNECTION_DISTANCE:
            connect(rb, source, connections)
        for d in demands:
            if distance(rb, d) <= CONNECTION_DISTANCE:
                connect(rb, d, connections)

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
        for r in robots:
            r.settled = False 
        # ---- Build connections ONCE (simple O(n²)) ----
        connections = []
        # robot–robot
        for i in range(len(robots)):
            for j in range(i + 1, len(robots)):
                if distance(robots[i], robots[j]) <= CONNECTION_DISTANCE:
                    connect(robots[i], robots[j], connections)
        # robot–source / robot–demands
        for rb in robots:
            if distance(rb, source) <= CONNECTION_DISTANCE:
                connect(rb, source, connections)
            for d in demands:
                if distance(rb, d) <= CONNECTION_DISTANCE:
                    connect(rb, d, connections)

        #---------DEBUG-----------
    print("\n--- DEBUG: Robots in range ---")
    for robot in robots:
        in_range = [other.robot_id for other in robots
                    if robot != other and distance(robot, other) <= CONNECTION_DISTANCE]
        print(f" Robot {robot.robot_id} -> {in_range}")

    print("\n--- DEBUG: Source direct connections ---")
    direct_from_source = [robot.robot_id for robot in robots if distance(source, robot) <= CONNECTION_DISTANCE]
    print(f"Source -> {direct_from_source}")

    print("\n--- DEBUG: Demand direct connections ---")
    for d in demands:
        direct_from_d = [r.robot_id for r in robots if distance(d, r) <= CONNECTION_DISTANCE]
        print(f"{d.name} -> {direct_from_d}")
    
    # ---- Heavy math ONCE ----
    compute_all_hops_and_totals(source, demands, robots, connections)
    # --- Derivar hop mínimo hacia cualquier demanda (para el score del builder) ---
    for r in robots:
        vals = [v for v in getattr(r, 'demand_hops', {}).values() if v is not None]
        r.hop_from_demand = min(vals) if vals else None

    print("Local minima (by total):", [r.robot_id for r in find_local_minima(robots, connections, "total")])

    # Choose pivot robot:
    # Step 1: Filter to local minima (lowest in their neighborhood by total_overall)
    # Step 2: Pick the one with smallest total_overall.
    #         If there is a tie, pick the one closest to the source.
    #         If still tied, choose by robot_id for deterministic behavior.
    local_mins = find_local_minima(robots, connections, metric="total")
    if local_mins:
        pivot = min(local_mins, key=pivot_key)  # << ahora desempata por cercanía a la fuente
    else:
        # Fallback: pick global best using the same tie-breaking logic
        candidates = [r for r in robots if getattr(r, "total_overall", None) is not None]
        pivot = min(candidates, key=pivot_key) if candidates else None

    # Print first 30 rows to console, save full table to a file
    demand_names = [d.name for d in demands]
    dump_hop_table_once(robots, demand_names, max_rows=30, save_path="hop_table.txt")

    global demand_paths

    if pivot:
        path_S = bfs_shortest_path(source, pivot, connections)
        demand_paths = {d: bfs_shortest_path(d, pivot, connections) for d in demands}
    else:
        path_S = []
        demand_paths = {d: [] for d in demands}
    
    # === HOPS report to PIVOT ===
    if pivot:
        # 1) Hops from Source to ALL robots 
        propagate_local_hop_count(source, robots, connections,
                                attr_hop='hop_from_source_LOCAL',
                                attr_parent='par_from_source_LOCAL',
                                debug=False)
        s2p_hops = getattr(pivot, 'hop_from_source_LOCAL', None)  # Source -> Pivot (en hops)

        # 2) Hops from each Demand to ALL robots
        # (Demanda -> Pivot == Pivot -> Demanda 
        demand_to_pivot = {}   # nombre -> hops (int) o None
        for d in demands:
            attr_h = f'hop_from_{d.name}_LOCAL'
            attr_p = f'par_from_{d.name}_LOCAL'
            propagate_local_hop_count(d, robots, connections, attr_h, attr_p, debug=False)
            demand_to_pivot[d.name] = getattr(pivot, attr_h, None)

        # 3) (Opcional) También puedes verificar con el camino real que estás dibujando:
        #    edges reales = len(path) - 1
        def path_edges(path):
            return None if not path or len(path) < 2 else len(path) - 1

        s2p_edges_drawn = path_edges(path_S)
        p2d_edges_drawn = {d.name: path_edges(demand_paths.get(d, [])) for d in demands}

        # 4) Total pivot-centric = (Source->Pivot) + sum(Pivot->Di)
        #    (Si alguno es None, el total es None/∞)
        parts = [s2p_hops] + [demand_to_pivot[nm] for nm in demand_to_pivot]
        pivot_total = None if any(v is None for v in parts) else sum(parts)

        # 5) Print report
        def fmt(x): return "∞" if x is None else str(x)

        print("\n=== PIVOT-CENTRIC HOPS (usando propagate_local_hop_count) ===")
        print(f"Pivot: Robot {pivot.robot_id}")
        print("Source -> Pivot (hops):", fmt(s2p_hops))
        for nm in sorted(demand_to_pivot.keys()):
            print(f"Pivot -> {nm} (hops):", fmt(demand_to_pivot[nm]))
        print("TOTAL (Source->Pivot + Σ Pivot->Di):", fmt(pivot_total))

        print("\n[Check con caminos dibujados (edges reales)]")
        print("Source->Pivot (edges):", fmt(s2p_edges_drawn))
        for nm in sorted(p2d_edges_drawn.keys()):
            print(f"Pivot->{nm} (edges):", fmt(p2d_edges_drawn[nm]))

    """
    sale raro con esto
    # --- Rutas guiadas por hops (Opción A) ---
    if pivot:
        # Source -> Pivot
        propagate_local_hop_count(source, robots, connections,
                                'hop_from_source', 'parent_from_source')
        path_S = build_optimal_path(source, pivot, robots, connections, 'hop_from_source')

        # Demanda -> Pivot (una por una)
        demand_paths = {}
        for d in demands:
            propagate_local_hop_count(d, robots, connections,
                                    'hop_from_demand', 'parent_from_demand')
            demand_paths[d] = build_optimal_path(d, pivot, robots, connections, 'hop_from_demand')
    else:
        path_S = []
        demand_paths = {d: [] for d in demands}
    """

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
    

    # ---- Idle loop: JUST draw (no recompute) ----
    running = True
    prev_pivot = pivot
    """
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
    """
        # ---- Dynamic loop: move → rebuild → (optional) recompute → draw ----
    frame = 0
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # 1) Apply sink attraction (only after network is formed)
        apply_sink_attraction(robots, demands, robots_in_union, connections, frame)

        # 2) Rebuild connections with updated positions
        connections = rebuild_connections(robots, source, demands)

        # 3) Recompute pivot + paths every N frames
        if frame % RECOMPUTE_EVERY == 0:
            compute_all_hops_and_totals(source, demands, robots, connections)
            for r in robots:
                vals = [v for v in getattr(r, 'demand_hops', {}).values() if v is not None]
                r.hop_from_demand = min(vals) if vals else None

            #recompute pivot
            local_mins = find_local_minima(robots, connections, metric="total")
            if local_mins:
                pivot = min(local_mins, key=pivot_key)
            else:
                candidates = [r for r in robots if getattr(r, "total_overall", None) is not None]
                pivot = min(candidates, key=pivot_key) if candidates else None
            
            # --- #recompute pivot
            log_pivot_change(prev_pivot, pivot, robots, local_mins)
            prev_pivot = pivot

            if pivot:
                path_S = bfs_shortest_path(source, pivot, connections)
                demand_paths = {d: bfs_shortest_path(d, pivot, connections) for d in demands}
            else:
                path_S = []
                demand_paths = {d: [] for d in demands}

            robots_in_union = set(n for n in path_S + sum(demand_paths.values(), []) if isinstance(n, Robot))

        # Range visually detected
        check_and_color_robots(robots, demands, frame)


        
        
        # 4) Draw
        screen.fill((255, 255, 255))
        source.draw(screen)
        for d in demands:
            d.draw(screen)

        draw_poly(path_S, (255, 0, 0), 3)
        for d, p in demand_paths.items():
            draw_poly(p, (255, 0, 0), 3)

        for rb in robots:
            if getattr(rb, "at_demand", False):
                rb.draw(screen, color=(128, 0, 128))             # morado si ya llegó
            elif rb in robots_in_union:
                rb.draw(screen, color=(0, 200, 0))        # verde (en cadena dorada)
            else:
                rb.draw(screen, color=(0, 100, 255))  
        draw_pivot_badge(screen, pivot)

        pygame.display.flip()
        

        
        frame += 1

    pygame.quit()


if __name__ == "__main__":
    main()