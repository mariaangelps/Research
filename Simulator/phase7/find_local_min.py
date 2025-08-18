import pygame
import random
import math
from collections import deque
from class_robot import Robot
from class_source_and_demand import Source, Demand
# from class_obs import Obstacle  # ❌ removed

ARENA_WIDTH, ARENA_HEIGHT = 600, 300
ROBOT_RADIUS = 5
N_ROBOTS = 8
N_DEMANDS = 2
CONNECTION_DISTANCE = 120

# ❌ obstacles removed entirely
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
    dem_set = set(demands)
    visited = set()
    queue = [source]
    while queue:
        current = queue.pop(0)
        if current in dem_set:
            return True
        visited.add(current)
        for a, b in connections:
            neighbor = b if a == current else a if b == current else None
            if neighbor and neighbor not in visited:
                queue.append(neighbor)
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

# ❌ obstacle-aware helpers removed
def is_best_path_valid(path, connections):
    for i in range(len(path) - 1):
        a = path[i]
        b = path[i + 1]
        if a == b:
            continue
        if (a, b) not in connections and (b, a) not in connections:
            return False

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

    winner = min(rows, key=lambda x: (x[1], x[2])) if rows else None
    print("\n--- WHY THIS DEMAND WON (GLOBAL) ---")
    for name, min_total, tie_len, bridger in rows:
        mark = "  <= winner" if (best_demand and name == best_demand.name) else ""
        print(f"{name} | min_total: {min_total} | tie_len: {tie_len}{mark}")


def compute_per_robot_scores(robots, demands):
    """Calcula, para cada robot r:
       r.total_by_demand[name] = S(r) + D_i(r)
       r.best_total, r.best_demand_name
    """
    for r in robots:
        r.total_by_demand = {}
        # S(r)
        s = getattr(r, 'hop_from_source', None)
        for d in demands:
            di = r.demand_hops.get(d.name, None) if hasattr(r, 'demand_hops') else None
            if s is None or di is None:
                r.total_by_demand[d.name] = None
            else:
                r.total_by_demand[d.name] = s + di

        # mejor total y a qué demanda
        pairs = [(dn, v) for dn, v in r.total_by_demand.items() if v is not None]
        if pairs:
            dn, v = min(pairs, key=lambda kv: kv[1])
            r.best_demand_name = dn
            r.best_total = v
        else:
            r.best_demand_name = None
            r.best_total = None


def get_graph_neighbors(node, connections):
    """Vecinos de un nodo en el grafo de conexiones."""
    neigh = []
    for a, b in connections:
        if a == node:
            neigh.append(b)
        elif b == node:
            neigh.append(a)
    return neigh


def local_minima(robots, connections, strict=False):
    """Devuelve el conjunto de robots que son mínimos locales en T*(r).
       strict=False  -> <= (permite empates)
       strict=True   -> <  (mínimo estrictamente menor)
    """
    mins = set()
    for r in robots:
        if r.best_total is None:
            continue
        neighs = [n for n in get_graph_neighbors(r, connections) if isinstance(n, Robot)]
        if not neighs:  # aislado: por convenio lo consideramos mínimo local si tiene valor
            mins.add(r)
            continue
        if strict:
            ok = all((n.best_total is None) or (r.best_total < n.best_total) for n in neighs)
        else:
            ok = all((n.best_total is None) or (r.best_total <= n.best_total) for n in neighs)
        if ok:
            mins.add(r)
    return mins

# === ZIP de una rama anclada en un pivote (primero en la lista) ===
def y_step(branch_nodes, target_node, connection_distance,
           desired_ratio=0.85, tail_gain=1.0, follow_gain=0.8, max_step=2.0):
    """
    branch_nodes: [pivot(Node|Robot), r1(Robot), r2(Robot), ..., tail(Robot)]
    Mueve el tail hacia target_node y el resto sigue cerrando la cadena.
    El pivot (primer elemento) NO se mueve.
    """
    robots_in_branch = [n for n in branch_nodes if isinstance(n, Robot)]
    if not robots_in_branch:
        return

    tail = robots_in_branch[-1]
    # --- 1) tail hacia la demanda ---
    dx = target_node.x - tail.x
    dy = target_node.y - tail.y
    dist_d = math.hypot(dx, dy)
    if dist_d > 1e-6:
        dx /= dist_d
        dy /= dist_d

        # cuida no romper el enlace con su anterior (si existe) o con el pivot
        prev = robots_in_branch[-2] if len(robots_in_branch) >= 2 else branch_nodes[0]
        dist_prev = math.hypot(tail.x - prev.x, tail.y - prev.y)
        slack = connection_distance * 0.98 - dist_prev
        step_cap = 0.0 if slack <= 0 else min(max_step, slack)
        step = min(step_cap, tail_gain * (dist_d / max(connection_distance, 1e-6)))
        if step > 0:
            tail.x += dx * step
            tail.y += dy * step

    # --- 2) seguir al sucesor desde atrás hacia el pivot ---
    desired = desired_ratio * connection_distance
    # robots_in_branch: ... r[i] -> r[i+1] ... pivot no se mueve
    for i in range(len(robots_in_branch) - 2, -1, -1):
        cur = robots_in_branch[i]
        nxt = robots_in_branch[i + 1] if i + 1 < len(robots_in_branch) else None
        anchor = branch_nodes[0] if i == 0 else robots_in_branch[i - 1]  # para no romper con su "anterior"

        # seguir al sucesor
        dx = (nxt.x - cur.x)
        dy = (nxt.y - cur.y)
        dist = math.hypot(dx, dy)
        if dist > 1e-6:
            dx /= dist; dy /= dist

        if dist > desired:
            # no romper enlace con el "anterior"
            dist_prev = math.hypot(cur.x - anchor.x, cur.y - anchor.y)
            slack_prev = connection_distance * 0.98 - dist_prev
            step_cap = 0.0 if slack_prev <= 0 else min(max_step, slack_prev)

            # seguridad extra (aunque te acercas, no pases el límite con nxt)
            over = dist - connection_distance
            if over > 0:
                step_cap = min(step_cap, over)

            step = min(step_cap, follow_gain * ((dist - desired) / max(connection_distance, 1e-6)))
            if step > 0:
                cur.x += dx * step
                cur.y += dy * step


# === Elegir pivote (mínimo local en el camino) y preparar las dos ramas ===
def compute_y_split(source, demands, robots, connections, best_demand, best_path_from_source):
    # otra demanda (la que no ganó)
    other_demand = next(d for d in demands if d != best_demand) if len(demands) > 1 else best_demand

    # robots del camino Source->best_demand
    path_robots = [n for n in best_path_from_source if isinstance(n, Robot)]

    # candidatos a pivote: mínimos locales ∩ robots del camino
    mins = local_minima(robots, connections, strict=False)
    cand = [r for r in path_robots if r in mins]

    if cand:
        # toma el que esté más “adelante” en el camino (hacia la demanda)
        pivot = cand[-1]
    else:
        # si no hay, usa el central del camino como pivote
        pivot = path_robots[len(path_robots)//2] if path_robots else None

    # TRONCO: Source -> ... -> pivot (para dibujar)
    trunk = []
    if pivot:
        for n in best_path_from_source:
            trunk.append(n)
            if n == pivot:
                break

    # RAMA A (ya alineada hacia la best_demand): pivot -> ... (los robots posteriores del camino)
    branchA = [pivot] + [n for n in best_path_from_source[best_path_from_source.index(pivot)+1:]
                         if isinstance(n, Robot)]

    # RAMA B (nueva): usa robots libres más cercanos al pivote
    free = [r for r in robots if r not in best_path_from_source and r != pivot]
    free.sort(key=lambda r: distance(r, pivot) if pivot else 1e9)
    # tamaño parecido a la otra rama
    k = max(2, len(branchA)) - 1
    branchB = [pivot] + free[:k]

    return pivot, trunk, branchA, branchB, best_demand, other_demand

def main():
    pygame.init()
    clock = pygame.time.Clock()
    screen = pygame.display.set_mode((ARENA_WIDTH, ARENA_HEIGHT))
    pygame.display.set_caption("Hop Count - Source & Demand Optimal Path (No Obstacles)")

    source = Node("Source", 50, ARENA_HEIGHT // 2, (255, 0, 0))
    demands = []
    for i in range(N_DEMANDS):
        y_pos = 30 + i * ((ARENA_HEIGHT - 60) / (N_DEMANDS - 1))
        demands.append(Node(f"D{i+1}", ARENA_WIDTH - 50, int(y_pos), (0, 128, 0)))

    connected = False
    attempts = 0
    while not connected:
        attempts += 1
        robots = []
        connections = []

        for i in range(N_ROBOTS):
            x = random.randint(100, ARENA_WIDTH - 80)
            y = random.randint(100, ARENA_HEIGHT - 80)
            robots.append(Robot(i, x, y, ROBOT_RADIUS))

        for i in range(len(robots)):
            for j in range(i + 1, len(robots)):
                if distance(robots[i], robots[j]) <= CONNECTION_DISTANCE:
                    connect(robots[i], robots[j], connections)

        for robot in robots:
            if distance(robot, source) <= CONNECTION_DISTANCE:
                connect(robot, source, connections)
            for d in demands:
                if distance(robot, d) <= CONNECTION_DISTANCE:
                    connect(robot, d, connections)

        connected = is_path_exists(source, demands, robots, connections)

    print(f"Connected after {attempts} attempts.")

    print("\n--- DEBUGGING: Robots in range ---")
    for robot in robots:
        in_range = [other.robot_id for other in robots
                    if robot != other and distance(robot, other) <= CONNECTION_DISTANCE]
        if in_range:
            print(f" Robot {robot.robot_id} can directly connect to robots: {in_range}")
        else:
            print(f" Robot {robot.robot_id} is isolated — no robots in range")

    print("\n--- DEBUGGING: Source direct connections ---")
    direct_from_source = [robot.robot_id for robot in robots if distance(source, robot) <= CONNECTION_DISTANCE]
    print(f"Source can directly connect to robots: {direct_from_source}")

    print("\n--- DEBUGGING: Demand direct connections ---")
    for d in demands:
        direct_from_d = [r.robot_id for r in robots if distance(d, r) <= CONNECTION_DISTANCE]
        print(f"{d.name} can directly connect to robots: {direct_from_d}")

    best_demand, best_path_from_source, best_path_from_demand = choose_best_demand(
        source, demands, robots, connections
    )
    # Ya elegiste la demanda “global” y tienes hops actualizados
    compute_per_robot_scores(robots, demands)
    mins_leq = local_minima(robots, connections, strict=False)  # mínimos locales con <=
    #mins_strict = local_minima(robots, connections, strict=True)  # opcional, estrictos
    # === preparar la Y (V -> Y) ===
    pivot, trunk, branchA, branchB, demandA, demandB = compute_y_split(
        source, demands, robots, connections, best_demand, best_path_from_source
    )


    debug_global_choice(source, demands, robots, connections, best_demand)

    print("\n--- DEBUGGING BEFORE MOVEMENT---")
    demand_names = [d.name for d in demands]
    best_d_name  = best_demand.name if best_demand else None
    for r in robots:
        best_d_hop = r.demand_hops.get(best_d_name, None) if best_d_name else None
        demand_hops_str = ", ".join(f"{dn}:{r.demand_hops.get(dn, None)}" for dn in demand_names)
        per_hop = [(dn, r.demand_hops.get(dn)) for dn in demand_names if r.demand_hops.get(dn) is not None]
        robot_best_hop = min(per_hop, key=lambda x: x[1]) if per_hop else (None, None)
        per_total = [(dn, r.hop_from_source + r.demand_hops.get(dn))
                     for dn in demand_names
                     if r.hop_from_source is not None and r.demand_hops.get(dn) is not None]
        robot_best_total = min(per_total, key=lambda x: x[1]) if per_total else (None, None)
        total_global = (r.hop_from_source + best_d_hop
                        if r.hop_from_source is not None and best_d_hop is not None else None)
        print(
            f"Robot {r.robot_id} | SourceHop: {r.hop_from_source} | "
            f"DemandHops[{demand_hops_str}] | "
            f"TotalHop-Robot:{robot_best_hop[1]} | "
            f"GlobalBest:{best_d_name}@{best_d_hop} | "
            f"TotalHop-Global: {total_global}\n"
        )
        print()
    print("------------------")
    
    print("\n--- LOCAL MINIMA DEBUG ---")
    for r in robots:
        totals_str = ", ".join(f"{dn}:{r.total_by_demand.get(dn)}" for dn in [d.name for d in demands])
        tag = "MIN_local" if r in mins_leq else ""
        print(f"R{r.robot_id:02d} S:{r.hop_from_source} | {totals_str} | "
            f"Best({r.best_demand_name}={r.best_total}) {tag}")
    print("Mínimos locales (<=):", sorted([r.robot_id for r in mins_leq]))
    #print("Mínimos locales estrictos (<):", sorted([r.robot_id for r in mins_strict]))


    # Initial draw (no obstacles)
    screen.fill((255, 255, 255))
    source.draw(screen)
    for d in demands:
        d.draw(screen)

    for a, b in connections:
        pygame.draw.line(screen, (210, 210, 210), (a.x, a.y), (b.x, b.y), 1)

    for i in range(len(best_path_from_source) - 1):
        pygame.draw.line(screen, (255, 0, 0), (best_path_from_source[i].x, best_path_from_source[i].y),
                         (best_path_from_source[i + 1].x, best_path_from_source[i + 1].y), 3)

    for i in range(len(best_path_from_demand) - 1):
        pygame.draw.line(screen, (0, 100, 255), (best_path_from_demand[i].x, best_path_from_demand[i].y),
                         (best_path_from_demand[i + 1].x, best_path_from_demand[i + 1].y), 3)

    robots_in_path = {r for r in best_path_from_source if isinstance(r, Robot)}
    """
    for robot in robots:
        if robot in robots_in_path:
            robot.draw(screen, color=(0, 200, 0))  # green
        else:
            robot.draw(screen, color=(0, 100, 255))  # blue
    """
    for robot in robots:
        # color base (como ya lo tenías)
        base_color = (0, 200, 0) if robot in robots_in_path else (0, 100, 255)
        robot.draw(screen, color=base_color)

        # contorno dorado = mínimo local (<=)
        if robot in mins_leq:
            pygame.draw.circle(screen, (255, 215, 0), (int(robot.x), int(robot.y)), ROBOT_RADIUS + 6, 3)
        """
        # etiqueta corta con T*(r)
        font = pygame.font.Font(None, 18)
        txt = "∞" if robot.best_total is None else str(robot.best_total)
        lbl = font.render(txt, True, (0, 0, 0))
        screen.blit(lbl, (robot.x + 8, robot.y - 8))
        """
        # si quieres ver por-demanda: T1/T2
        # dnames = [d.name for d in demands]
        # t1 = robot.total_by_demand.get(dnames[0])
        # t2 = robot.total_by_demand.get(dnames[1]) if len(dnames) > 1 else None
        # lbl2 = font.render(f"{t1}/{t2}", True, (80,80,80))
        # screen.blit(lbl2, (robot.x + 8, robot.y + 6))

    pygame.display.flip()
    clock.tick(60)
    
    # WAIT FOR KEY PRESS BEFORE CONTINUING (kept, but no obstacle forces will run)
    
    waiting = True

    #print("\nPress any key to continue...\n")
    
    while waiting:
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                waiting = False
            elif event.type == pygame.QUIT:
                pygame.quit()
                return
    
    # Main loop (no obstacle physics)
    running = True
    while running:
        screen.fill((255, 255, 255))
        source.draw(screen)
        for d in demands:
            d.draw(screen)
        # === animación: dos ramas zipper desde el pivote a las dos demandas ===
        if pivot:
            y_step(branchA, demandA, CONNECTION_DISTANCE,
                desired_ratio=0.85, tail_gain=1.0, follow_gain=0.85, max_step=2.5)
            y_step(branchB, demandB, CONNECTION_DISTANCE,
                desired_ratio=0.85, tail_gain=1.0, follow_gain=0.85, max_step=2.5)

        # Recompute connections in case you later add movement
        connections = []
        for i in range(len(robots)):
            for j in range(i + 1, len(robots)):
                if distance(robots[i], robots[j]) <= CONNECTION_DISTANCE:
                    connect(robots[i], robots[j], connections)

        for robot in robots:
            if distance(robot, source) <= CONNECTION_DISTANCE:
                connect(robot, source, connections)
            for d in demands:
                if distance(robot, d) <= CONNECTION_DISTANCE:
                    connect(robot, d, connections)

        best_demand, best_path_from_source, best_path_from_demand = choose_best_demand(
            source, demands, robots, connections
        )

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        for a, b in connections:
            pygame.draw.line(screen, (210, 210, 210), (a.x, a.y), (b.x, b.y), 1)
        
        # TRONCO (Source -> pivot)
        for i in range(len(trunk) - 1):
            pygame.draw.line(screen, (50, 50, 50), (trunk[i].x, trunk[i].y), (trunk[i+1].x, trunk[i+1].y), 3)

        # RAMA A hacia demandA (naranja)
        seqA = [branchA[0]] + [r for r in branchA[1:] if isinstance(r, Robot)]
        for i in range(len(seqA) - 1):
            pygame.draw.line(screen, (255, 140, 0), (seqA[i].x, seqA[i].y), (seqA[i+1].x, seqA[i+1].y), 3)
        # cierre a la demanda
        if pivot and distance(seqA[-1], demandA) <= CONNECTION_DISTANCE:
            pygame.draw.line(screen, (255, 140, 0), (seqA[-1].x, seqA[-1].y), (demandA.x, demandA.y), 3)

        # RAMA B hacia demandB (azul)
        seqB = [branchB[0]] + [r for r in branchB[1:] if isinstance(r, Robot)]
        for i in range(len(seqB) - 1):
            pygame.draw.line(screen, (0, 100, 255), (seqB[i].x, seqB[i].y), (seqB[i+1].x, seqB[i+1].y), 3)
        if pivot and distance(seqB[-1], demandB) <= CONNECTION_DISTANCE:
            pygame.draw.line(screen, (0, 100, 255), (seqB[-1].x, seqB[-1].y), (demandB.x, demandB.y), 3)

        # Colores de robots: tronco = gris oscuro, rama A = naranja, rama B = azul, otros = gris claro
        set_trunk = {n for n in trunk if isinstance(n, Robot)}
        set_A = {n for n in branchA if isinstance(n, Robot)}
        set_B = {n for n in branchB if isinstance(n, Robot)}

        for r in robots:
            if r in set_A:
                r.draw(screen, color=(255, 140, 0))
            elif r in set_B:
                r.draw(screen, color=(0, 100, 255))
            elif r in set_trunk:
                r.draw(screen, color=(70, 70, 70))
            else:
                r.draw(screen, color=(160, 160, 160))
        """
        for i in range(len(best_path_from_source) - 1):
            pygame.draw.line(screen, (255, 0, 0), (best_path_from_source[i].x, best_path_from_source[i].y),
                             (best_path_from_source[i + 1].x, best_path_from_source[i + 1].y), 3)
        for i in range(len(best_path_from_demand) - 1):
            pygame.draw.line(screen, (0, 100, 255), (best_path_from_demand[i].x, best_path_from_demand[i].y),
                             (best_path_from_demand[i + 1].x, best_path_from_demand[i + 1].y), 3)

        robots_in_path = {r for r in best_path_from_source if isinstance(r, Robot)}
        for robot in robots:
            if robot in robots_in_path:
                robot.draw(screen, color=(0, 200, 0))
            else:
                robot.draw(screen, color=(0, 100, 255))
        """
        pygame.display.flip()

    print("\n>>> Final Best Path Source ➔ Demand:")
    print([get_node_name(r) for r in best_path_from_source])
    print("\n>>> Final Best Path Demand ➔ Source:")
    print([get_node_name(r) for r in best_path_from_demand])
    print(f"\n>>> Best demand chosen: {best_demand.name if best_demand else 'None'}")
    pygame.quit()

if __name__ == "__main__":
    main()
