import pygame
import random
import math
from collections import deque
from class_robot import Robot
from class_source_and_demand import Source, Demand
# from class_obs import Obstacle  

ARENA_WIDTH, ARENA_HEIGHT = 600, 300
ROBOT_RADIUS = 2
N_ROBOTS = 120
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

    winner = min(rows, key=lambda x: (x[1], x[2])) if rows else None
    print("\n--- WHY THIS DEMAND WON (GLOBAL) ---")
    for name, min_total, tie_len, bridger in rows:
        mark = "  <= winner" if (best_demand and name == best_demand.name) else ""
        print(f"{name} | min_total: {min_total} | tie_len: {tie_len}{mark}")
        
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

def main():
    pygame.init()
    clock = pygame.time.Clock()
    screen = pygame.display.set_mode((ARENA_WIDTH, ARENA_HEIGHT))
    pygame.display.set_caption("Hop Count - Joint Network to D1 & D2 (No Obstacles)")

    # Nodos fijos
    source = Node("Source", 50, ARENA_HEIGHT // 2, (255, 0, 0))
    demands = []
    for i in range(N_DEMANDS):
        y_pos = 30 + i * ((ARENA_HEIGHT - 60) / (N_DEMANDS - 1))
        demands.append(Node(f"D{i+1}", ARENA_WIDTH - 50, int(y_pos), (0, 128, 0)))

    # ---------- Construcción de un grafo que conecte Source con TODAS las Demands ----------
    connected = False
    attempts = 0
    while not connected:
        attempts += 1
        robots = []
        connections = []

        # Robots aleatorios
        for i in range(N_ROBOTS):
            x = random.randint(100, ARENA_WIDTH - 80)
            y = random.randint(100, ARENA_HEIGHT - 80)
            robots.append(Robot(i, x, y, ROBOT_RADIUS))

        # Conexiones robot-robot
        for i in range(len(robots)):
            for j in range(i + 1, len(robots)):
                if distance(robots[i], robots[j]) <= CONNECTION_DISTANCE:
                    connect(robots[i], robots[j], connections)

        # Conexiones a Source y a cada Demand
        for robot in robots:
            if distance(robot, source) <= CONNECTION_DISTANCE:
                connect(robot, source, connections)
            for d in demands:
                if distance(robot, d) <= CONNECTION_DISTANCE:
                    connect(robot, d, connections)

        # ¿Source alcanza a TODAS las demandas?
        connected = is_path_exists(source, demands, robots, connections)

    print(f"Connected after {attempts} attempts.")

    # ---------- DEBUG rápido ----------
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

    # Tabla de hops y elección de pivote (minimiza Src→r + Σ Di→r)
    compute_all_hops_and_totals(source, demands, robots, connections)
    print_hop_table(robots, [d.name for d in demands])
    pivot = choose_pivot_robot(robots)
    if pivot:
        print(f"\n>>> PIVOT ROBOT: {pivot.robot_id} (total_overall={pivot.total_overall})")
    else:
        print("\n>>> No hay pivote (algún hop es inaccesible).")

    # ---------- DIBUJO INICIAL (red conjunta S–Pivot–D1/D2) ----------
    def draw_poly(path, color=(255, 0, 0), width=3):
        for i in range(len(path) - 1):
            pygame.draw.line(screen, color, (path[i].x, path[i].y), (path[i+1].x, path[i+1].y), width)

    screen.fill((255, 255, 255))
    source.draw(screen)
    for d in demands:
        d.draw(screen)

    # Caminos por hops mínimos
    if pivot:
        path_S = bfs_shortest_path(source, pivot, connections)
        demand_paths = {d: bfs_shortest_path(d, pivot, connections) for d in demands}
    else:
        path_S = []
        demand_paths = {d: [] for d in demands}

    # Grafo en gris
    for a, b in connections:
        pygame.draw.line(screen, (210, 210, 210), (a.x, a.y), (b.x, b.y), 1)

    # Red unida en rojo
    draw_poly(path_S, (255, 0, 0), 3)
    for d, p in demand_paths.items():
        draw_poly(p, (255, 0, 0), 3)

    # Robots que pertenecen a la red (verde)
    robots_in_union = set(n for n in path_S + sum(demand_paths.values(), []) if isinstance(n, Robot))
    for robot in robots:
        if robot in robots_in_union:
            robot.draw(screen, color=(0, 200, 0))
        else:
            robot.draw(screen, color=(0, 100, 255))

    pygame.display.flip()
    clock.tick(60)

    # Espera tecla
    waiting = True
    print("\nPress any key to continue...\n")
    while waiting:
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                waiting = False
            elif event.type == pygame.QUIT:
                pygame.quit()
                return

    # ---------- LOOP PRINCIPAL (si luego mueves robots, recalcula y redibuja) ----------
    running = True
    while running:
        screen.fill((255, 255, 255))
        source.draw(screen)
        for d in demands:
            d.draw(screen)

        # Recalcular conexiones por si cambias posiciones en el futuro
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

        # Recalcular hops, pivote y caminos
        compute_all_hops_and_totals(source, demands, robots, connections)
        pivot = choose_pivot_robot(robots)
        if pivot:
            path_S = bfs_shortest_path(source, pivot, connections)
            demand_paths = {d: bfs_shortest_path(d, pivot, connections) for d in demands}
        else:
            path_S = []
            demand_paths = {d: [] for d in demands}

        # Eventos
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # Dibujo del grafo
        for a, b in connections:
            pygame.draw.line(screen, (210, 210, 210), (a.x, a.y), (b.x, b.y), 1)

        # Dibujo de la red conjunta
        draw_poly(path_S, (255, 0, 0), 3)
        for d, p in demand_paths.items():
            draw_poly(p, (255, 0, 0), 3)

        robots_in_union = set(n for n in path_S + sum(demand_paths.values(), []) if isinstance(n, Robot))
        for robot in robots:
            if robot in robots_in_union:
                robot.draw(screen, color=(0, 200, 0))
            else:
                robot.draw(screen, color=(0, 100, 255))

        pygame.display.flip()

    # Al salir, imprime los caminos
    print("\n>>> Path Source → Pivot:")
    print([get_node_name(n) for n in path_S])
    for d, p in demand_paths.items():
        print(f">>> Path {d.name} → Pivot:")
        print([get_node_name(n) for n in p])

    pygame.quit()

if __name__ == "__main__":
    main()