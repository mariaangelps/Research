import pygame
import random
import math
from collections import deque
from class_robot import Robot
from class_source_and_demand import Source, Demand
# from class_obs import Obstacle  # ❌ removed

ARENA_WIDTH, ARENA_HEIGHT = 600, 300
ROBOT_RADIUS = 5
N_ROBOTS = 40
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
    for robot in robots:
        if robot in robots_in_path:
            robot.draw(screen, color=(0, 200, 0))  # green
        else:
            robot.draw(screen, color=(0, 100, 255))  # blue

    pygame.display.flip()
    clock.tick(60)

    # WAIT FOR KEY PRESS BEFORE CONTINUING (kept, but no obstacle forces will run)
    waiting = True
    print("\nPress any key to continue...\n")
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

        pygame.display.flip()

    print("\n>>> Final Best Path Source ➔ Demand:")
    print([get_node_name(r) for r in best_path_from_source])
    print("\n>>> Final Best Path Demand ➔ Source:")
    print([get_node_name(r) for r in best_path_from_demand])
    print(f"\n>>> Best demand chosen: {best_demand.name if best_demand else 'None'}")
    pygame.quit()

if __name__ == "__main__":
    main()
