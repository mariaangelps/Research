import pygame
import random
import math
from collections import deque
from class_robot import Robot
from class_source_and_demand import Source, Demand
from class_obs import Obstacle

ARENA_WIDTH, ARENA_HEIGHT = 600, 300
ROBOT_RADIUS = 5
N_ROBOTS = 20
#N_EXTRA_ROBOTS = 10
N_DEMANDS = 2
CONNECTION_DISTANCE = 120
# create random obstacles
"""
obstacles = []
for _ in range(2):
    x = random.randint(150, ARENA_WIDTH - 150)
    y = random.randint(100, ARENA_HEIGHT - 100)
    obstacles.append(Obstacle(x, y))

"""


#with obstacles 
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
    #print(f"\nStarting local token-passing from: {source.name}")
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

            # Only update if it's the first time or we found a shorter path
            if existing_hop is not None and hops >= existing_hop:
                continue

            setattr(current, attr_hop, hops)
            setattr(current, attr_parent, parent if isinstance(parent, Robot) else None)

            """"
            print(f"\nRobot {current.robot_id} received token | Hop: {hops} | From: {parent.robot_id if isinstance(parent, Robot) else 'Source'}")
            """
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
    """True si desde el source se puede alcanzar TODAS las demands por las conexiones."""
    visited = set([source])
    queue = [source]
    while queue:
        cur = queue.pop(0)
        for a, b in connections:
            nb = b if a == cur else a if b == cur else None
            if nb and nb not in visited:
                visited.add(nb)
                queue.append(nb)
    return all(d in visited for d in demands)

#function to verify if there is a direct path 
def existe_ruta_fisica(a, b, conexiones_fisicas):
    visitado = set()
    #queue
    cola = deque()
    cola.append(a)
    #visited
    visitado.add(a)

    while cola:
        actual = cola.popleft()
        if actual == b:
            return True

        #neighbors
        vecinos = [
            r2 for (r1, r2) in conexiones_fisicas if r1 == actual
        ] + [
            r1 for (r1, r2) in conexiones_fisicas if r2 == actual
        ]

        for vecino in vecinos:
            if vecino not in visitado:
                visitado.add(vecino)
                cola.append(vecino)

    return False
def get_node_name(n):
    return n.name if isinstance(n, Node) else f"Robot {n.robot_id}"

"""""
def is_in_obstacle_range(robot, obstacles, danger_radius):

    ALLOW_MARGIN = 0.97
    for obs in obstacles:
        dist = math.hypot(robot.x - obs.x, robot.y - obs.y)
        if dist < danger_radius * ALLOW_MARGIN:
            return True
    return False


def apply_virtual_forces(robots, obstacles, best_path, connection_distance, optimal_path_links):

    # --- 1. Repulsion from obstacles ---
    for robot in robots:
        for obstacle in obstacles:
            dx = robot.x - obstacle.x
            dy = robot.y - obstacle.y
            dist = math.hypot(dx, dy)
            if dist < 60:
                dx /= dist
                dy /= dist

                repel_strength =1*(60 -dist)
                rvector = (dx, dy)

                robot.x += dx * repel_strength
                robot.y += dy * repel_strength

    
    # --- 2. Attraction to path neighbors (only robots in best path) ---
    for idx, robot in enumerate(best_path):
        if not isinstance(robot, Robot):
            continue

        # Find previous and next robots in path
        neighbors = []
        if idx > 0 and isinstance(best_path[idx - 1], Robot):
            neighbors.append(best_path[idx - 1])
        if idx < len(best_path) - 1 and isinstance(best_path[idx + 1], Robot):
            neighbors.append(best_path[idx + 1])

        for neighbor in neighbors:
            dx = neighbor.x - robot.x
            dy = neighbor.y - robot.y
            dist = math.hypot(dx, dy)
            
            # --- Attraction if out of range ---
            if dist > connection_distance:
                dx /= dist
                dy /= dist
                attraction_strength = 0.5 * (dist - connection_distance) / connection_distance
                robot.x += dx * attraction_strength
                robot.y += dy * attraction_strength

            # --- Optional: gentle repulsion if too close ---
            elif dist < connection_distance * 0.5 and dist != 0:
                dx /= dist
                dy /= dist
                repel_strength = 0.2 * (connection_distance * 0.5 - dist) / connection_distance
                robot.x -= dx * repel_strength
                robot.y -= dy * repel_strength

            

        # 3. Reconnect isolated golden-path robots (optional recovery)
    for robot in robots:
        if robot not in best_path or not isinstance(robot, Robot):
            continue

        # Check if robot has any golden path neighbor in range
        has_neighbor = any(
            isinstance(neigh, Robot) and neigh != robot and distance(robot, neigh) <= connection_distance
            for neigh in best_path
        )

        if not has_neighbor:
            # Pull toward the closest robot in best path
            closest = None
            min_dist = float('inf')
            for other in best_path:
                if other == robot or not isinstance(other, Robot):
                    continue
                d = distance(robot, other)
                if d < min_dist:
                    min_dist = d
                    closest = other

            if closest:
                dx = closest.x - robot.x
                dy = closest.y - robot.y
                dist = math.hypot(dx, dy)
                if dist != 0:
                    dx /= dist
                    dy /= dist
                    pull_strength = 0.3 * (dist / connection_distance)
                    robot.x += dx * pull_strength
                    robot.y += dy * pull_strength
    # --- 4. Restoring force for previously connected robots ---
    # --- 4. Restoring force for previously connected robots ---
    for idx in range(len(best_path) - 1):
        r1 = best_path[idx]
        r2 = best_path[idx + 1]

        if isinstance(r1, Robot) and isinstance(r2, Robot):
            dx = r2.x - r1.x
            dy = r2.y - r1.y
            dist = math.hypot(dx, dy)

            if dist > connection_distance:
                dx /= dist
                dy /= dist

                # increase force if they are too far
                strength_factor = min(1.5, (dist - connection_distance) / connection_distance)
                restoring_strength = 1.0 * strength_factor  # you can bump this up

                # Pull both toward each other
                r1.x += dx * restoring_strength * 0.5
                r1.y += dy * restoring_strength * 0.5
                r2.x -= dx * restoring_strength * 0.5
                r2.y -= dy * restoring_strength * 0.5


    
    # --- 5. Restoring force for fixed optimal path connections ---
    for (r1, r2) in optimal_path_links:
        if not isinstance(r1, Robot) or not isinstance(r2, Robot):
            continue

        dx = r2.x - r1.x
        dy = r2.y - r1.y
        dist = math.hypot(dx, dy)

        if dist > 0.8 * connection_distance:  # even if still "connected", keep them close
            dx /= dist
            dy /= dist

            restoring_force = 1.5 * (dist - connection_distance) / connection_distance

            r1.x += dx * restoring_force * 0.5
            r1.y += dy * restoring_force * 0.5
            r2.x -= dx * restoring_force * 0.5
            r2.y -= dy * restoring_force * 0.5
"""
# --- ENFORCE CONNECTION CONSTRAINTS ---
# This runs after computing the forces, but before applying movement.
# It ensures robots don't break any existing fixed connection due to motion.
# --- ENFORCE CONNECTION CONSTRAINTS ---
# Enhanced version that reverts all robots if any connection would break
"""
def enforce_connection_constraints(robots, fixed_connections, connection_distance):
    # Store previous positions for all robots
    for robot in robots:
        robot.prev_x, robot.prev_y = robot.x, robot.y

    # Track if any constraint is violated
    any_violation = False

    for (r1, r2) in fixed_connections:
        if not isinstance(r1, Robot) or not isinstance(r2, Robot):
            continue

        dx = r2.x - r1.x
        dy = r2.y - r1.y
        dist = math.hypot(dx, dy)

        if dist > connection_distance:
            any_violation = True
            break

    # If any connection would break, revert all robots to previous positions
    if any_violation:
        for robot in robots:
            robot.x, robot.y = robot.prev_x, robot.prev_y

"""
def is_best_path_valid(path, connections):
    """Checks if all consecutive elements in the path are still physically connected."""
    #print("🔍 Path being validated:", [get_node_name(p) for p in path])

    for i in range(len(path) - 1):
        a = path[i]
        b = path[i + 1]
        if a == b:
            continue  # Skip duplicate nodes
        if (a, b) not in connections and (b, a) not in connections:
            #print(f"⚠️ Broken connection between {get_node_name(a)} and {get_node_name(b)}")
            #print(f"  - Distance: {distance(a, b):.2f}")
            return False





def build_optimal_path(start, end, robots, connections, hop_attr):
    path = []
    visited = set()
    current = start

    current_hop = 0 if not isinstance(start, Robot) else getattr(start, hop_attr)

    while current != end:
        visited.add(current)
        # Priority: if robot can directly sense Demand, connect immediately
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
                # check its directly connected 
                if existe_ruta_fisica(current, r, connections):
                    # Total hop = source + demand
                    source_hop = getattr(r, 'hop_from_source')
                    demand_hop = getattr(r, 'hop_from_demand')

                    # Treat None as infinity
                    if source_hop is None:
                        source_hop = float('inf')
                    if demand_hop is None:
                        demand_hop = float('inf')

                    total_hop = source_hop + demand_hop

                    candidates.append((total_hop, r))

        if not candidates:
            break

        #  choose robot with least total hop count
        #_, best_next = min(candidates, key=lambda x: x[0])
        def connection_score(robot, connections):
            vecinos = [
                r2 for (r1, r2) in connections if r1 == robot and isinstance(r2, Robot)
            ] + [
                r1 for (r1, r2) in connections if r2 == robot and isinstance(r1, Robot)
            ]
            return -len(vecinos)

        _, best_next = min(candidates, key=lambda x: (x[0], connection_score(x[1], connections)))


        path.append(best_next)
        visited.add(best_next)
        current = best_next
        current_hop += 1

    """"
    # connect to the end 
    if isinstance(end, Node):
        last = path[-1] if path else None
        if last and existe_ruta_fisica(last, end, connections):
            path.append(end)
    """
    # Conecta to the beginning 
    if isinstance(start, Node):
        first = path[0] if path else None
        if first and first != start and distance(start, first) <= CONNECTION_DISTANCE:
            path.insert(0, start)

    return path

def compute_multi_demand_hops(demands, robots, connections):
    """
    Para cada robot:
      - r.hops_to_demand[d] = hop específico hacia d
      - r.hop_from_demand   = min_d r.hops_to_demand[d]      (para debug/global)
      - r.nearest_demand    = argmin_d r.hops_to_demand[d]
    """
    for r in robots:
        r.hops_to_demand = {}   # <- mapa específico
        r.hop_from_demand = None
        r.nearest_demand = None

    for d in demands:
        hop = {d: 0}
        visited = set([d])
        q = deque([d])

        while q:
            cur = q.popleft()
            vecinos = (
                [r2 for (r1, r2) in connections if r1 == cur] +
                [r1 for (r1, r2) in connections if r2 == cur]
            )
            for nb in vecinos:
                if nb in visited:
                    continue
                visited.add(nb)
                hop[nb] = hop[cur] + 1
                q.append(nb)

        # guardar hop específico hacia d
        for r in robots:
            if r in hop:
                r.hops_to_demand[d] = hop[r]

    # derivar min global (opcional, para prints)
    for r in robots:
        if r.hops_to_demand:
            dmin, vmin = min(r.hops_to_demand.items(), key=lambda kv: kv[1])
            r.hop_from_demand = vmin
            r.nearest_demand = dmin
        else:
            r.hop_from_demand = None
            r.nearest_demand = None
def build_network_for_demands(demands, source, robots, connections):
    """
    Devuelve una lista de rutas (una por demand target),
    usando hops específicos al target, no el mínimo global.
    """
    uncovered = set(demands)
    network_paths = []

    def cost_to(d):
        # costo: min_r (hop_source(r) + hop_especifico_d(r))
        best = float('inf')
        for r in robots:
            hs = getattr(r, 'hop_from_source', None)
            hd = r.hops_to_demand.get(d, None) if hasattr(r, 'hops_to_demand') else None
            if hs is None or hd is None:
                continue
            best = min(best, hs + hd)
        return best

    while uncovered:
        target = min(uncovered, key=cost_to)

        # preparar atributo temporal con hops hacia 'target'
        for r in robots:
            r.hop_to_target = r.hops_to_demand.get(target, None) if hasattr(r, 'hops_to_demand') else None

        # ahora el gradient dentro de build_optimal_path apunta a 'target'
        path = build_optimal_path(source, target, robots, connections, 'hop_to_target')

        if path and len(path) >= 2:
            network_paths.append(path)
            uncovered.remove(target)
        else:
            # si no hay ruta para esta demand, salimos para no ciclar
            break

    # (opcional) limpiar el atributo temporal
    for r in robots:
        if hasattr(r, 'hop_to_target'):
            delattr(r, 'hop_to_target')

    return network_paths
def choose_branch_robot(demands, robots):
    """
    Elige el robot que minimiza: hop_from_source(r) + sum_d hops_to_demand[d](r)
    (Requiere que ya estén calculados hop_from_source y hops_to_demand)
    """
    best = None
    best_cost = float('inf')
    for r in robots:
        hs = getattr(r, 'hop_from_source', None)
        if hs is None:
            continue
        # deben existir hops específicos a cada demand
        if not hasattr(r, 'hops_to_demand'):
            continue
        ok = all(d in r.hops_to_demand for d in demands)
        if not ok:
            continue
        cost = hs + sum(r.hops_to_demand[d] for d in demands)
        if cost < best_cost:
            best_cost = cost
            best = r
    return best
def build_shared_trunk_network(demands, source, robots, connections):
    """
    Devuelve una lista de rutas: [ tronco+rama(D1), tronco+rama(D2), ... ]
    donde el tronco Source→branch se comparte.
    """
    branch = choose_branch_robot(demands, robots)
    if branch is None:
        return []  # no hay punto que conecte todo

    # --- Tronco: Source -> branch (end es Robot)
    trunk = build_optimal_path(source, branch, robots, connections, 'hop_from_source')

    network_paths = []
    for d in demands:
        # preparar gradiente específico al target d
        for r in robots:
            r.hop_to_target = r.hops_to_demand.get(d, None) if hasattr(r, 'hops_to_demand') else None

        # --- Rama: branch -> d (start = Robot, end = Node)
        limb = build_optimal_path(branch, d, robots, connections, 'hop_to_target')

        # unir tronco + rama (evitar duplicar el branch)
        full_path = list(trunk) + limb  # limb no incluye el start Robot explícitamente
        network_paths.append(full_path)

    # limpiar atributo temporal
    for r in robots:
        if hasattr(r, 'hop_to_target'):
            delattr(r, 'hop_to_target')

    return network_paths

def build_path_after_repulsion(start, end, robots, connections, hop_attr):
    path = []
    visited = set()
    current = start
    current_hop = 0 if not isinstance(start, Robot) else getattr(start, hop_attr)

    while current != end:
        visited.add(current)

        # Soporta lista de demands o una sola
        ends = end if isinstance(end, (list, tuple, set)) else [end]
        touching = None
        for e in ends:
            if isinstance(e, Node) and distance(current, e) <= CONNECTION_DISTANCE:
                touching = e
                break

        if touching is not None:
            if not path or path[-1] != current:
                path.append(current)
            path.append(touching)
            break


        candidates = []

        # Buscar solo vecinos directos de current
        neighbors = [
            r2 for (r1, r2) in connections if r1 == current and isinstance(r2, Robot)
        ] + [
            r1 for (r1, r2) in connections if r2 == current and isinstance(r1, Robot)
        ]

        for r in neighbors:
            if r in visited:
                continue
            if getattr(r, hop_attr) == current_hop + 1:
                source_hop = getattr(r, 'hop_from_source') or float('inf')
                demand_hop = getattr(r, 'hop_from_demand') or float('inf')
                total_hop = source_hop + demand_hop
                candidates.append((total_hop, r))

        if not candidates:
            break

        #candidates.sort(key=lambda x: (x[0], x[1].robot_id))
        #best_next = candidates[0][1]
        def connection_score(robot, connections):
            vecinos = [
                r2 for (r1, r2) in connections if r1 == robot and isinstance(r2, Robot)
            ] + [
                r1 for (r1, r2) in connections if r2 == robot and isinstance(r1, Robot)
            ]
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


obstacles_active = True  # as soon as virtual forces are used





def main():
    pygame.init()
    screen = pygame.display.set_mode((ARENA_WIDTH, ARENA_HEIGHT))
    pygame.display.set_caption("Hop Count - Source & Demand Optimal Path")

    source = Node("Source", 50, ARENA_HEIGHT // 2, (255, 0, 0))
    demands=[]
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
        """
        for j in range(N_EXTRA_ROBOTS):
            x = random.randint(100, ARENA_WIDTH - 80)
            y = random.randint(100, ARENA_HEIGHT - 80)
            robots.append(Robot(N_ROBOTS + j, x, y, ROBOT_RADIUS))
        """
        for i in range(len(robots)):
            for j in range(i + 1, len(robots)):
                if distance(robots[i], robots[j]) <= CONNECTION_DISTANCE:

                    #if not is_in_obstacle_range(robots[i], obstacles, CONNECTION_DISTANCE) and not is_in_obstacle_range(robots[j], obstacles, CONNECTION_DISTANCE):
                        connect(robots[i], robots[j], connections)


        
        #assures robots are inside the conection range
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
            in_range = []
            for other in robots:
                if robot != other and distance(robot, other) <= CONNECTION_DISTANCE:
                    in_range.append(other.robot_id)
            if in_range:
                print(f" Robot {robot.robot_id} can directly connect to robots: {in_range}")
            else:
                print(f" Robot {robot.robot_id} is isolated — no robots in range")

    print("\n--- DEBUGGING: Source direct connections ---")
    direct_from_source = []
    for robot in robots:
        if distance(source, robot) <= CONNECTION_DISTANCE:
            direct_from_source.append(robot.robot_id)
    print(f"Source can directly connect to robots: {direct_from_source}")

    print("\n--- DEBUGGING: Demand direct connections ---")
    direct_from_demand = []
    for d in demands:  # ✅ imprime por cada demanda
        direct_from_d = [r.robot_id for r in robots if distance(d, r) <= CONNECTION_DISTANCE]
        print(f"{d.name} can directly connect to robots: {direct_from_d}")

    print(f"Demand can directly connect to robots: {direct_from_demand}")


    propagate_local_hop_count(source, robots, connections, 'hop_from_source', 'parent_from_source')
    compute_multi_demand_hops(demands, robots, connections)
        # Red (múltiples caminos, uno por demand)
    network_paths = build_shared_trunk_network(demands, source, robots, connections)

    # (opcional) set de aristas si quieres usarlas luego
    network_edges = set()
    for path in network_paths:
        for a, b in zip(path, path[1:]):
            network_edges.add((a, b))
            network_edges.add((b, a))
    print("\n--- DEBUGGING ---")
    for r in robots:
        total_hops = None
        if r.hop_from_source is not None and r.hop_from_demand is not None:
            total_hops = r.hop_from_source + r.hop_from_demand
        print(f"Robot {r.robot_id} | SourceHop: {r.hop_from_source} | DemandHop: {r.hop_from_demand} | TotalHop: {total_hops}")
    print("------------------")

    """
    best_path_from_source = build_optimal_path(source, demands, robots, connections, 'hop_from_source')
    best_path_from_demand = build_optimal_path(demands, source, robots, connections, 'hop_from_demand') 
    

    fixed_connections = set()
    for i in range(len(best_path_from_source) - 1):
        a = best_path_from_source[i]
        b = best_path_from_source[i + 1]
        
        # Siempre añade el par, incluso si es Source o Demand
        if isinstance(a, (Robot, Node)) and isinstance(b, (Robot, Node)):
            # Para evitar duplicados, ordena por id si ambos son robots
            if isinstance(a, Robot) and isinstance(b, Robot):
                pair = (a, b) if a.robot_id < b.robot_id else (b, a)
            else:
                pair = (a, b)  # Mantén el orden para incluir Source/Demand con robots
            fixed_connections.add(pair)

        # También agregar las conexiones del camino inverso Demand ➔ Source
    for i in range(len(best_path_from_demand) - 1):
        a = best_path_from_demand[i]
        b = best_path_from_demand[i + 1]

        if isinstance(a, (Robot, Node)) and isinstance(b, (Robot, Node)):
            if isinstance(a, Robot) and isinstance(b, Robot):
                pair = (a, b) if a.robot_id < b.robot_id else (b, a)
            else:
                pair = (a, b)
            fixed_connections.add(pair)


            
    """




    #debugging
    """
    print("\n Verificando conexión física del camino Source ➔ Demand:")
    for i in range(len(best_path_from_source) - 1):
        a = best_path_from_source[i]
        b = best_path_from_source[i + 1]
        if existe_ruta_fisica(a, b, connections):
            print(f"Conexión física entre {get_node_name(a)} y {get_node_name(b)}")
        else:
            print(f"SIN conexión física entre {get_node_name(a)} y {get_node_name(b)}")
    """

    """
    print("\n>>> Best Path Source ➔ Demand:")
    print([get_node_name(r) for r in best_path_from_source])

    print("\n>>> Best Path Demand ➔ Source :")
    print([get_node_name(r) for r in best_path_from_demand])
    """
    """
    print("\n>>> Robots in ORANGE Path (Source ➔ Demand):")
    orange_path_src = [f"R{n.robot_id}" for n in best_path_from_source if isinstance(n, Robot)]
    print(" ➔ ".join(orange_path_src))
   
    print("\n>>> Robots in ORANGE Path (Demand ➔ Source):")
    orange_path_dst = [f"R{n.robot_id}" for n in best_path_from_demand if isinstance(n, Robot)]
    print(" ➔ ".join(orange_path_dst))

    """

    # Draw initial scene BEFORE repulsion
    screen.fill((255, 255, 255))
    source.draw(screen)
    #print a list
    for d in demands:
        d.draw(screen)
    """
    # Draw obstacles and connection range
    for obstacle in obstacles:
        obstacle.draw(screen)
        pygame.draw.circle(screen, (255, 200, 200), (int(obstacle.x), int(obstacle.y)), 60, 1)
    """
    # Draw all connections
    for a, b in connections:
        pygame.draw.line(screen, (210, 210, 210), (a.x, a.y), (b.x, b.y), 1)

    for path in network_paths:
        for i in range(len(path) - 1):
            pygame.draw.line(screen, (255, 0, 0), (path[i].x, path[i].y),
                            (path[i + 1].x, path[i + 1].y), 3)


    # Draw robots
    robots_in_path = {n for p in network_paths for n in p if isinstance(n, Robot)}
    for robot in robots:
        """
        if is_in_obstacle_range(robot, obstacles, 60):

            robot.draw(screen, color=(180, 80, 0))  # naranja para peligro
        """   
        if robot in robots_in_path:
            robot.draw(screen, color=(0, 200, 0))  # green
        else:
            robot.draw(screen, color=(0, 100, 255))  # blue
        

    pygame.display.flip()

    # WAIT FOR KEY PRESS BEFORE CONTINUING
    waiting = True
    print("\nPress any key to continue and apply REPULSION...\n")
    while waiting:
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                waiting = False
            elif event.type == pygame.QUIT:
                pygame.quit()
                return
    running = True
    while running:
        screen.fill((255, 255, 255))
        source.draw(screen)
        for d in demands:
            d.draw(screen)
        
        #obstacles 
        # Repulsion force (only visualization for now)
        """
        for obstacle in obstacles:
            obstacle.draw(screen)
            pygame.draw.circle(screen, (255, 200, 200), (int(obstacle.x), int(obstacle.y)), 60, 1)

            apply_virtual_forces(robots, obstacles, best_path_from_source, CONNECTION_DISTANCE, fixed_connections)
            #enforce_connection_constraints(robots, fixed_connections, CONNECTION_DISTANCE)
            
            # Apply repulsion
            for robot in robots:
                for obstacle in obstacles:
                    dx = robot.x - obstacle.x
                    dy = robot.y - obstacle.y
                    dist = math.hypot(dx, dy)
                    if dist < 60:
                        
                        
                        repel_strength = 1* (60 - dist) 
                        # Store original position
                        original_x, original_y = robot.x, robot.y

                        # Tentatively move
                        robot.x += dx * repel_strength
                        robot.y += dy * repel_strength

                        # Check if all existing connections are still valid
                        still_connected = all(
                            distance(robot, neighbor) <= CONNECTION_DISTANCE
                            for (r1, r2) in connections
                            if robot in (r1, r2)
                            for neighbor in [r2 if r1 == robot else r1 if r2 == robot else None]
                            if neighbor is not None
                        )

                        # If a connection would break, revert movement
                        if not still_connected:
                            robot.x, robot.y = original_x, original_y
        """

        # Recalculate connections based on new positions
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

        # Update hops and paths after movement
        propagate_local_hop_count(source, robots, connections, 'hop_from_source', 'parent_from_source')
        compute_multi_demand_hops(demands, robots, connections)
        """
        best_path_from_source = build_path_after_repulsion(source, demands, robots, connections, 'hop_from_source')
        best_path_from_demand = build_path_after_repulsion(demands, source, robots, connections, 'hop_from_demand')
        """
        # Red (múltiples caminos, uno por demand)
        network_paths = build_shared_trunk_network(demands, source, robots, connections)

        # (opcional) set de aristas si quieres usarlas luego
        network_edges = set()
        for path in network_paths:
            for a, b in zip(path, path[1:]):
                network_edges.add((a, b))
                network_edges.add((b, a))
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
    
        
        for a, b in connections:
            pygame.draw.line(screen, (210, 210, 210), (a.x, a.y), (b.x, b.y), 1)

        for path in network_paths:
            for i in range(len(path) - 1):
                pygame.draw.line(screen, (255, 0, 0), (path[i].x, path[i].y),
                                (path[i + 1].x, path[i + 1].y), 3)

        # Draw fixed optimal connections (in orange)
        """
        for (r1, r2) in fixed_connections:
            pygame.draw.line(screen, (255, 165, 0), (r1.x, r1.y), (r2.x, r2.y), 2)
        """

        # Rutas óptimas (todas) en rojo
        for path in network_paths:
            for i in range(len(path) - 1):
                pygame.draw.line(screen, (255, 0, 0), (path[i].x, path[i].y),
                                (path[i + 1].x, path[i + 1].y), 3)

        # Create a set of robots in the optimal path for quick lookup
        robots_in_path = {n for p in network_paths for n in p if isinstance(n, Robot)}

        

        
        # Range visually detected
        """
        
        for robot in robots:
            pygame.draw.circle(screen, (180, 180, 180), (int(robot.x), int(robot.y)), CONNECTION_DISTANCE, 1)
        
        """
        for robot in robots:
            #if is_in_obstacle_range(robot, obstacles, 60):
                #robot.draw(screen, color=(180, 80, 0))  # naranja para peligro
            if robot in robots_in_path:
                robot.draw(screen, color=(0, 200, 0))  # verde
            else:
                robot.draw(screen, color=(0, 100, 255))  # azul


        pygame.display.flip()
    print("\n--- FINAL DEBUGGING: Robots in range ---")
    for robot in robots:
        in_range = [o.robot_id for o in robots
                    if o != robot and distance(robot, o) <= CONNECTION_DISTANCE]
        if in_range:
            print(f" Robot {robot.robot_id} can directly connect to robots: {in_range}")
        else:
            print(f" Robot {robot.robot_id} is isolated — no robots in range")

    print("\n--- FINAL DEBUGGING: Source direct connections ---")
    direct_from_source = [r.robot_id for r in robots
                          if distance(source, r) <= CONNECTION_DISTANCE]
    print(f"Source can directly connect to robots after repulsion: {direct_from_source}")

    print("\n--- FINAL DEBUGGING: Demand direct connections ---")
    direct_from_demand = [r.robot_id for r in robots
                          if distance(demands, r) <= CONNECTION_DISTANCE]
    print(f"Demand can directly connect to robots after repulsion: {direct_from_demand}")

    print("\n--- UPDATED FINAL STATE AFTER REPULSION ---")
    for r in robots:
        total_hops = (r.hop_from_source + r.hop_from_demand
                      if r.hop_from_source is not None and r.hop_from_demand is not None
                      else None)
        print(f"Robot {r.robot_id} | SourceHop: {r.hop_from_source} | DemandHop: {r.hop_from_demand} | TotalHop: {total_hops}")

    """
    print("\n>>> Final Best Path Source ➔ Demand (After Repulsion):")
    print([get_node_name(r) for r in best_path_from_source])
    print("\n>>> Final Best Path Demand ➔ Source (After Repulsion):")
   # print([get_node_name(r) for r in best_path_from_demand])
    """
    pygame.quit()
    



if __name__ == "__main__":
    main()