import pygame
import random
import math
from collections import deque
from class_robot import Robot
from class_source_and_demand import Source, Demand
from class_obs import Obstacle

ARENA_WIDTH, ARENA_HEIGHT = 1000, 600
ROBOT_RADIUS = 10
N_ROBOTS = 40
#N_EXTRA_ROBOTS = 10
CONNECTION_DISTANCE = 120
# Example: create 3 random obstacles
obstacles = []
for _ in range(3):
    x = random.randint(150, ARENA_WIDTH - 150)
    y = random.randint(100, ARENA_HEIGHT - 100)
    obstacles.append(Obstacle(x, y))




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

def is_path_exists(source, demand, robots, connections):
    visited = set()
    queue = [source]
    while queue:
        current = queue.pop(0)
        if current == demand:
            return True
        visited.add(current)
        for a, b in connections:
            neighbor = b if a == current else a if b == current else None
            if neighbor and neighbor not in visited:
                queue.append(neighbor)
    return False

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

def apply_virtual_forces(robots, obstacles, best_path, connection_distance, optimal_path_links):

    # --- 1. Repulsion from obstacles ---
    # --- 1. Repulsion from obstacles ---
    for robot in robots:
        for obstacle in obstacles:
            dx = robot.x - obstacle.x
            dy = robot.y - obstacle.y
            dist = math.hypot(dx, dy)
            if dist < 60 :
                dx /= dist
                dy /= dist

                

                repel_strength = 1 * (60 - dist)
                robot.next_x += dx * repel_strength
                robot.next_y += dy * repel_strength



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
                robot.next_x += dx * attraction_strength
                robot.next_y += dy * attraction_strength


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
                    robot.next_x += dx * pull_strength
                    robot.next_y += dy * pull_strength

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
                r1.next_x += dx * restoring_strength * 0.5
                r1.next_y += dy * restoring_strength * 0.5
                r2.next_x -= dx * restoring_strength * 0.5
                r2.next_y -= dy * restoring_strength * 0.5



    
    # --- 5. Restoring force for fixed optimal path connections ---
    for (r1, r2) in optimal_path_links:
        if not isinstance(r1, Robot) or not isinstance(r2, Robot):
            continue

        dx = r2.x - r1.x
        dy = r2.y - r1.y
        dist = math.hypot(dx, dy)

        if dist > 0.9 * connection_distance:  # even if still "connected", keep them close
            dx /= dist
            dy /= dist

            restoring_force = 3.0 * (dist - connection_distance) / connection_distance

            r1.next_x += dx * restoring_force * 0.5
            r1.next_y += dy * restoring_force * 0.5
            r2.next_x -= dx * restoring_force * 0.5
            r2.next_y -= dy * restoring_force * 0.5


# --- ENFORCE CONNECTION CONSTRAINTS ---
# This runs after computing the forces, but before applying movement.
# It ensures robots don't break any existing fixed connection due to motion.
# --- ENFORCE CONNECTION CONSTRAINTS ---
# Enhanced version that reverts all robots if any connection would break

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
        _, best_next = min(candidates, key=lambda x: x[0])

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


obstacles_active = True  # as soon as virtual forces are used

def validate_future_positions(robots, fixed_connections, connection_distance):
    for (r1, r2) in fixed_connections:
        if not isinstance(r1, Robot) or not isinstance(r2, Robot):
            continue

        dx = r2.next_x - r1.next_x
        dy = r2.next_y - r1.next_y
        dist = math.hypot(dx, dy)

        if dist > connection_distance:
            #ía la conexión entre R{r1.robot_id} y R{r2.robot_id}")
            return False

    return True




def main():
    pygame.init()
    screen = pygame.display.set_mode((ARENA_WIDTH, ARENA_HEIGHT))
    pygame.display.set_caption("Hop Count - Source & Demand Optimal Path")

    source = Node("Source", 50, ARENA_HEIGHT // 2, (255, 0, 0))
    demand = Node("Demand", ARENA_WIDTH - 50, ARENA_HEIGHT // 2, (0, 128, 0))

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
                    connect(robots[i], robots[j], connections)


        
        #assures robots are inside the conection range
        for robot in robots:
            if distance(robot, source) <= CONNECTION_DISTANCE:
                connect(robot, source, connections)
            if distance(robot, demand) <= CONNECTION_DISTANCE:
                connect(robot, demand, connections)

        connected = is_path_exists(source, demand, robots, connections)

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
    for robot in robots:
        if distance(demand, robot) <= CONNECTION_DISTANCE:
            direct_from_demand.append(robot.robot_id)
    print(f"Demand can directly connect to robots: {direct_from_demand}")


    propagate_local_hop_count(source, robots, connections, 'hop_from_source', 'parent_from_source')
    propagate_local_hop_count(demand, robots, connections, 'hop_from_demand', 'parent_from_demand')

    print("\n--- DEBUGGING ---")
    for r in robots:
        total_hops = None
        if r.hop_from_source is not None and r.hop_from_demand is not None:
            total_hops = r.hop_from_source + r.hop_from_demand
        print(f"Robot {r.robot_id} | SourceHop: {r.hop_from_source} | DemandHop: {r.hop_from_demand} | TotalHop: {total_hops}")
    print("------------------")

    best_path_from_source = build_optimal_path(source, demand, robots, connections, 'hop_from_source')
    best_path_from_demand = build_optimal_path(demand, source, robots, connections, 'hop_from_demand') 


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


    print("\n>>> Best Path Source ➔ Demand:")
    print([get_node_name(r) for r in best_path_from_source])

    print("\n>>> Best Path Demand ➔ Source :")
    print([get_node_name(r) for r in best_path_from_demand])

    print("\n>>> Robots in ORANGE Path (Source ➔ Demand):")
    orange_path_src = [f"R{n.robot_id}" for n in best_path_from_source if isinstance(n, Robot)]
    print(" ➔ ".join(orange_path_src))

    print("\n>>> Robots in ORANGE Path (Demand ➔ Source):")
    orange_path_dst = [f"R{n.robot_id}" for n in best_path_from_demand if isinstance(n, Robot)]
    print(" ➔ ".join(orange_path_dst))

    debug_printed=False #for repulsion

    running = True
    while running:
        screen.fill((255, 255, 255))
        source.draw(screen)
        demand.draw(screen)

        #obstacles 
        # Repulsion force (only visualization for now)
        for obstacle in obstacles:
            obstacle.draw(screen)
            pygame.draw.circle(screen, (255, 200, 200), (int(obstacle.x), int(obstacle.y)), 60, 1)




        # Recalculate connections based on new positions
        connections = []
        for i in range(len(robots)):
            for j in range(i + 1, len(robots)):
                if distance(robots[i], robots[j]) <= CONNECTION_DISTANCE:
                    connect(robots[i], robots[j], connections)

        for robot in robots:
            if distance(robot, source) <= CONNECTION_DISTANCE:
                connect(robot, source, connections)
            if distance(robot, demand) <= CONNECTION_DISTANCE:
                connect(robot, demand, connections)

        # Update hops and paths after movement
        propagate_local_hop_count(source, robots, connections, 'hop_from_source', 'parent_from_source')
        propagate_local_hop_count(demand, robots, connections, 'hop_from_demand', 'parent_from_demand')

        # Recalculate path if current one is broken
        if not is_best_path_valid(best_path_from_source, connections):
            #print("⚠️ Path broken — recomputing using build_optimal_path().")

            # Recalculate hop counts first (because robot positions may have changed)
            propagate_local_hop_count(source, robots, connections, 'hop_from_source', 'parent_from_source')
            propagate_local_hop_count(demand, robots, connections, 'hop_from_demand', 'parent_from_demand')

            # Rebuild the optimal path dynamically
            best_path_from_source = build_optimal_path(source, demand, robots, connections, 'hop_from_source')
            best_path_from_demand = build_optimal_path(demand, source, robots, connections, 'hop_from_demand')

            # Update fixed connection set
            fixed_connections = set()
            for i in range(len(best_path_from_source) - 1):
                a = best_path_from_source[i]
                b = best_path_from_source[i + 1]
                if isinstance(a, (Robot, Node)) and isinstance(b, (Robot, Node)):
                    if isinstance(a, Robot) and isinstance(b, Robot):
                        pair = (a, b) if a.robot_id < b.robot_id else (b, a)
                    else:
                        pair = (a, b)
                    fixed_connections.add(pair)

            # Ensure the path starts from Source and ends at Demand (if possible)
            if best_path_from_source:
                first = best_path_from_source[0]
                if first != source and distance(source, first) <= CONNECTION_DISTANCE and (source, first) not in connections and (first, source) not in connections:
                    connect(source, first, connections)
                    best_path_from_source.insert(0, source)

                last = best_path_from_source[-1]
                if last != demand and distance(last, demand) <= CONNECTION_DISTANCE and (demand, last) not in connections and (last, demand) not in connections:
                    connect(demand, last, connections)
                    best_path_from_source.append(demand)


           
                # Save previous positions before movement
        for r in robots:
            r.prev_x, r.prev_y = r.x, r.y
            r.next_x, r.next_y = r.x, r.y 

        apply_virtual_forces(robots, obstacles, best_path_from_source, CONNECTION_DISTANCE, fixed_connections)
        apply_virtual_forces(robots, obstacles, best_path_from_demand, CONNECTION_DISTANCE, fixed_connections)

        enforce_connection_constraints(robots, fixed_connections, CONNECTION_DISTANCE)
        
        # Recalcular conexiones después de movimiento
        connections = []
        for i in range(len(robots)):
            for j in range(i + 1, len(robots)):
                if distance(robots[i], robots[j]) <= CONNECTION_DISTANCE:
                    connect(robots[i], robots[j], connections)

        for robot in robots:
            if distance(robot, source) <= CONNECTION_DISTANCE:
                connect(robot, source, connections)
            if distance(robot, demand) <= CONNECTION_DISTANCE:
                connect(robot, demand, connections)

        # Propagar hop counts de nuevo
        propagate_local_hop_count(source, robots, connections, 'hop_from_source', 'parent_from_source')
        propagate_local_hop_count(demand, robots, connections, 'hop_from_demand', 'parent_from_demand')

        # Reconstruir caminos después del movimiento
        best_path_from_source = build_optimal_path(source, demand, robots, connections, 'hop_from_source')
        best_path_from_demand = build_optimal_path(demand, source, robots, connections, 'hop_from_demand')

        # Actualizar conexiones fijas
        fixed_connections = set()
        for i in range(len(best_path_from_source) - 1):
            a = best_path_from_source[i]
            b = best_path_from_source[i + 1]
            if isinstance(a, (Robot, Node)) and isinstance(b, (Robot, Node)):
                if isinstance(a, Robot) and isinstance(b, Robot):
                    pair = (a, b) if a.robot_id < b.robot_id else (b, a)
                else:
                    pair = (a, b)
                fixed_connections.add(pair)

        if validate_future_positions(robots, fixed_connections, CONNECTION_DISTANCE):
            for r in robots:
                r.x, r.y = r.next_x, r.next_y
        else:
            #print("⛔ Movimiento cancelado: se rompería una conexión fija.")
            for r in robots:
                r.next_x, r.next_y = r.x, r.y  # Revertimos propuesta
                propagate_local_hop_count(source, robots, connections, 'hop_from_source', 'parent_from_source')
                propagate_local_hop_count(demand, robots, connections, 'hop_from_demand', 'parent_from_demand')

                best_path_from_source = build_optimal_path(source, demand, robots, connections, 'hop_from_source')
                best_path_from_demand = build_optimal_path(demand, source, robots, connections, 'hop_from_demand')

                fixed_connections = set()
                for i in range(len(best_path_from_source) - 1):
                    a = best_path_from_source[i]
                    b = best_path_from_source[i + 1]
                    if isinstance(a, (Robot, Node)) and isinstance(b, (Robot, Node)):
                        if isinstance(a, Robot) and isinstance(b, Robot):
                            pair = (a, b) if a.robot_id < b.robot_id else (b, a)
                        else:
                            pair = (a, b)
                        fixed_connections.add(pair)




        

        """
        if not debug_printed:
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
            direct_from_source = [r.robot_id for r in robots if distance(source, r) <= CONNECTION_DISTANCE]
            print(f"Source can directly connect to robots after repulsion: {direct_from_source}")

            print("\n--- DEBUGGING: Demand direct connections ---")
            direct_from_demand = [r.robot_id for r in robots if distance(demand, r) <= CONNECTION_DISTANCE]
            print(f"Demand can directly connect to robots after repulsion: {direct_from_demand}")

            print("\n--- UPDATED AFTER REPULSION & COHESION ---")
            for r in robots:
                total_hops = (
                    r.hop_from_source + r.hop_from_demand
                    if r.hop_from_source is not None and r.hop_from_demand is not None
                    else None
                )
                print(f"Robot {r.robot_id} | SourceHop: {r.hop_from_source} | DemandHop: {r.hop_from_demand} | TotalHop: {total_hops}")

            print("\n>>> Best Path Source ➔ Demand (After Repulsion & Cohesion):")
            print([get_node_name(r) for r in best_path_from_source])
            print("\n>>> Best Path Demand ➔ Source (After Repulsion & Cohesion):")
            print([get_node_name(r) for r in best_path_from_demand])

            debug_printed = True

            """



        """
        print("\n Network updated after repulsion:")
        print("Connections re-evaluated based on new positions.")
        print("Hop counts re-propagated from Source and Demand.")
        print("Optimal paths recomputed.")
        print("Current best path Source ➔ Demand:")
        print([get_node_name(r) for r in best_path_from_source])
        print("Current best path Demand ➔ Source:")
        print([get_node_name(r) for r in best_path_from_demand])

        """
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        for a, b in connections:
            pygame.draw.line(screen, (210, 210, 210), (a.x, a.y), (b.x, b.y), 1)

        for i in range(len(best_path_from_source) - 1):
            pygame.draw.line(screen, (255, 0, 0), (best_path_from_source[i].x, best_path_from_source[i].y),
                             (best_path_from_source[i + 1].x, best_path_from_source[i + 1].y), 3)
        
        # Draw fixed optimal connections (in orange)
        for (r1, r2) in fixed_connections:
            pygame.draw.line(screen, (255, 165, 0), (r1.x, r1.y), (r2.x, r2.y), 2)
        

        for i in range(len(best_path_from_demand) - 1):
            pygame.draw.line(screen, (0, 100, 255), (best_path_from_demand[i].x, best_path_from_demand[i].y),
                             (best_path_from_demand[i + 1].x, best_path_from_demand[i + 1].y), 3)

        # Create a set of robots in the optimal path for quick lookup
        robots_in_path = {r for r in best_path_from_source if isinstance(r, Robot)}
        

        
        # Range visually detected
        
        
        for robot in robots:
            pygame.draw.circle(screen, (180, 180, 180), (int(robot.x), int(robot.y)), CONNECTION_DISTANCE, 1)
        
        
        for robot in robots:
            if robot in robots_in_path:
                robot.draw(screen, color=(0, 200, 0))  # green
            else:
                robot.draw(screen, color=(0, 100, 255))  # blue
        

        pygame.display.flip()

    pygame.quit()
    print("\n\n=== FINAL OPTIMAL PATHS AFTER MOVEMENT ===")
    print("Best Path Source ➔ Demand:")
    print([get_node_name(r) for r in best_path_from_source])

    print("\nBest Path Demand ➔ Source:")
    print([get_node_name(r) for r in best_path_from_demand])
    if best_path_from_source[-1] != demand:
        print("\n⚠️ WARNING: Final path from Source does not reach Demand.")

    if best_path_from_demand[-1] != source:
        print("⚠️ WARNING: Final path from Demand does not reach Source.")



if __name__ == "__main__":
    main()