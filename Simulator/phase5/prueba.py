import pygame
import random
import math
from collections import deque
from class_robot import Robot
from class_source_and_demand import Source, Demand

ARENA_WIDTH, ARENA_HEIGHT = 1300, 500
ROBOT_RADIUS = 10
N_ROBOTS = 20
N_EXTRA_ROBOTS = 10
CONNECTION_DISTANCE = 120

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
    print(f"\nStarting local token-passing from: {source.name}")
    for robot in robots:
        setattr(robot, attr_hop, None)
        setattr(robot, attr_parent, None)

    queue = [(source, 0, None)]
    visited = set()

    while queue:
        current, hops, parent = queue.pop(0)

        if isinstance(current, Robot):
            current_score = hops + current.robot_id
            existing_hop = getattr(current, attr_hop)
            existing_score = None if existing_hop is None else existing_hop + current.robot_id

            if existing_score is not None and current_score >= existing_score:
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

def existe_ruta_fisica(a, b, conexiones_fisicas):
    visitado = set()
    cola = deque()
    cola.append(a)
    visitado.add(a)

    while cola:
        actual = cola.popleft()
        if actual == b:
            return True

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

def build_optimal_path(start, end, robots, connections, hop_attr):
    path = []
    visited = set()
    current = start

    current_hop = 0 if not isinstance(start, Robot) else getattr(start, hop_attr)

    while current != end:
        visited.add(current)
        candidates = []
        for r in robots:
            if r in visited:
                continue
            if getattr(r, hop_attr) == current_hop + 1:
                if existe_ruta_fisica(current, r, connections):
                    score = getattr(r, hop_attr) + r.robot_id
                    candidates.append((score, r))

        #DEBUGGING
        if not candidates:
            # Don't print the warning if we can still connect to the endpoint directly
            can_connect_to_end = isinstance(end, Node) and existe_ruta_fisica(current, end, connections)
            if not can_connect_to_end:
                print(f" Stopped at hop {current_hop}: no physical path to any robot with hop {current_hop + 1} from {get_node_name(current)}")
            break


        _, best_next = min(candidates, key=lambda x: x[0])

        path.append(best_next)
        visited.add(best_next)
        current = best_next
        current_hop += 1

    #checks if we can connect to the end
    if isinstance(end, Node):
        last = path[-1] if path else None
        if last and existe_ruta_fisica(last, end, connections):
            path.append(end)

    #checks if we can connect to the beginning
    if isinstance(start, Node):
        first = path[0] if path else None
        if first and existe_ruta_fisica(start, first, connections):
            path.insert(0, start)

    return path







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

        for j in range(N_EXTRA_ROBOTS):
            x = random.randint(100, ARENA_WIDTH - 80)
            y = random.randint(100, ARENA_HEIGHT - 80)
            robots.append(Robot(N_ROBOTS + j, x, y, ROBOT_RADIUS))

        for i in range(len(robots)):
            for j in range(i + 1, len(robots)):
                if distance(robots[i], robots[j]) <= CONNECTION_DISTANCE:
                    connect(robots[i], robots[j], connections)


        

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
    
    #debugging
    """
    print("\n🔍 Verificando conexión física del camino Source ➔ Demand:")
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

    running = True
    while running:
        screen.fill((255, 255, 255))
        source.draw(screen)
        demand.draw(screen)

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

        # Create a set of robots in the optimal path for quick lookup
        robots_in_path = {r for r in best_path_from_source if isinstance(r, Robot)}
        
        """
        # Range visually detected
        for robot in robots:
            pygame.draw.circle(screen, (180, 180, 180), (int(robot.x), int(robot.y)), CONNECTION_DISTANCE, 1)
        """

        for robot in robots:
            if robot in robots_in_path:
                robot.draw(screen, color=(0, 200, 0))  # green
            else:
                robot.draw(screen, color=(0, 100, 255))  # blue


        pygame.display.flip()

    pygame.quit()

if __name__ == "__main__":
    main()