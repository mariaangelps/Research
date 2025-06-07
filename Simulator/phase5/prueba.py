import pygame
import random
import math
from class_robot import Robot
from class_source_and_demand import Source, Demand

ARENA_WIDTH, ARENA_HEIGHT = 900, 400
ROBOT_RADIUS = 10
N_ROBOTS = 11
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

            print(f"\nRobot {current.robot_id} received token | Hop: {hops} | From: {parent.robot_id if isinstance(parent, Robot) else 'Source'}")

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

def build_optimal_path(start, end, robots, connections, hop_attr):
    path = []
    visited = set()
    current = start

    # Obtener hop del nodo inicial (SourceHop o DemandHop)
    if isinstance(start, Robot):
        current_hop = getattr(start, hop_attr)
    else:
        current_hop = 0  # Source o Demand tienen hop 0 al iniciar

    while current != end:
        visited.add(current)
        neighbors = [b if a == current else a for a, b in connections if a == current or b == current]
        candidates = []

        for neighbor in neighbors:
            if isinstance(neighbor, Robot) and neighbor not in visited:
                neighbor_hop = getattr(neighbor, hop_attr)
                if neighbor_hop == current_hop + 1:
                    score = neighbor_hop + neighbor.robot_id
                    candidates.append((score, neighbor))

            elif neighbor == end:
                path.append(end)
                return path

        if not candidates:
            break  # No hay más vecinos con el hop siguiente

        # Elegir el de menor (hop + id)
        _, best_neighbor = min(candidates, key=lambda x: x[0])

        path.append(best_neighbor)
        current = best_neighbor
        current_hop = getattr(current, hop_attr)

    # Si está conectado directamente al final, agrégalo
    if any((a == current and b == end) or (b == current and a == end) for a, b in connections):
        path.append(end)

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
            x = random.randint(60, ARENA_WIDTH - 60)
            y = random.randint(60, ARENA_HEIGHT - 60)
            robots.append(Robot(i, x, y, ROBOT_RADIUS))

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

    propagate_local_hop_count(source, robots, connections, 'hop_from_source', 'parent_from_source')
    propagate_local_hop_count(demand, robots, connections, 'hop_from_demand', 'parent_from_demand')

    print("\n--- DEBUGGING ---")
    for r in robots:
        total_hops = None
        if r.hop_from_source is not None and r.hop_from_demand is not None:
            total_hops = r.hop_from_source + r.hop_from_demand
        print(f"Robot {r.robot_id} | SourceHop: {r.hop_from_source} | DemandHop: {r.hop_from_demand} | TotalHop: {total_hops}")
    print("------------------")

    best_path_from_source = [r for r in build_optimal_path(source, demand, robots, connections, 'hop_from_source') if isinstance(r, Robot)]
    best_path_from_demand = [r for r in build_optimal_path(demand, source, robots, connections, 'hop_from_demand') if isinstance(r, Robot)]

    print("\n>>> MEJOR CAMINO Source ➔ Demand (por hop + ID):")
    print([r.robot_id for r in best_path_from_source])

    print("\n>>> MEJOR CAMINO Demand ➔ Source (por hop + ID):")
    print([r.robot_id for r in best_path_from_demand])

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

        for robot in robots:
            robot.draw(screen, color=(0, 100, 255))

        pygame.display.flip()

    pygame.quit()

if __name__ == "__main__":
    main()
