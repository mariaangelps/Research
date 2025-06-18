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
        if first and distance(start, first) <= CONNECTION_DISTANCE:
            path.insert(0, start)
    
    return path

def build_path_after_repulsion(start, end, robots, connections, hop_attr):
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

        candidates.sort(key=lambda x: (x[0], x[1].robot_id))
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
            pygame.draw.circle(screen, (255, 200, 200), (int(obstacle.x), int(obstacle.y)), CONNECTION_DISTANCE, 1)


            # Apply repulsion
            for robot in robots:
                for obstacle in obstacles:
                    dx = robot.x - obstacle.x
                    dy = robot.y - obstacle.y
                    dist = math.hypot(dx, dy)
                    if dist < 120:
                        if dist != 0:
                            dx /= dist
                            dy /= dist
                        repel_strength = 1.5 * (120 - dist) / 120
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


            # Recalculate connections
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

            # Recalculate hops
            propagate_local_hop_count(source, robots, connections, 'hop_from_source', 'parent_from_source')
            propagate_local_hop_count(demand, robots, connections, 'hop_from_demand', 'parent_from_demand')

            best_path_from_source = build_path_after_repulsion(source, demand, robots, connections, 'hop_from_source')
            best_path_from_demand = build_path_after_repulsion(demand, source, robots, connections, 'hop_from_demand')

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
                direct_from_source = []
                for robot in robots:
                    if distance(source, robot) <= CONNECTION_DISTANCE:
                        direct_from_source.append(robot.robot_id)
                        #gotta fix this print statement 
                print(f"Source can directly connect to robots after repulsion: {direct_from_source}")

                print("\n--- DEBUGGING: Demand direct connections ---")
                direct_from_demand = []
                for robot in robots:
                    if distance(demand, robot) <= CONNECTION_DISTANCE:
                        direct_from_demand.append(robot.robot_id)
                print(f"Demand can directly connect to robots after repulsion: {direct_from_demand}")

                print("\n--- UPDATED AFTER REPULSION ---")
                for r in robots:
                    total_hops = (
                        r.hop_from_source + r.hop_from_demand
                        if r.hop_from_source is not None and r.hop_from_demand is not None
                        else None
                    )
                    print(f"Robot {r.robot_id} | SourceHop: {r.hop_from_source} | DemandHop: {r.hop_from_demand} | TotalHop: {total_hops}")

                print("\n>>> Best Path Source ➔ Demand (After Repulsion):")
                print([get_node_name(r) for r in best_path_from_source])
                print("\n>>> Best Path Demand ➔ Source (After Repulsion):")
                print([get_node_name(r) for r in best_path_from_demand])

                

                debug_printed = True  





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

        for i in range(len(best_path_from_demand) - 1):
            pygame.draw.line(screen, (0, 100, 255), (best_path_from_demand[i].x, best_path_from_demand[i].y),
                             (best_path_from_demand[i + 1].x, best_path_from_demand[i + 1].y), 3)

        # Create a set of robots in the optimal path for quick lookup
        robots_in_path = {r for r in best_path_from_source if isinstance(r, Robot)}
        

        
        # Range visually detected
        """
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