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
CONNECTION_DISTANCE = 120
RESTORING_FORCE_THRESHOLD = 100  # Start applying strong forces when closer than this to max distance
REPULSION_STRENGTH = 2
RESTORING_STRENGTH = 3.0  # Stronger than repulsion to maintain connections

# Create obstacles
obstacles = []
for _ in range(3):
    x = random.randint(150, ARENA_WIDTH - 150)
    y = random.randint(100, ARENA_HEIGHT - 100)
    obstacles.append(Obstacle(x, y))

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
    """Propagate hop counts from a source node to all connected robots"""
    for robot in robots:
        setattr(robot, attr_hop, None)
        setattr(robot, attr_parent, None)

    queue = []
    visited = set()

    # Start from direct connections to source
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

        visited.add(current)

        # Find neighbors
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
    """Check if a path exists between source and demand"""
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
    """Verify if there is a direct physical connection between two nodes"""
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
    """Get display name for a node"""
    return n.name if isinstance(n, Node) else f"Robot {n.robot_id}"

def build_optimal_path(start, end, robots, connections, hop_attr):
    """Build optimal path using hop count heuristic"""
    path = []
    visited = set()
    current = start

    current_hop = 0 if not isinstance(start, Robot) else getattr(start, hop_attr)

    while current != end:
        visited.add(current)
        
        # If we can directly connect to the end, do so
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
                    source_hop = getattr(r, 'hop_from_source')
                    demand_hop = getattr(r, 'hop_from_demand')

                    if source_hop is None:
                        source_hop = float('inf')
                    if demand_hop is None:
                        demand_hop = float('inf')

                    total_hop = source_hop + demand_hop
                    candidates.append((total_hop, r))

        if not candidates:
            break

        _, best_next = min(candidates, key=lambda x: x[0])
        path.append(best_next)
        visited.add(best_next)
        current = best_next
        current_hop += 1

    # Connect to the beginning if needed
    if isinstance(start, Node):
        first = path[0] if path else None
        if first and distance(start, first) <= CONNECTION_DISTANCE:
            path.insert(0, start)
    
    return path

def apply_virtual_forces(robots, connections, obstacles):
    """Apply virtual forces for obstacle avoidance and connection preservation"""
    forces = {robot.robot_id: [0, 0] for robot in robots}
    
    # Obstacle repulsion forces
    for robot in robots:
        for obstacle in obstacles:
            dx = robot.x - obstacle.x
            dy = robot.y - obstacle.y
            dist = math.hypot(dx, dy)
            
            if dist < 100 and dist > 0:  # Avoid division by zero
                # Normalize direction
                dx /= dist
                dy /= dist
                # Apply repulsion force
                repel_force = REPULSION_STRENGTH * (100 - dist) / 100
                forces[robot.robot_id][0] += dx * repel_force
                forces[robot.robot_id][1] += dy * repel_force
    
    # Connection preservation forces
    for a, b in connections:
        if isinstance(a, Robot) and isinstance(b, Robot):
            dx = b.x - a.x
            dy = b.y - a.y
            dist = math.hypot(dx, dy)
            
            # Apply strong restoring force when approaching connection limit
            if dist > RESTORING_FORCE_THRESHOLD and dist > 0:
                # Force to keep robots within connection range
                dx /= dist
                dy /= dist
                
                # Stronger force as they get closer to breaking point
                force_magnitude = RESTORING_STRENGTH * (dist - RESTORING_FORCE_THRESHOLD) / (CONNECTION_DISTANCE - RESTORING_FORCE_THRESHOLD)
                
                # Apply force to both robots (toward each other)
                forces[a.robot_id][0] += dx * force_magnitude
                forces[a.robot_id][1] += dy * force_magnitude
                forces[b.robot_id][0] -= dx * force_magnitude
                forces[b.robot_id][1] -= dy * force_magnitude
    
    return forces

def apply_movement_with_constraint_checking(robots, forces, connections):
    """Apply movement while ensuring no connections are broken"""
    for robot in robots:
        proposed_x = robot.x + forces[robot.robot_id][0]
        proposed_y = robot.y + forces[robot.robot_id][1]
        
        # Check if this movement would break any connections
        movement_allowed = True
        
        for a, b in connections:
            if a == robot and isinstance(b, Robot):
                new_dist = math.hypot(proposed_x - b.x, proposed_y - b.y)
                if new_dist > CONNECTION_DISTANCE:
                    movement_allowed = False
                    break
            elif b == robot and isinstance(a, Robot):
                new_dist = math.hypot(proposed_x - a.x, proposed_y - a.y)
                if new_dist > CONNECTION_DISTANCE:
                    movement_allowed = False
                    break
        
        # Only apply movement if it doesn't break connections
        if movement_allowed:
            robot.x = max(ROBOT_RADIUS, min(ARENA_WIDTH - ROBOT_RADIUS, proposed_x))
            robot.y = max(ROBOT_RADIUS, min(ARENA_HEIGHT - ROBOT_RADIUS, proposed_y))

def update_network_state(source, demand, robots):
    """Update all network connections and hop counts"""
    connections = []
    
    # Robot-to-robot connections
    for i in range(len(robots)):
        for j in range(i + 1, len(robots)):
            if distance(robots[i], robots[j]) <= CONNECTION_DISTANCE:
                connect(robots[i], robots[j], connections)

    # Source and demand connections
    for robot in robots:
        if distance(robot, source) <= CONNECTION_DISTANCE:
            connect(robot, source, connections)
        if distance(robot, demand) <= CONNECTION_DISTANCE:
            connect(robot, demand, connections)

    # Propagate hop counts
    propagate_local_hop_count(source, robots, connections, 'hop_from_source', 'parent_from_source')
    propagate_local_hop_count(demand, robots, connections, 'hop_from_demand', 'parent_from_demand')
    
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

    return connections

def main():
    pygame.init()
    screen = pygame.display.set_mode((ARENA_WIDTH, ARENA_HEIGHT))
    pygame.display.set_caption("Dynamic Optimal Path with Connection Preservation")
    clock = pygame.time.Clock()

    source = Node("Source", 50, ARENA_HEIGHT // 2, (255, 0, 0))
    demand = Node("Demand", ARENA_WIDTH - 50, ARENA_HEIGHT // 2, (0, 128, 0))

    # Initialize robots with guaranteed connectivity
    connected = False
    attempts = 0
    while not connected:
        attempts += 1
        robots = []
        
        for i in range(N_ROBOTS):
            x = random.randint(100, ARENA_WIDTH - 80)
            y = random.randint(100, ARENA_HEIGHT - 80)
            robots.append(Robot(i, x, y, ROBOT_RADIUS))
        
        connections = update_network_state(source, demand, robots)
        connected = is_path_exists(source, demand, robots, connections)

    print(f"Connected after {attempts} attempts.")
    
    # Main simulation loop
    running = True
    frame_count = 0
    
    while running:
        frame_count += 1
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # CONTINUOUS UPDATES (every frame)
        
        # 1. Apply virtual forces
        forces = apply_virtual_forces(robots, connections, obstacles)
        
        # 2. Apply movement with constraint checking
        apply_movement_with_constraint_checking(robots, forces, connections)
        
        # 3. Update network state (connections and hop counts)
        connections = update_network_state(source, demand, robots)
        
        # 4. Compute optimal paths (CONTINUOUSLY UPDATED)
        best_path_from_source = build_optimal_path(source, demand, robots, connections, 'hop_from_source')
        best_path_from_demand = build_optimal_path(demand, source, robots, connections, 'hop_from_demand')
        
        # Debug output every 60 frames (1 second at 60 FPS)
        if frame_count % 60 == 0:
            print(f"\n--- Frame {frame_count} Debug ---")
            print(f"Active connections: {len(connections)}")
            print(f"Path Source->Demand: {[get_node_name(r) for r in best_path_from_source]}")
            print(f"Path Demand->Source: {[get_node_name(r) for r in best_path_from_demand]}")
        
        # RENDERING
        screen.fill((255, 255, 255))
        
        # Draw obstacles
        for obstacle in obstacles:
            obstacle.draw(screen)
            pygame.draw.circle(screen, (255, 200, 200), (int(obstacle.x), int(obstacle.y)), CONNECTION_DISTANCE, 1)

        
        # Draw source and demand
        source.draw(screen)
        demand.draw(screen)
        
        # Draw all connections (light gray)
        for a, b in connections:
            pygame.draw.line(screen, (210, 210, 210), (a.x, a.y), (b.x, b.y), 1)
        
        # Draw optimal paths (thick colored lines)
        for i in range(len(best_path_from_source) - 1):
            pygame.draw.line(screen, (255, 0, 0), 
                           (best_path_from_source[i].x, best_path_from_source[i].y),
                           (best_path_from_source[i + 1].x, best_path_from_source[i + 1].y), 3)
        
        for i in range(len(best_path_from_demand) - 1):
            pygame.draw.line(screen, (0, 100, 255), 
                           (best_path_from_demand[i].x, best_path_from_demand[i].y),
                           (best_path_from_demand[i + 1].x, best_path_from_demand[i + 1].y), 3)
        
        # Draw robots (highlight those in optimal path)
        robots_in_path = {r for r in best_path_from_source if isinstance(r, Robot)}
        
        for robot in robots:
            if robot in robots_in_path:
                robot.draw(screen, color=(0, 200, 0))  # Green for robots in optimal path
            else:
                robot.draw(screen, color=(0, 100, 255))  # Blue for other robots
        
        pygame.display.flip()
        clock.tick(60)  # 60 FPS

    pygame.quit()

if __name__ == "__main__":
    main()