import pygame
import random
import math
from class_robot import Robot

# Arena and simulation parameters
ARENA_WIDTH, ARENA_HEIGHT = 900, 400
ROBOT_RADIUS = 10
N_ROBOTS = 5
CONNECTION_DISTANCE = 80

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

def propagate_hop_count(source_node, robots, connections):
    visited = set()
    queue = [(source_node, 0)]

    for robot in robots:
        robot.hop_count = None
        robot.connected_to_source = False

    while queue:
        current, hops = queue.pop(0)

        # If it's a robot and hasn't been visited with a shorter path
        if isinstance(current, Robot):
            if current.hop_count is not None:
                continue  # already visited with shorter path
            current.hop_count = hops
            current.connected_to_source = True

        visited.add(current)

        for a, b in connections:
            neighbor = None
            if a == current and b not in visited:
                neighbor = b
            elif b == current and a not in visited:
                neighbor = a

            if neighbor and neighbor not in visited:
                queue.append((neighbor, hops + 1))


def all_connected_to_source(robots):
    return all(robot.connected_to_source for robot in robots)

def validate_hop_counts(robots, connections):
    for robot in robots:
        if robot.connected_to_source and robot.hop_count is None:
            print(f"[!] Robot {robot.robot_id} marked connected but has no hop count.")
        if not robot.connected_to_source and robot.hop_count is not None:
            print(f"[!] Robot {robot.robot_id} has hop count but is not marked connected.")
        if robot.hop_count is not None and robot.hop_count > 0:
            has_valid_neighbor = False
            for a, b in connections:
                neighbor = b if a == robot else a if b == robot else None
                if neighbor:
                    if isinstance(neighbor, Robot) and neighbor.hop_count == robot.hop_count - 1:
                        has_valid_neighbor = True
                    elif isinstance(neighbor, Node) and neighbor.name == "Source":
                        has_valid_neighbor = True
            if not has_valid_neighbor:
                print(f"[!] Robot {robot.robot_id} has no valid upstream connection (hop continuity error).")
def print_connection_tree(source, robots, connections):
    # Construye un grafo desde las conexiones
    graph = {}
    for a, b in connections:
        graph.setdefault(a, []).append(b)
        graph.setdefault(b, []).append(a)

    def get_name(node):
        return f"Robot {node.robot_id}" if isinstance(node, Robot) else node.name

    visited = set()

    def dfs(node, level):
        indent = "    " * level
        print(f"{indent}{get_name(node)}")
        visited.add(node)

        neighbors = sorted(graph.get(node, []), key=lambda n: get_name(n))
        for neighbor in neighbors:
            if neighbor not in visited:
                dfs(neighbor, level + 1)

    print("\n--- Connection Tree (based on hop structure) ---")
    dfs(source, 0)
    print("------------------------------------------------\n")






def main():
    print("Simulation started")

    pygame.init()
    screen = pygame.display.set_mode((ARENA_WIDTH, ARENA_HEIGHT))
    pygame.display.set_caption("Robot Arena with Hop Count")

    robots_list = []
    for robot_id in range(N_ROBOTS):
        x = random.randint(5 * ROBOT_RADIUS, ARENA_WIDTH - 5 * ROBOT_RADIUS)
        y = random.randint(5 * ROBOT_RADIUS, ARENA_HEIGHT - 5 * ROBOT_RADIUS)
        robot = Robot(robot_id, x, y, ROBOT_RADIUS)
        robots_list.append(robot)

    source = Node("Source", 50, ARENA_HEIGHT // 2, (255, 0, 0))
    demand = Node("Demand", ARENA_WIDTH - 50, ARENA_HEIGHT // 2, (0, 128, 0))

    destinations = [
        (100, 100),
        (200, 200),
        (300, 300),
        (500, 100),
        (600, 300),
    ]

    def assign_destinations(robots_list, destinations):
        for i, robot in enumerate(robots_list):
            if i < len(destinations):
                robot.set_destination(*destinations[i])
                print(f"Robot {robot.robot_id} assigned destination: {destinations[i]}")

    assign_destinations(robots_list, destinations)

    connections = []
    clock = pygame.time.Clock()
    simulation_complete = False
    source_already_connected = False
    demand_already_connected = False
    alignment_in_progress = False
    alignment_targets = []
    running = True

    

    while running:
        screen.fill((255, 255, 255))
        source.draw(screen)
        demand.draw(screen)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        for robot in robots_list:
            robot.update()

        # Connect nearby robots
        for i in range(len(robots_list)):
            for j in range(i + 1, len(robots_list)):
                if distance(robots_list[i], robots_list[j]) < CONNECTION_DISTANCE:
                    connect(robots_list[i], robots_list[j], connections)

        # Connect one robot to Source
        if not source_already_connected:
            for robot in robots_list:
                if distance(robot, source) < CONNECTION_DISTANCE:
                    connect(robot, source, connections)
                    source_already_connected = True
                    print(f"Robot {robot.robot_id} connected to Source")
                    break

        # Connect one robot to Demand
        if not demand_already_connected:
            for robot in robots_list:
                if distance(robot, demand) < CONNECTION_DISTANCE:
                    connect(robot, demand, connections)
                    demand_already_connected = True
                    print(f"Robot {robot.robot_id} connected to Demand")
                    break

        # Hop count calculation
        propagate_hop_count(source, robots_list, connections)

        # Check demand connection
        demand_connected = any(
            (a == demand and isinstance(b, Robot) and b.connected_to_source) or
            (b == demand and isinstance(a, Robot) and a.connected_to_source)
            for a, b in connections
        )

        if all_connected_to_source(robots_list) and demand_connected and not simulation_complete:
            simulation_complete = True
            for robot in robots_list:
                robot.moving = False

            print("All robots connected. Movement stopped.")
            print("--- Hop Counts ---")
            for robot in robots_list:
                print(f"Robot {robot.robot_id} | Hop Count: {robot.hop_count}")
            print("-------------------")

            print("--- Final Connections ---")
            for a, b in connections:
                name_a = a.robot_id if hasattr(a, 'robot_id') else a.name
                name_b = b.robot_id if hasattr(b, 'robot_id') else b.name
                print(f"{name_a} <--> {name_b}")
            print("-------------------------")
            print_connection_tree(source, robots_list, connections)


            # Validate hop count logic
            validate_hop_counts(robots_list, connections)

            

            # Align robots
            alignment_in_progress = True
            alignment_targets = []
            spacing = (demand.x - source.x) / (len(robots_list) + 1)
            y_line = source.y
            for i, robot in enumerate(sorted(robots_list, key=lambda r: r.robot_id)):
                target_x = source.x + (i + 1) * spacing
                alignment_targets.append((robot, target_x, y_line))

        # Move robots to alignment targets
        if alignment_in_progress:
            all_aligned = True
            for robot, target_x, target_y in alignment_targets:
                dx = target_x - robot.x
                dy = target_y - robot.y
                if abs(dx) > 1:
                    robot.x += dx * 0.05
                    all_aligned = False
                if abs(dy) > 1:
                    robot.y += dy * 0.05
                    all_aligned = False
            if all_aligned:
                print("Robots successfully aligned.")
                alignment_in_progress = False

        # Draw connections
        for a, b in connections:
            pygame.draw.line(screen, (0, 0, 0), (a.x, a.y), (b.x, b.y), 2)

        # Draw robots
        for robot in robots_list:
            color = (0, 200, 0) if robot.connected_to_source else (0, 0, 255)
            robot.draw(screen, color=color)

        pygame.display.flip()
        clock.tick(60)

    pygame.quit()

if __name__ == "__main__":
    main()
