import pygame
import random
import math
from class_robot import Robot

ARENA_WIDTH, ARENA_HEIGHT = 900, 400
ROBOT_RADIUS = 10
N_ROBOTS = 5
CONNECTION_DISTANCE = 80

# Laberinto con camino libre de Source a Demand
MAZE = [
    [1, 0, 1, 1, 0, 1, 1, 0, 1, 1],
    [0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
    [1, 1, 1, 0, 1, 0, 1, 0, 1, 1],
    [0, 0, 0, 0, 1, 0, 0, 0, 0, 1],
]

CELL_WIDTH = ARENA_WIDTH // len(MAZE[0])
CELL_HEIGHT = ARENA_HEIGHT // len(MAZE)

def distance(a, b):
    return math.hypot(a.x - b.x, a.y - b.y)

def try_connect(a, b, connections, walls):
    if (a, b) not in connections and (b, a) not in connections:
        if not is_blocked(a, b, walls):
            connections.append((a, b))
            return True
        else:
            print(f"Blocked connection: {getattr(a, 'robot_id', getattr(a, 'name', 'Unknown'))} -X- {getattr(b, 'robot_id', getattr(b, 'name', 'Unknown'))}")
    return False

def is_blocked(a, b, obstacles):
    for ox, oy in obstacles:
        dist = abs((b.y - a.y) * ox - (b.x - a.x) * oy + b.x * a.y - b.y * a.x) / distance(a, b)
        proj_x = ((ox - a.x) * (b.x - a.x) + (oy - a.y) * (b.y - a.y)) / distance(a, b)**2
        if 0 <= proj_x <= 1:
            if dist < 15:
                return True
    return False

def propagate_hop_count(source_node, robots, connections):
    visited = set()
    queue = [(source_node, 0)]

    for robot in robots:
        robot.hop_count = None
        robot.connected_to_source = False

    while queue:
        current, hops = queue.pop(0)
        if isinstance(current, Robot):
            if current.hop_count is not None:
                continue
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

def print_connection_tree(source, robots, connections):
    graph = {}
    for a, b in connections:
        graph.setdefault(a, []).append(b)
        graph.setdefault(b, []).append(a)

    def get_name(node):
        return f"Robot {node.robot_id}" if isinstance(node, Robot) else node.name

    visited = set()

    def hop_tree_dfs(node, level):
        indent = "    " * level
        print(f"{indent}{get_name(node)}")
        visited.add(node)

        neighbors = []
        for neighbor in graph.get(node, []):
            if neighbor in visited:
                continue
            if isinstance(node, Node) and node.name == "Source":
                if isinstance(neighbor, Robot) and neighbor.hop_count == 1:
                    neighbors.append(neighbor)
            elif isinstance(node, Robot):
                if isinstance(neighbor, Robot) and neighbor.hop_count == node.hop_count + 1:
                    neighbors.append(neighbor)
                elif isinstance(neighbor, Node) and neighbor.name == "Demand":
                    neighbors.append(neighbor)

        neighbors = sorted(neighbors, key=get_name)
        for neighbor in neighbors:
            hop_tree_dfs(neighbor, level + 1)

    print("\n--- Connection Tree (based on hop count structure) ---")
    hop_tree_dfs(source, 0)
    print("------------------------------------------------------\n")

def main():
    global walls
    walls = []
    print("Simulation started")

    pygame.init()
    screen = pygame.display.set_mode((ARENA_WIDTH, ARENA_HEIGHT))
    pygame.display.set_caption("Robot Arena with Laberinto")

    robots_list = []
    for robot_id in range(N_ROBOTS):
        while True:
            x = random.randint(5 * ROBOT_RADIUS, ARENA_WIDTH - 5 * ROBOT_RADIUS)
            y = random.randint(5 * ROBOT_RADIUS, ARENA_HEIGHT - 5 * ROBOT_RADIUS)
            col = x // CELL_WIDTH
            row = y // CELL_HEIGHT
            if MAZE[row][col] == 0:
                robot = Robot(robot_id, x, y, ROBOT_RADIUS)
                robots_list.append(robot)
                break

    global demand
    source = Node("Source", CELL_WIDTH // 2, CELL_HEIGHT * 1 + CELL_HEIGHT // 2, (255, 0, 0))
    demand = Node("Demand", ARENA_WIDTH - CELL_WIDTH // 2, CELL_HEIGHT * 1 + CELL_HEIGHT // 2, (0, 128, 0))

    obstacles = []
    for _ in range(10):
        ox = random.randint(100, ARENA_WIDTH - 100)
        oy = random.randint(50, ARENA_HEIGHT - 50)
        obstacles.append((ox, oy))

    connections = []
    clock = pygame.time.Clock()
    simulation_complete = False
    source_already_connected = False
    demand_already_connected = False
    running = True

    while running:
        screen.fill((255, 255, 255))
        for ox, oy in obstacles:
            pygame.draw.circle(screen, (128, 0, 128), (ox, oy), 12)

        source.draw(screen)
        demand.draw(screen)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        for robot in robots_list:
            for ox, oy in obstacles:
                dx = robot.x - ox
                dy = robot.y - oy
                dist = math.hypot(dx, dy)
                if dist < 20:
                    angle = math.atan2(dy, dx)
                    robot.x += math.cos(angle) * 12  # Bounce stronger
                    robot.y += math.sin(angle) * 12
            robot.update()

        for i in range(len(robots_list)):
            for j in range(i + 1, len(robots_list)):
                if distance(robots_list[i], robots_list[j]) < CONNECTION_DISTANCE:
                    if not is_blocked(robots_list[i], robots_list[j], obstacles):
                        try_connect(robots_list[i], robots_list[j], connections, walls)

        if not source_already_connected:
            for robot in robots_list:
                if distance(robot, source) < CONNECTION_DISTANCE and not is_blocked(robot, source, obstacles):
                    try_connect(robot, source, connections, walls)
                    source_already_connected = True
                    break

        if not demand_already_connected:
            for robot in robots_list:
                if distance(robot, demand) < CONNECTION_DISTANCE and not is_blocked(robot, demand, obstacles):
                    try_connect(robot, demand, connections, walls)
                    demand_already_connected = True
                    break

        propagate_hop_count(source, robots_list, connections)

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

        for a, b in connections:
            pygame.draw.line(screen, (0, 0, 0), (a.x, a.y), (b.x, b.y), 2)

        for robot in robots_list:
            color = (0, 200, 0) if robot.connected_to_source else (0, 0, 255)
            robot.draw(screen, color=color)

        pygame.display.flip()
        clock.tick(60)

    pygame.quit()

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

if __name__ == "__main__":
    main()
