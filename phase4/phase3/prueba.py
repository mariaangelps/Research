import pygame
import random
import math
from class_robot import Robot

ARENA_WIDTH, ARENA_HEIGHT = 900, 400
ROBOT_RADIUS = 10
N_ROBOTS = 15
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

def propagate_hop_count(source, robots, connections):
    visited = set()
    queue = [(source, 0, None)]

    for robot in robots:
        robot.hop_count = None
        robot.parent = None

    while queue:
        current, hops, parent = queue.pop(0)

        if isinstance(current, Robot):
            if current.hop_count is not None:
                continue
            current.hop_count = hops
            current.parent = parent

        visited.add(current)

        for a, b in connections:
            neighbor = b if a == current else a if b == current else None
            if neighbor and neighbor not in visited:
                queue.append((neighbor, hops + 1, current))

def trace_path_to_sink(robots, sink, connections):
    for robot in robots:
        if any((a == robot and b == sink) or (b == robot and a == sink) for a, b in connections):
            if robot.hop_count is not None:
                return robot
    return None

def get_path(robot):
    path = []
    while isinstance(robot, Robot):  # Solo sigue mientras sea un Robot
        path.append(robot)
        robot = robot.parent
    return list(reversed(path))

    return list(reversed(path))

def is_path_from_source_to_sink(source, sink, robots, connections):
    visited = set()
    queue = [source]

    while queue:
        current = queue.pop(0)
        if current == sink:
            return True
        visited.add(current)

        for a, b in connections:
            neighbor = b if a == current else a if b == current else None
            if neighbor and neighbor not in visited:
                queue.append(neighbor)
    return False

def main():
    pygame.init()
    screen = pygame.display.set_mode((ARENA_WIDTH, ARENA_HEIGHT))
    pygame.display.set_caption("Hop Count with Guaranteed Source ➜ Sink Connection")

    source = Node("Source", 50, ARENA_HEIGHT // 2, (255, 0, 0))
    sink = Node("Sink", ARENA_WIDTH - 50, ARENA_HEIGHT // 2, (0, 128, 0))

    # Reintenta hasta lograr conexión Source -> Sink
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
            if distance(robot, sink) <= CONNECTION_DISTANCE:
                connect(robot, sink, connections)

        connected = is_path_from_source_to_sink(source, sink, robots, connections)

    print(f"✅ Red generada en intento #{attempts}, hay camino de Source ➜ Sink.")

    # Propagar hops
    propagate_hop_count(source, robots, connections)

    # Mostrar hop count
    print("\n--- DEBUGGING MESSAGES ---")
    for robot in robots:
        print(f"[Robot {robot.robot_id}] hop_count: {robot.hop_count}")
    print("--------------------------")

    # Ruta hasta el Sink
    last_robot = trace_path_to_sink(robots, sink, connections)
    if last_robot:
        path = get_path(last_robot)
        print("\n>>> Optimal Path:")
        for r in path:
            print(f"Robot {r.robot_id} (hop: {r.hop_count})")
    else:
        print("❌ No se encontró camino al Sink (esto no debería pasar).")

    # Visual loop
    running = True
    while running:
        screen.fill((255, 255, 255))
        source.draw(screen)
        sink.draw(screen)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # Conexiones
        for a, b in connections:
            pygame.draw.line(screen, (180, 180, 180), (a.x, a.y), (b.x, b.y), 1)

        # Ruta óptima
        if last_robot:
            path = get_path(last_robot)
            for i in range(len(path) - 1):
                pygame.draw.line(screen, (255, 0, 255),
                                 (path[i].x, path[i].y),
                                 (path[i+1].x, path[i+1].y), 3)

        # Robots
        for robot in robots:
            color = (255, 0, 255) if last_robot and robot in get_path(last_robot) else (0, 200, 0)
            robot.draw(screen, color=color)

        pygame.display.flip()

    pygame.quit()

if __name__ == "__main__":
    main()
