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

def propagate_hop_count(start_node, robots, connections, attr_hop, attr_parent):
    visited = set()
    queue = [(start_node, 0, None)]

    for robot in robots:
        setattr(robot, attr_hop, None)
        setattr(robot, attr_parent, None)

    while queue:
        current, hops, parent = queue.pop(0)

        if isinstance(current, Robot):
            if getattr(current, attr_hop) is not None:
                continue
            setattr(current, attr_hop, hops)
            setattr(current, attr_parent, parent if isinstance(parent, Robot) else None)

        visited.add(current)

        for a, b in connections:
            neighbor = b if a == current else a if b == current else None
            if neighbor and neighbor not in visited:
                queue.append((neighbor, hops + 1, current))

def is_path_exists(source, sink, robots, connections):
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

def trace_path_to(target_node, robots, connections, hop_attr, parent_attr):
    candidates = []
    for robot in robots:
        if any((a == robot and b == target_node) or (b == robot and a == target_node) for a, b in connections):
            if getattr(robot, hop_attr) is not None:
                # reconstruir el camino
                path = get_path(robot, parent_attr)
                if path and path[0] != robot:  # asegurarse de que tiene padres
                    candidates.append((robot, len(path)))

    # elegir el robot con el camino más corto hacia el target_node
    if candidates:
        best = min(candidates, key=lambda x: x[1])
        return best[0]
    return None


def get_path(robot, parent_attr):
    path = []
    while isinstance(robot, Robot):
        path.append(robot)
        robot = getattr(robot, parent_attr)
    return list(reversed(path))

def main():
    print("nuevo")
    pygame.init()
    screen = pygame.display.set_mode((ARENA_WIDTH, ARENA_HEIGHT))
    pygame.display.set_caption("Hop Count - Source & Sink Paths Compared")

    source = Node("Source", 50, ARENA_HEIGHT // 2, (255, 0, 0))
    sink = Node("Sink", ARENA_WIDTH - 50, ARENA_HEIGHT // 2, (0, 128, 0))

    # Retry until connected
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

        connected = is_path_exists(source, sink, robots, connections)

    print(f"✅ Connected after {attempts} attempts.")

    # Propagate from source and sink
    propagate_hop_count(source, robots, connections, 'hop_from_source', 'parent_from_source')
    propagate_hop_count(sink, robots, connections, 'hop_from_sink', 'parent_from_sink')

    print("\n--- DEBUGGING ---")
    for r in robots:
        print(f"Robot {r.robot_id} | SourceHop: {r.hop_from_source} | SinkHop: {r.hop_from_sink}")
    print("------------------")

    # Get both paths
    last_from_source = trace_path_to(sink, robots, connections, 'hop_from_source', 'parent_from_source')
    last_from_sink = trace_path_to(source, robots, connections, 'hop_from_sink', 'parent_from_sink')

    path_source = get_path(last_from_source, 'parent_from_source') if last_from_source else []
    path_sink = get_path(last_from_sink, 'parent_from_sink') if last_from_sink else []

    # Choose best path
    best_path = []
    direction = ""
    print("\n>>> PATH LENGTHS:")
    print(f"Source ➜ Sink length: {len(path_source) if path_source else '❌ no path'}")
    print(f"Sink ➜ Source length: {len(path_sink) if path_sink else '❌ no path'}")

    if path_source and path_sink:
        if len(path_source) <= len(path_sink):
            best_path = path_source
            direction = "Source ➜ Sink"
        else:
            best_path = path_sink
            direction = "Sink ➜ Source"
    elif path_source:
        best_path = path_source
        direction = "Source ➜ Sink"
    elif path_sink:
        best_path = path_sink
        direction = "Sink ➜ Source"
    else:
        print("❌ No path found.")
        return

    print(f"\n>>> BEST PATH ({direction}):")
    for r in best_path:
        h = r.hop_from_source if direction == "Source ➜ Sink" else r.hop_from_sink
        print(f"Robot {r.robot_id} (hop: {h})")

    # Draw loop
    running = True
    while running:
        screen.fill((255, 255, 255))
        source.draw(screen)
        sink.draw(screen)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        for a, b in connections:
            pygame.draw.line(screen, (210, 210, 210), (a.x, a.y), (b.x, b.y), 1)

        # 🔴 path_source
        if path_source and path_source != best_path:
            for i in range(len(path_source) - 1):
                pygame.draw.line(screen, (255, 0, 0),
                                 (path_source[i].x, path_source[i].y),
                                 (path_source[i+1].x, path_source[i+1].y), 3)

        # 🔵 path_sink
        if path_sink and path_sink != best_path:
            for i in range(len(path_sink) - 1):
                pygame.draw.line(screen, (0, 100, 255),
                                 (path_sink[i].x, path_sink[i].y),
                                 (path_sink[i+1].x, path_sink[i+1].y), 3)

        # 💜 best_path
        if best_path:
            for i in range(len(best_path) - 1):
                pygame.draw.line(screen, (255, 0, 255),
                                 (best_path[i].x, best_path[i].y),
                                 (best_path[i+1].x, best_path[i+1].y), 5)

        for robot in robots:
            if robot in best_path:
                color = (255, 0, 255)
            elif robot in path_source:
                color = (255, 150, 150)
            elif robot in path_sink:
                color = (100, 150, 255)
            else:
                color = (0, 180, 0)
            robot.draw(screen, color=color)

        pygame.display.flip()

    pygame.quit()

if __name__ == "__main__":
    main()
