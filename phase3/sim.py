import pygame
import random
import math
from class_robot import Robot  # Asegúrate que Robot tenga .set_destination() y .update()

# Parámetros globales
ARENA_WIDTH, ARENA_HEIGHT = 900, 400
ROBOT_RADIUS = 10
N_ROBOTS = 5
CONNECTION_DISTANCE = 80

# Nodos fijos
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

# Utilidades
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

    while queue:
        current, hops = queue.pop(0)

        if isinstance(current, Robot):
            current.hop_count = hops

        visited.add(current)

        for a, b in connections:
            neighbor = None
            if a == current and b not in visited:
                neighbor = b
            elif b == current and a not in visited:
                neighbor = a

            if neighbor and neighbor not in visited:
                queue.append((neighbor, hops + 1))

def main():
    print("Sim Begin")

    pygame.init()
    screen = pygame.display.set_mode((ARENA_WIDTH, ARENA_HEIGHT))
    pygame.display.set_caption("Robot Arena with Hop Count")

    robots_list = []
    for robot_id in range(N_ROBOTS):
        x = random.randint(5 * ROBOT_RADIUS, ARENA_WIDTH - 5 * ROBOT_RADIUS)
        y = random.randint(5 * ROBOT_RADIUS, ARENA_HEIGHT - 5 * ROBOT_RADIUS)
        robot = Robot(robot_id, x, y, ROBOT_RADIUS)
        robots_list.append(robot)

    # Nodos fijos
    source = Node("Source", 50, ARENA_HEIGHT // 2, (255, 0, 0))
    demand = Node("Demand", ARENA_WIDTH - 50, ARENA_HEIGHT // 2, (0, 128, 0))

    # Destinos iniciales
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
                print(f"Robot {robot.robot_id} → assigned destination: {destinations[i]}")

    assign_destinations(robots_list, destinations)

    connections = []
    clock = pygame.time.Clock()
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

        # Conexiones
        for i in range(len(robots_list)):
            for j in range(i + 1, len(robots_list)):
                if distance(robots_list[i], robots_list[j]) < CONNECTION_DISTANCE:
                    connect(robots_list[i], robots_list[j], connections)

        for robot in robots_list:
            if distance(robot, source) < CONNECTION_DISTANCE:
                connect(robot, source, connections)
            if distance(robot, demand) < CONNECTION_DISTANCE:
                connect(robot, demand, connections)

        # Propagación de hop count
        propagate_hop_count(source, robots_list, connections)

        # Dibujar conexiones
        for a, b in connections:
            pygame.draw.line(screen, (0, 0, 0), (a.x, a.y), (b.x, b.y), 2)

        # Dibujar robots
        for robot in robots_list:
            # Color según hop count
            if robot.hop_count is not None:
                green = min(255, 50 + robot.hop_count * 30)
                color = (0, green, 255)
            else:
                color = (0, 0, 255)
            robot.draw(screen, color=color)

            # Mostrar hop count
            font = pygame.font.Font(None, 20)
            hc = str(robot.hop_count) if robot.hop_count is not None else "-"
            text = font.render(hc, True, (0, 0, 0))
            screen.blit(text, (robot.x - 5, robot.y - 25))

        # Mostrar hop count si demand está conectado
        for a, b in connections:
            if a == demand and isinstance(b, Robot) and b.hop_count is not None:
                font = pygame.font.Font(None, 24)
                text = font.render(f"Demand hop count: {b.hop_count + 1}", True, (0, 0, 0))
                screen.blit(text, (ARENA_WIDTH // 2 - 80, 10))

        pygame.display.flip()
        clock.tick(60)

    pygame.quit()

if __name__ == "__main__":
    main()
