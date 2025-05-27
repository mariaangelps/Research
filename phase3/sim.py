import pygame
import random
import math
from class_robot import Robot  # Asegúrate de que Robot tenga set_destination() y update()

# Parámetros
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

def all_connected_to_source(robots):
    return all(robot.hop_count is not None for robot in robots)

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
                print(f"Robot {robot.robot_id} → assigned destination: {destinations[i]}")

    assign_destinations(robots_list, destinations)

    connections = []
    clock = pygame.time.Clock()
    simulation_complete = False
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

        # Propagar hop count
        propagate_hop_count(source, robots_list, connections)

        # Verifica si demand está conectado a un robot con hop_count
        demand_connected = any(
            (a == demand and isinstance(b, Robot) and b.hop_count is not None) or
            (b == demand and isinstance(a, Robot) and a.hop_count is not None)
            for a, b in connections
        )

        # Si todos los robots están conectados y demand también → detener movimiento y mostrar info
        if all_connected_to_source(robots_list) and demand_connected and not simulation_complete:
            for robot in robots_list:
                robot.moving = False
            simulation_complete = True
            print("\n✅ Todos los robots están conectados. Deteniendo movimiento.")
            print("\n--- HOP COUNTS ---")
            for robot in robots_list:
                print(f"Robot {robot.robot_id} | Final destination: {robot.destination} | Hop Count: {robot.hop_count}")
            print("------------------\n")

        # Dibujar conexiones
        for a, b in connections:
            pygame.draw.line(screen, (0, 0, 0), (a.x, a.y), (b.x, b.y), 2)

        # Dibujar robots (sin texto de hop count)
        for robot in robots_list:
            if robot.hop_count is not None:
                green = min(255, 50 + robot.hop_count * 30)
                color = (0, green, 255)
            else:
                color = (0, 0, 255)
            robot.draw(screen, color=color)

        # Mensaje visual si completado
        if simulation_complete:
            font_big = pygame.font.Font(None, 48)
            msg = font_big.render("✅ Conexión completa", True, (0, 128, 0))
            screen.blit(msg, (ARENA_WIDTH // 2 - 160, 20))

        pygame.display.flip()
        clock.tick(60)

    pygame.quit()

if __name__ == "__main__":
    main()
