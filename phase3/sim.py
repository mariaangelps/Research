import pygame
import random
import math
from class_robot import Robot  # Asegúrate de que esta clase tenga set_destination() y update()

# Parámetros
ARENA_WIDTH, ARENA_HEIGHT = 900, 400
ROBOT_RADIUS = 10
N_ROBOTS = 5
CONNECTION_DISTANCE = 80  # Distancia máxima para conectarse

# Nodos estáticos: source y demand
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

def main():
    print("Sim Begin")

    pygame.init()
    screen = pygame.display.set_mode((ARENA_WIDTH, ARENA_HEIGHT))
    pygame.display.set_caption("Robot Arena with Connections")

    # Crear robots
    robots_list = []
    for robot_id in range(N_ROBOTS):
        x = random.randint(5 * ROBOT_RADIUS, ARENA_WIDTH - 5 * ROBOT_RADIUS)
        y = random.randint(5 * ROBOT_RADIUS, ARENA_HEIGHT - 5 * ROBOT_RADIUS)
        robot = Robot(robot_id, x, y, ROBOT_RADIUS)
        robots_list.append(robot)

    # Crear nodos source y demand
    source = Node("Source", 50, ARENA_HEIGHT // 2, (255, 0, 0))
    demand = Node("Demand", ARENA_WIDTH - 50, ARENA_HEIGHT // 2, (0, 128, 0))

    # Destinos
    destinations = [
        (100, 100),
        (200, 200),
        (300, 300),
        (500, 100),
        (600, 300),
    ]

    # Asignar destinos
    def assign_destinations(robots_list, destinations):
        for i, robot in enumerate(robots_list):
            if i < len(destinations):
                robot.set_destination(*destinations[i])
                print(f"Robot {robot.robot_id} → assigned destination: {destinations[i]}")

    assign_destinations(robots_list, destinations)

    connections = []

    running = True
    while running:
        screen.fill((255, 255, 255))

        # Dibujar source y demand
        source.draw(screen)
        demand.draw(screen)

        # Actualizar y dibujar robots
        for robot in robots_list:
            robot.update()
            robot.draw(screen)

        # Verificar y conectar robots entre sí
        for i in range(len(robots_list)):
            for j in range(i + 1, len(robots_list)):
                if distance(robots_list[i], robots_list[j]) < CONNECTION_DISTANCE:
                    connect(robots_list[i], robots_list[j], connections)

        # Conectar robots al source
        for robot in robots_list:
            if distance(robot, source) < CONNECTION_DISTANCE:
                connect(robot, source, connections)

        # Conectar robots al demand
        for robot in robots_list:
            if distance(robot, demand) < CONNECTION_DISTANCE:
                connect(robot, demand, connections)

        # Dibujar conexiones
        for a, b in connections:
            pygame.draw.line(screen, (0, 0, 0), (a.x, a.y), (b.x, b.y), 2)

        pygame.display.flip()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

    pygame.quit()

if __name__ == "__main__":
    main()
