import pygame
import math
import random

# Configuración inicial
pygame.init()
WIDTH, HEIGHT = 1000, 500
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Robot Chain Connection - Fixed")
clock = pygame.time.Clock()
FPS = 60

# Colores
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
BLUE = (0, 100, 255)
GREEN = (50, 200, 50)
RED = (255, 50, 50)
YELLOW = (255, 255, 0)

class Source:
    def __init__(self, x, y, radius):
        self.x = x
        self.y = y
        self.radius = radius
        self.connected_to = []
        self.connection_progress = {}

    def draw(self, surface):
        pygame.draw.circle(surface, RED, (int(self.x), int(self.y)), self.radius)
        pygame.draw.circle(surface, BLACK, (int(self.x), int(self.y)), self.radius, 2)
        font = pygame.font.SysFont('Arial', 18)
        text = font.render("Source", True, BLACK)
        text_rect = text.get_rect(center=(self.x, self.y))
        surface.blit(text, text_rect)

class Demand:
    def __init__(self, x, y, radius):
        self.x = x
        self.y = y
        self.radius = radius
        self.connected_to = []
        self.connection_progress = {}
        self.connected = False

    def draw(self, surface):
        color = GREEN if self.connected else YELLOW
        pygame.draw.circle(surface, color, (int(self.x), int(self.y)), self.radius)
        pygame.draw.circle(surface, BLACK, (int(self.x), int(self.y)), self.radius, 2)
        font = pygame.font.SysFont('Arial', 18)
        text = font.render("Demand", True, BLACK)
        text_rect = text.get_rect(center=(self.x, self.y))
        surface.blit(text, text_rect)

class Robot:
    def __init__(self, robot_id, x, y, radius):
        self.robot_id = robot_id
        self.x = x
        self.y = y
        self.radius = radius
        self.target_x = x
        self.target_y = y
        self.speed = 2.5
        self.connected_to = []
        self.connection_progress = {}
        self.connected_to_source = False
        self.moving = False
        self.waiting = True if robot_id != 0 else False
        self.arrived = False

    def set_destination(self, x, y):
        self.target_x = x
        self.target_y = y
        self.moving = True
        self.arrived = False

    def update(self):
        if self.moving and not self.arrived:
            dx = self.target_x - self.x
            dy = self.target_y - self.y
            distance = math.hypot(dx, dy)
            
            if distance > 2:
                self.x += dx / distance * self.speed
                self.y += dy / distance * self.speed
            else:
                self.x = self.target_x
                self.y = self.target_y
                self.arrived = True
                self.moving = False

    def draw(self, surface):
        color = GREEN if self.connected_to_source else BLUE
        pygame.draw.circle(surface, color, (int(self.x), int(self.y)), self.radius)
        pygame.draw.circle(surface, BLACK, (int(self.x), int(self.y)), self.radius, 2)
        font = pygame.font.SysFont('Arial', 18)
        text = font.render(str(self.robot_id), True, BLACK)
        text_rect = text.get_rect(center=(self.x, self.y))
        surface.blit(text, text_rect)

def distance(a, b):
    return math.hypot(a.x - b.x, a.y - b.y)

def connect_nodes(a, b):
    if b not in a.connected_to:
        a.connected_to.append(b)
        a.connection_progress[b] = 0.0
    
    if a not in b.connected_to:
        b.connected_to.append(a)
        b.connection_progress[a] = 0.0

def propagate_source_connection(node):
    if not node.connected_to_source:
        node.connected_to_source = True
        for neighbor in node.connected_to:
            if hasattr(neighbor, 'connected_to_source'):
                propagate_source_connection(neighbor)

def main():
    # Crear Source y Demand
    source = Source(100, HEIGHT//2, 25)
    demand = Demand(WIDTH-100, HEIGHT//2, 25)

    # Crear robots (5 en este caso)
    n_robots = 5
    robots = []
    spacing = (demand.x - source.x) / (n_robots + 1)
    
    # Posiciones iniciales con pequeña variación aleatoria
    for i in range(n_robots):
        x = source.x + spacing * (i + 1) + random.randint(-50, 50)
        y = HEIGHT//2 + random.randint(-50, 50)
        robot = Robot(i, x, y, 20)
        robots.append(robot)

    # Configurar destinos iniciales en línea
    for i, robot in enumerate(robots):
        target_x = source.x + spacing * (i + 1)
        target_y = HEIGHT//2
        robot.set_destination(target_x, target_y)

    # El robot 0 comienza activo
    robots[0].waiting = False
    robots[0].moving = True

    running = True
    current_active = 0  # Controla qué robot debe moverse

    while running:
        screen.fill(WHITE)
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # Actualizar posiciones de los robots
        for robot in robots:
            robot.update()

        # Conexión del robot 0 al Source
        if current_active == 0 and source not in robots[0].connected_to:
            connect_nodes(source, robots[0])
            robots[0].moving = False
            propagate_source_connection(robots[0])
            current_active += 1

        # Lógica de conexión secuencial
        for i in range(1, n_robots):
            current_robot = robots[i]
            prev_robot = robots[i-1]
            
            # Activar el robot actual cuando el anterior está listo
            if i == current_active and prev_robot.connected_to_source and prev_robot.arrived:
                current_robot.waiting = False
                current_robot.moving = True
                current_robot.set_destination(prev_robot.x + 60, prev_robot.y)

            # Conectar cuando están cerca
            if (i == current_active and 
                distance(current_robot, prev_robot) < 60 and 
                prev_robot.arrived):
                
                connect_nodes(current_robot, prev_robot)
                current_robot.moving = False
                propagate_source_connection(current_robot)
                
                # Avanzar al siguiente robot solo si la conexión está completa
                if current_robot.connection_progress.get(prev_robot, 0) >= 1.0:
                    current_active += 1

        # Actualizar progreso de conexiones
        for robot in robots:
            for neighbor in robot.connected_to:
                if robot.connection_progress.get(neighbor, 0) < 1.0:
                    robot.connection_progress[neighbor] += 0.02

        # Conexión del último robot al Demand
        if (current_active >= n_robots and 
            distance(robots[-1], demand) < 80 and 
            demand not in robots[-1].connected_to):
            
            connect_nodes(robots[-1], demand)
            demand.connected = True

        # Dibujar conexiones
        for robot in robots:
            for neighbor in robot.connected_to:
                progress = robot.connection_progress.get(neighbor, 0)
                if progress > 0:
                    start_pos = (robot.x, robot.y)
                    end_pos = (neighbor.x, neighbor.y)
                    intermediate_x = robot.x + (neighbor.x - robot.x) * min(progress, 1.0)
                    intermediate_y = robot.y + (neighbor.y - robot.y) * min(progress, 1.0)
                    line_color = GREEN if isinstance(neighbor, Robot) else YELLOW
                    pygame.draw.line(screen, line_color, start_pos, (intermediate_x, intermediate_y), 3)

        # Dibujar todos los elementos
        source.draw(screen)
        demand.draw(screen)
        for robot in robots:
            robot.draw(screen)

        pygame.display.flip()
        clock.tick(FPS)

    pygame.quit()

if __name__ == "__main__":
    main()