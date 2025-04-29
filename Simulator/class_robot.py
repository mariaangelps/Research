import pygame
import math
import random
from class_logic import Logic

class Robot:
    def __init__(self, robot_id, x, y, radius):
        self.robot_id = robot_id
        self.x = x
        self.y = y
        self.radius = radius
        self.color = (0, 150, 0)
        self.angle = random.uniform(0, 2 * math.pi)
        self.logic = Logic(self.angle)
        # Destino actual (inicialmente ninguno)
        self.dest_x = None
        self.dest_y = None
        self.has_destination = False

    def draw(self, screen):
        pygame.draw.circle(screen, self.color, (int(self.x), int(self.y)), self.radius)
        # Opcional: dibujar el destino como un pequeño círculo rojo.
        if self.has_destination:
            pygame.draw.circle(screen, (255, 0, 0), (self.dest_x, self.dest_y), 5)

    def set_destination(self, x, y):
        self.dest_x = x
        self.dest_y = y
        self.has_destination = True
        print(f"Robot {self.robot_id} → New destination: ({x}, {y})")

    def reached_destination(self):
        if not self.has_destination:
            return False
        dx = self.dest_x - self.x
        dy = self.dest_y - self.y
        # Usamos math.hypot para la distancia
        return math.hypot(dx, dy) < self.radius

    def assign_next_destination(self, arena_width, arena_height):
        margin = 50  # opcional, para evitar que vayan a los bordes extremos
        dest_x = random.randint(margin, arena_width - margin)
        dest_y = random.randint(margin, arena_height - margin)
        self.set_destination(dest_x, dest_y)
        print(f"Robot {self.robot_id} → New destination: ({dest_x}, {dest_y})")


    def update(self, screen, arena_width, arena_height, robots_list):
        if self.has_destination:
            dx = self.dest_x - self.x
            dy = self.dest_y - self.y
            distance = math.hypot(dx, dy)

            # Verificar si alcanzó el destino
            if distance < self.radius:
                self.has_destination = False
                print(f"Robot {self.robot_id} → Destination reached")
                self.logic.drive_speed = 0
            else:
                self.angle = math.atan2(dy, dx)
                self.logic.angle = self.angle
        # Si el robot no tiene destino, le asignamos uno nuevo globalmente
        if not self.has_destination:
            self.assign_next_destination(robots_list)

        # Actualizar color y realizar movimiento con precisión (x e y como float)
        self.color = self.logic.get_color()
        speed = self.logic.get_drive_speed()
        self.x += speed * math.cos(self.angle)
        self.y += speed * math.sin(self.angle)

        # Mantenerse dentro de los límites
        self.x = max(self.radius, min(arena_width - self.radius, self.x))
        self.y = max(self.radius, min(arena_height - self.radius, self.y))

        self.draw(screen)
