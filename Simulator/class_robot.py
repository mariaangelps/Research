import pygame
import math
from class_logic import Logic
import random

class Robot:
    def __init__(self, robot_id, x, y, radius):
        self.robot_id = robot_id
        self.x = x
        self.y = y
        self.radius = radius
        self.color = (0, 150, 0)
        self.angle = random.uniform(0, 2 * math.pi)
        self.logic = Logic(self.angle)
        self.dest_x = None
        self.dest_y = None
        self.has_destination = False

    def draw(self, screen):
        pygame.draw.circle(screen, self.color, (int(self.x), int(self.y)), self.radius)

    def set_destination(self, x, y):
        self.dest_x = x
        self.dest_y = y
        self.has_destination = True

    def reached_destination(self):
        if not self.has_destination:
            return "No destination set"
        dx = self.dest_x - self.x
        dy = self.dest_y - self.y
        distance = math.hypot(dx, dy)
        if distance < self.radius:
            return f"Reached destination: ({self.dest_x}, {self.dest_y})"
        else:
            return f"Moving towards: ({self.dest_x}, {self.dest_y})"

    def update(self, screen, arena_width, arena_height, my_position, goal_position):
        my_x, my_y = my_position
        goal_x, goal_y = goal_position
        
        # Calcular la diferencia entre las posiciones
        dx = goal_x - my_x
        dy = goal_y - my_y
        distance = math.hypot(dx, dy)

        if distance < self.radius:
            self.has_destination = False
            self.logic.drive_speed = 0
            print(f"Robot {self.robot_id} → destination reached")
        else:
            # Actualizar el ángulo con la lógica
            self.logic.update(dx, dy)
            self.angle = self.logic.get_angle()  # Obtener el ángulo actualizado
            
            # Mover el robot hacia el destino
            speed = self.logic.get_drive_speed()
            my_x += int(speed * math.cos(self.angle))
            my_y += int(speed * math.sin(self.angle))

        # Asegurarse de que el robot no se salga del área
        my_x = max(self.radius, min(arena_width - self.radius, my_x))
        my_y = max(self.radius, min(arena_height - self.radius, my_y))

        # Actualizar las coordenadas del robot
        self.x = my_x
        self.y = my_y
        
        self.draw(screen)

        

