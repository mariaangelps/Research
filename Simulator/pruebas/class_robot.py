import pygame
import math
import random
from Simulator.pruebas.class_logic import Logic
class Robot:
    def __init__(self, robot_id, x, y, radius):
        self.robot_id = robot_id
        self.x = x
        self.y = y
        self.radius = radius
        self.color = (0, 0, 255)
        self.destination = (x, y)
        self.speed = 4
        self.ready_for_next = False

    def set_destination(self, x, y):
        self.destination = (x, y)
        self.ready_for_next = False

    def update(self, screen, arena_width, arena_height):
        dest_x, dest_y = self.destination
        dx = dest_x - self.x
        dy = dest_y - self.y
        distance = math.hypot(dx, dy)

        if distance < 5:
            self.ready_for_next = True
        else:
            angle = math.atan2(dy, dx)
            self.x += self.speed * math.cos(angle)
            self.y += self.speed * math.sin(angle)

    def draw(self, screen):
        # Dibuja el robot
        pygame.draw.circle(screen, self.color, (int(self.x), int(self.y)), self.radius)

        # Dibuja el destino como un punto (pequeño círculo rojo)
        dest_x, dest_y = self.destination
        pygame.draw.circle(screen, (255, 0, 0), (int(dest_x), int(dest_y)), 5)  # Rojo y pequeño
