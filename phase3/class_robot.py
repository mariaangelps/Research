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
        self.destination = (x, y)
        self.speed = 2
        self.moving = True
        self.connected_to = []
        self.connected_to_source = False
        self.connected_to_demand = False

    def set_new_destination(self):
        dx = random.randint(-20, 20)
        dy = random.randint(-10, 10)
        self.destination = (self.x + dx, self.y + dy)

    def set_destination(self, x, y):
        self.destination = (x, y)
        self.moving = True

    def update(self):
        if not self.moving:
            return

        dx = self.destination[0] - self.x
        dy = self.destination[1] - self.y
        dist = math.hypot(dx, dy)

        if dist < 2:  # umbral para decir que llegó
            self.x, self.y = self.destination
            print(f"Robot {self.robot_id} → destination reached at ({int(self.x)}, {int(self.y)})")
            
            # Elegir un nuevo destino (simulación continua)
            new_x = random.randint(50, 850)
            new_y = random.randint(50, 350)
            self.destination = (new_x, new_y)
            print(f"Robot {self.robot_id} → assigned new destination: ({new_x}, {new_y})")
        else:
            self.x += self.speed * dx / dist
            self.y += self.speed * dy / dist


    def draw(self, screen, color=(0, 0, 255)):
        pygame.draw.circle(screen, color, (int(self.x), int(self.y)), self.radius)
