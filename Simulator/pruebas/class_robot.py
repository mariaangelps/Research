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
        dx = random.randint(-60, 60)
        dy = random.randint(-30, 30)
        self.destination = (self.x + dx, self.y + dy)

    def update(self):
        if not self.moving:
            return
        if self.destination == (self.x, self.y):
            self.set_new_destination()
        dx = self.destination[0] - self.x
        dy = self.destination[1] - self.y
        dist = math.hypot(dx, dy)
        if dist < self.speed:
            self.x, self.y = self.destination
            self.set_new_destination()
        else:
            self.x += self.speed * dx / dist
            self.y += self.speed * dy / dist

    def draw(self, screen):
        if self.connected_to_source and self.connected_to_demand:
            color = (0, 200, 0)  # Green
        else:
            color = (100, 100, 255)  # Blue
        pygame.draw.circle(screen, color, (int(self.x), int(self.y)), self.radius)
        font = pygame.font.SysFont(None, 20)
        text = font.render(str(self.robot_id), True, (0, 0, 0))
        text_rect = text.get_rect(center=(int(self.x), int(self.y)))
        screen.blit(text, text_rect)