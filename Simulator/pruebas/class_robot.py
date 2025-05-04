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
        self.color = (128, 128, 128)  # Gray initially
        self.destination = (x, y)
        self.speed = 2
        self.ready_for_next = False
        self.connected_to = []
        self.connected = False

    def set_destination(self, x, y):
        self.destination = (x, y)
        self.ready_for_next = False

    def update(self):
        dx = self.destination[0] - self.x
        dy = self.destination[1] - self.y
        dist = math.hypot(dx, dy)
        if dist < self.speed:
            self.x, self.y = self.destination
            self.ready_for_next = True
        else:
            self.x += self.speed * dx / dist
            self.y += self.speed * dy / dist

    def draw(self, screen):
        if self.connected:
            self.color = (0, 200, 0)  # Green if connected
        else:
            self.color = (128, 128, 128)  # Gray

        pygame.draw.circle(screen, self.color, (int(self.x), int(self.y)), self.radius)

        # Draw ID number on the robot
        font = pygame.font.SysFont(None, 16)
        text = font.render(str(self.robot_id), True, (0, 0, 0))
        text_rect = text.get_rect(center=(int(self.x), int(self.y)))
        screen.blit(text, text_rect)
