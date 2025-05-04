import pygame
import math
import random
from class_logic import Logic

class Source:
    def __init__(self, source_id, x, y, radius):
        self.source_id = source_id
        self.x = x
        self.y = y
        self.radius = radius
        self.color = (255, 165, 0)  # Orange
        self.connected_to = []  # Can act like a robot
        self.connected = True  # Source is always connected

    def draw(self, screen):
        pygame.draw.circle(screen, self.color, (int(self.x), int(self.y)), self.radius)

class Demand:
    def __init__(self, demand_id, x, y, radius):
        self.demand_id = demand_id
        self.x = x
        self.y = y
        self.radius = radius
        self.color = (0, 128, 255)  # Blue
        self.connected_to = []  # Can receive connections
        self.connected = False

    def draw(self, screen):
        pygame.draw.circle(screen, self.color, (int(self.x), int(self.y)), self.radius)