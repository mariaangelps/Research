import pygame
import random
import math
from class_robot import Robot

class Obstacle:
    def __init__(self, x, y, radius=20):
        self.x = x
        self.y = y
        self.radius = radius

    def draw(self, screen):
        pygame.draw.circle(screen, (150, 75, 0), (int(self.x), int(self.y)), self.radius)  # brown
