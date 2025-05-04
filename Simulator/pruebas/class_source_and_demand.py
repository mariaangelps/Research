import pygame
import math
import random

class Source:
    def __init__(self, label, x, y, radius):
        self.label = label
        self.x = x
        self.y = y
        self.radius = radius

    def draw(self, screen):
        pygame.draw.circle(screen, (0, 0, 0), (self.x, self.y), self.radius)
        font = pygame.font.Font(None, 36)
        text = font.render("S", True, (255, 255, 255))
        screen.blit(text, (self.x - self.radius / 2, self.y - self.radius / 2))


class Demand:
    def __init__(self, label, x, y, radius):
        self.label = label
        self.x = x
        self.y = y
        self.radius = radius
        self.connected = False

    def draw(self, screen):
        color = (0, 255, 0) if self.connected else (0, 0, 0)
        pygame.draw.circle(screen, color, (self.x, self.y), self.radius)
        font = pygame.font.Font(None, 36)
        text = font.render("D", True, (255, 255, 255))
        screen.blit(text, (self.x - self.radius / 2, self.y - self.radius / 2))