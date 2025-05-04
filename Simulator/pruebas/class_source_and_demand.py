import pygame
import math
import random


class Source:
    def __init__(self, source_id, x, y, radius):
        self.source_id = source_id
        self.x = x
        self.y = y
        self.radius = radius
        self.color = (0, 0, 0)  # Black
        self.connected_to = []
        self.connected = True  # Source always connected

    def draw(self, screen):
        pygame.draw.circle(screen, self.color, (int(self.x), int(self.y)), self.radius)
        font = pygame.font.SysFont(None, 22)
        text = font.render("S", True, (255, 255, 255))
        text_rect = text.get_rect(center=(int(self.x), int(self.y)))
        screen.blit(text, text_rect)

class Demand:
    def __init__(self, demand_id, x, y, radius):
        self.demand_id = demand_id
        self.x = x
        self.y = y
        self.radius = radius
        self.color = (0, 0, 0)  # Black
        self.connected_to = []
        self.connected = False

    def draw(self, screen):
        pygame.draw.circle(screen, self.color, (int(self.x), int(self.y)), self.radius)
        font = pygame.font.SysFont(None, 22)
        text = font.render("D", True, (255, 255, 255))
        text_rect = text.get_rect(center=(int(self.x), int(self.y)))
        screen.blit(text, text_rect)



