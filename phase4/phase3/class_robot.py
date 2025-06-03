import pygame

class Robot:
    def __init__(self, robot_id, x, y, radius):
        self.robot_id = robot_id
        self.x = x
        self.y = y
        self.radius = radius
        self.hop_count = None
        self.parent = None

    def draw(self, screen, color=(0, 200, 0)):
        pygame.draw.circle(screen, color, (int(self.x), int(self.y)), self.radius)
        font = pygame.font.Font(None, 20)
        label = font.render(str(self.robot_id), True, (0, 0, 0))
        screen.blit(label, (self.x - 10, self.y - 25))
