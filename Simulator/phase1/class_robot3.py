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
        self.color = (255, 165, 0)  # orange
        self.angle = random.uniform(0, 2 * math.pi)
        self.logic = Logic(self.angle)

        self.dest_x = None
        self.dest_y = None
        self.has_destination = False
        self.waiting = False
        self.wait_start_time = 0
        self.ready_for_next = False
        self.previous_dest = None

    def draw(self, screen):
    # Dibuja el robot como un círculo verde
        pygame.draw.circle(screen, self.color, (int(self.x), int(self.y)), self.radius)
        
        # Dibuja el destino como un pequeño punto rojo si tiene uno
        if self.has_destination and self.dest_x is not None and self.dest_y is not None:
            pygame.draw.circle(screen, (255, 0, 0), (int(self.dest_x), int(self.dest_y)), 5)

    def set_destination(self, x, y):
        self.dest_x = x
        self.dest_y = y
        self.has_destination = True
        self.waiting = False
        self.ready_for_next = False

    def update(self, screen, arena_width, arena_height, destinations):
        if self.has_destination:
            dx = self.dest_x - self.x
            dy = self.dest_y - self.y
            distance = math.sqrt(dx**2 + dy**2)

            if distance < self.radius:
                if not self.waiting:
                    print(f"Robot {self.robot_id} → destination reached at {int(self.x)} and {int(self.y)}")
                    self.waiting = True
                    self.wait_start_time = pygame.time.get_ticks()
                    self.logic.drive_speed = 0
                else:
                    if pygame.time.get_ticks() - self.wait_start_time >= 1200:
                        self.has_destination = False
                        self.ready_for_next = True
            else:
                if not self.waiting:
                    self.angle = math.atan2(dy, dx)
                    self.logic.angle = self.angle
                    speed = self.logic.get_drive_speed()
                    self.x += int(speed * math.cos(self.angle))
                    self.y += int(speed * math.sin(self.angle))

        self.x = max(self.radius, min(arena_width - self.radius, self.x))
        self.y = max(self.radius, min(arena_height - self.radius, self.y))
        self.draw(screen)

    def assign_new_destination(self, destinations):
        available = [d for d in destinations if d != self.previous_dest]
        if available:
            new_dest = random.choice(available)
            print(f"Robot {self.robot_id} → assigned new destination: {new_dest}")
            self.set_destination(*new_dest)
            self.previous_dest = new_dest
        else:
            self.previous_dest = None
            self.assign_new_destination(destinations)