import pygame
import math
from class_logic import Logic
import random


class Robot:
    def __init__(self, robot_id, x, y, radius):
        self.robot_id = robot_id
        self.x = x
        self.y = y
        self.radius = radius
        self.color = (0, 150, 0)  # Default color
        self.angle = random.uniform(0, 2 * math.pi)
        self.logic = Logic(self.angle)

        # Destination attributes
        self.dest_x = None
        self.dest_y = None
        self.has_destination = False

    def draw(self, screen):
        pygame.draw.circle(screen, self.color, (self.x, self.y), self.radius)

    def set_destination(self, x, y):
        self.dest_x = x
        self.dest_y = y
        self.has_destination = True

    def update(self, screen, arena_width, arena_height):
        if self.has_destination:
            pygame.draw.circle(screen, (255, 0, 0), (self.dest_x, self.dest_y), 5)
            dx = self.dest_x - self.x
            dy = self.dest_y - self.y
            distance = math.sqrt(dx**2 + dy**2)

            if distance < self.radius:
                self.has_destination = False
                print(f"Robot {self.robot_id} → destination reached")
                self.logic.drive_speed = 0
            else:
                self.angle = math.atan2(dy, dx)
                self.logic.angle = self.angle  # Update logic angle if needed

        # Update color and movement
        self.color = self.logic.get_color()
        speed = self.logic.get_drive_speed()
        self.x += int(speed * math.cos(self.angle))
        self.y += int(speed * math.sin(self.angle))

        # Keep inside bounds
        self.x = max(self.radius, min(arena_width - self.radius, self.x))
        self.y = max(self.radius, min(arena_height - self.radius, self.y))

        self.draw(screen)

    def reached_destination(self):
        if not self.has_destination:
            return "No destination set"  # No destination assigned
        dx = self.dest_x - self.x
        dy = self.dest_y - self.y
        distance = math.sqrt(dx**2 + dy**2)
        
        if distance < self.radius:
            return f"Reached destination: ({self.dest_x}, {self.dest_y})"
        else:
            return f"Moving towards: ({self.dest_x}, {self.dest_y})"

