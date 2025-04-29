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
        self.color = (0, 150, 0)  # Default color
        self.angle = random.uniform(0, 2 * math.pi)
        self.logic = Logic(self.angle)

        # Destination attributes
        self.dest_x = None
        self.dest_y = None
        self.has_destination = False
        self.arrival_time = None  # Store the time when the destination is reached
        self.wait_time = 3000  # 3 seconds in milliseconds

    def set_destination(self, x, y):
        self.dest_x = x
        self.dest_y = y
        self.has_destination = True
        self.arrival_time = None  # Reset arrival time when a new destination is set

    def update(self, screen, arena_width, arena_height):
        if self.has_destination:
            dx = self.dest_x - self.x
            dy = self.dest_y - self.y
            distance = math.hypot(dx, dy)

            if distance < self.radius:
                if self.arrival_time is None:
                    self.arrival_time = pygame.time.get_ticks()  # Capture arrival time
                    print(f"Robot {self.robot_id} → destination reached at {self.x}, {self.y}")

                # Check if enough time has passed to set a new destination
                if pygame.time.get_ticks() - self.arrival_time >= self.wait_time:
                    self.has_destination = False
                    print(f"Robot {self.robot_id} → ready for new destination")
            else:
                self.angle = math.atan2(dy, dx)
                self.logic.angle = self.angle  # Update logic angle
                speed = self.logic.get_drive_speed()
                self.x += int(speed * math.cos(self.angle))
                self.y += int(speed * math.sin(self.angle))

        # Keep inside bounds
        self.x = max(self.radius, min(arena_width - self.radius, self.x))
        self.y = max(self.radius, min(arena_height - self.radius, self.y))

        # Draw the robot and its destination
        self.color = self.logic.get_color()
        self.draw(screen)

        if self.has_destination:
            pygame.draw.circle(screen, (255, 0, 0), (int(self.dest_x), int(self.dest_y)), 5)

    def draw(self, screen):
        pygame.draw.circle(screen, self.color, (self.x, self.y), self.radius)
