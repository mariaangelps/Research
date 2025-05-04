import pygame
import math
import random
import time
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
        self.previous_dest = None  # Keep track of the last destination to avoid repetition
        self.time_at_destination = None  # Timestamp for when the robot arrives at a destination
        self.wait_time = 3  # 3 seconds wait time at destination

    def draw(self, screen):
        pygame.draw.circle(screen, self.color, (int(self.x), int(self.y)), self.radius)

    def set_destination(self, x, y):
        self.dest_x = x
        self.dest_y = y
        self.has_destination = True

    def update(self, screen, arena_width, arena_height, destinations):
        if self.has_destination:
            pygame.draw.circle(screen, (255, 0, 0), (self.dest_x, self.dest_y), 5)

            dx = self.dest_x - self.x
            dy = self.dest_y - self.y
            distance = math.sqrt(dx**2 + dy**2)

            if distance < self.radius:
                print(f"Robot {self.robot_id} → destination reached at {self.x} and {self.y}")
                self.has_destination = False  # Stop the robot
                self.logic.drive_speed = 0  # Stop movement

                # Record the time when the robot reaches the destination
                self.time_at_destination = pygame.time.get_ticks()

            else:
                # Move towards the destination
                self.angle = math.atan2(dy, dx)
                self.logic.angle = self.angle

                speed = self.logic.get_drive_speed()
                self.x += int(speed * math.cos(self.angle))
                self.y += int(speed * math.sin(self.angle))

        # If the robot has reached the destination, wait for the specified time
        if not self.has_destination and self.time_at_destination is not None:
            if pygame.time.get_ticks() - self.time_at_destination >= self.wait_time * 1000:
                # After waiting, assign a new destination
                self.assign_new_destination(destinations)

        # Keep the robot within bounds of the arena
        self.x = max(self.radius, min(arena_width - self.radius, self.x))
        self.y = max(self.radius, min(arena_height - self.radius, self.y))

        self.draw(screen)

    def assign_new_destination(self, destinations):
        if not self.has_destination:
            if destinations:
                # Filter out destinations that have been already assigned or recently visited
                available_destinations = [dest for dest in destinations if dest != self.previous_dest]

                if available_destinations:
                    new_dest = random.choice(available_destinations)
                    print(f"Robot {self.robot_id} → assigned new destination: {new_dest}")
                    self.set_destination(new_dest[0], new_dest[1])
                    self.previous_dest = new_dest
                    self.logic.drive_speed = 2
                else:
                    print(f"Robot {self.robot_id} → resetting available destinations.")
                    self.previous_dest = None
                    self.assign_new_destination(destinations)
