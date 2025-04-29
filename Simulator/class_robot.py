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
        self.previous_dest = None  # Keep track of the last destination to avoid repetition

    def draw(self, screen):
        pygame.draw.circle(screen, self.color, (int(self.x), int(self.y)), self.radius)

    def set_destination(self, x, y):
        self.dest_x = x
        self.dest_y = y
        self.has_destination = True

    def update(self, screen, arena_width, arena_height, destinations):
        if self.has_destination:
            pygame.draw.circle(screen, (255, 0, 0), (self.dest_x, self.dest_y), 5)

            #self.x is current position
            #self.dest_x is the destination location for x
            dx = self.dest_x - self.x  # Distance in x
            dy = self.dest_y - self.y  # Distance in y
            distance = math.sqrt(dx**2 + dy**2)  # Calculate the Euclidean distance

            # Check if the robot has reached the destination
            if distance < self.radius:
                print(f"Robot {self.robot_id} → destination reached at {self.x} and {self.y}")
                self.has_destination = False  # Mark destination as reached
                self.logic.drive_speed = 0  # Stop the robot
                self.assign_new_destination(destinations)  # Assign a new destination once the current one is reached
            else:
                # Move towards the destination by calculating the angle and updating position
                self.angle = math.atan2(dy, dx)  # Calculate the angle to the destination
                self.logic.angle = self.angle  # Update logic angle

                # Update the position
                speed = self.logic.get_drive_speed()  # Get the current speed from the logic
                self.x += int(speed * math.cos(self.angle))  # Move in the x direction
                self.y += int(speed * math.sin(self.angle))  # Move in the y direction

        # Keep the robot within bounds of the arena
        self.x = max(self.radius, min(arena_width - self.radius, self.x))
        self.y = max(self.radius, min(arena_height - self.radius, self.y))

        self.draw(screen)  # Draw the robot on the screen


    def assign_new_destination(self, destinations):
        if not self.has_destination:  # Only assign a new destination if no current one
            if destinations:
                # Filter out destinations that have been already assigned or recently visited
                available_destinations = [dest for dest in destinations if dest != self.previous_dest]
                
                if available_destinations:
                    new_dest = random.choice(available_destinations)
                    print(f"Robot {self.robot_id} → assigned new destination: {new_dest}")
                    self.set_destination(new_dest[0], new_dest[1])  # Assign a new random destination
                    self.previous_dest = new_dest  # Update the previous destination
                else:
                    print(f"Robot {self.robot_id} → resetting available destinations.")
                    self.previous_dest = None  # Reset previous destination if no options left
                    self.assign_new_destination(destinations)  # Retry with all available destinations
