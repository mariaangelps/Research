import pygame
import random
import math
from class_logic import Logic

class Robot:
    drive_speed = 8  # Default speed

    def __init__(self, robot_id, x, y, radius, hand_shadow_radius, robots_list, background_image_path=None):
        self.robot_id = robot_id
        self.x = x
        self.y = y
        self.radius = radius
        self.hand_shadow_radius = hand_shadow_radius
        self.color = (0, 0, 255)
        self.background_image = pygame.image.load(background_image_path) if background_image_path else None
        self.angle = random.uniform(0, 2 * math.pi)
        self.under_shadow = False
        self.robots_list = robots_list
        self.am_contacting = False
        self.who_contacting = []
        self.odometry = 0.0
        self.logic = Logic(self.angle, "moving")

        # Destination attributes
        self.dest_x = None
        self.dest_y = None
        self.max_time = None
        self.destination_timer = 0
        self.has_destination = False

    def draw(self, screen):
        pygame.draw.circle(screen, self.color, (self.x, self.y), self.radius)

    def detect_hand_shadow(self, hand_coordinates):
        self.under_shadow = False
        if hand_coordinates:
            for hand_x, hand_y in hand_coordinates:
                if abs(self.x - hand_x) <= self.hand_shadow_radius and abs(self.y - hand_y) <= self.hand_shadow_radius:
                    self.under_shadow = True
                    break
    """
    def detect_collision(self):
        self.who_contacting = []
        for robot in self.robots_list:
            if robot != self:
                distance = math.sqrt((self.x - robot.x)**2 + (self.y - robot.y)**2)
                if distance < self.radius + robot.radius:
                    self.who_contacting.append(robot)
        self.am_contacting = len(self.who_contacting) > 0
    """
    def set_destination(self, x, y, time_in_seconds, fps=30):
        self.dest_x = x
        self.dest_y = y
        self.has_destination = True
        self.max_time = int(time_in_seconds * fps)
        self.destination_timer = 0

    def get_destination(self):
        if self.has_destination:
            return (self.dest_x, self.dest_y)
        return None

    def update(self, screen, hand_coordinates, arena_width, arena_height):
        self.detect_hand_shadow(hand_coordinates)
        #self.detect_collision()

        if self.has_destination:
            dx = self.dest_x - self.x
            dy = self.dest_y - self.y
            distance = math.sqrt(dx**2 + dy**2)

            if distance < self.radius:
                self.has_destination = False
                self.drive_speed = 0
                self.logic.state = "arrived" 
                print(f"Robot {self.robot_id} → destination reached ")
            else:
                self.angle = math.atan2(dy, dx)
                self.drive_speed = self.logic.drive_speed
        else:
            self.drive_speed = 0  # Stop if no destination

        # Background color logic
        pixel_color = (0, 0, 0)
        if self.background_image:
            try:
                pixel_color = self.background_image.get_at((self.x, self.y))
            except IndexError:
                pass

        # State machine logic
        self.logic.update(self.am_contacting, self.who_contacting, pixel_color, self.under_shadow)
        self.drive_speed = self.logic.get_drive_speed()
        self.angle = self.logic.get_angle()
        self.color = self.logic.get_color()

        # Movement
        self.x += int(self.drive_speed * math.cos(self.angle))
        self.y += int(self.drive_speed * math.sin(self.angle))

        # Bounds
        self.x = max(self.radius, min(arena_width - self.radius, self.x))
        self.y = max(self.radius, min(arena_height - self.radius, self.y))

        self.draw(screen)
