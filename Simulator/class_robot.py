import pygame
import random
import math
from class_logic import Logic

class Robot:
    drive_speed = 8  # Increased speed for x and y directions

    def __init__(self, robot_id, x, y, radius, hand_shadow_radius, robots_list, background_image_path=None):
        self.robot_id = robot_id  # Assign a unique ID to each robot
        self.x = x
        self.y = y
        self.radius = radius
        self.hand_shadow_radius = hand_shadow_radius
        self.color = (0, 0, 255)  # Default color is blue
        self.background_image = pygame.image.load(background_image_path) if background_image_path else None
        self.angle = random.uniform(0, 2 * math.pi)  # Random angle in radians
        self.under_shadow = False
        self.robots_list = robots_list
        self.am_contacting = False
        self.who_contacting = []
        self.odometry = 0.0  # Distance traveled by the robot
        self.logic = Logic(self.angle, "moving")  # Start the state machine in the "moving" state


    def draw(self, screen):
        pygame.draw.circle(screen, self.color, (self.x, self.y), self.radius)

    # El programa sabe que el robot está undershadow por esta funcion 
    def detect_hand_shadow(self, hand_coordinates):
        #si hay al menos una mano detectada, continuamos
        if hand_coordinates:
            # Check if the center of the robot is under the shadow by comparing its position to hand coordinates
            x, y = self.x, self.y
            self.under_shadow = False
            # Si la mano está dentro de ese cuadrado alrededor del robot, consideramos que el 
            # robot está bajo una sombra de mano.
            for hand_x, hand_y in hand_coordinates:
                if abs(x - hand_x) <= self.hand_shadow_radius and abs(y - hand_y) <= self.hand_shadow_radius:
                    self.under_shadow = True
        return 

    def detect_collision(self):
        # Check for collisions with other robots in the robots_list
        self.who_contacting = []
        for robot in self.robots_list:
            if robot != self:
                distance = math.sqrt((self.x - robot.x)**2 + (self.y - robot.y)**2)
                if distance < self.radius + robot.radius:
                    self.who_contacting.append(robot)
        if len(self.who_contacting) > 0:
            self.am_contacting = True
        else:
            self.am_contacting = False
        return

    def update(self, screen, hand_coordinates, arena_width, arena_height):
        # Inputs
        self.detect_hand_shadow(hand_coordinates)
        self.detect_collision()
        if self.background_image:
            pixel_color = self.background_image.get_at((self.x, self.y))

        # Logic
        self.logic.update(self.am_contacting, self.who_contacting, pixel_color, self.under_shadow)
        
        self.drive_speed = self.logic.get_drive_speed()
        self.angle = self.logic.get_angle()
        self.color = self.logic.get_color()

        # Movement and outputs
        self.x += int(self.drive_speed * math.cos(self.angle))
        self.y += int(self.drive_speed * math.sin(self.angle))

        # Keep the robot inside the arena boundaries
        self.x = max(self.radius, min(arena_width - self.radius, self.x))
        self.y = max(self.radius, min(arena_height - self.radius, self.y))

            
        self.draw(screen)
