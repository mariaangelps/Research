import random
import math

class Logic:
    def __init__(self, angle, start_state):
        self.state = start_state
        self.drive_speed = 5
        self.angle = angle
        self.color = [0, 0, 0]
        self.timer = 0
        self.config_reverse_time = 5
        
    def get_drive_speed(self):
        return self.drive_speed
    
    def get_angle(self):
        return self.angle
    
    def get_color(self):
        return self.color
    
    def update(self, am_contacting, who_contacting, pixel_color, under_shadow):
        # Basic stuff
        self.timer = self.timer + 1

        # -----------------------------------------------------------------------
        # Finite state machine logic
        if self.state == "moving":
            # Turn green, and move until hitting either another robot or a projected-light color or shadow
            self.color = [0, 150, 0]
            self.drive_speed = 8

            # Stop moving if under shadow
            if under_shadow:
                self.state = "under_shadow"
            elif am_contacting:
                self.state = "bumping"
                self.timer = 0
            else:
                # Reflect horizontally if hitting green region
                if pixel_color == (0, 255, 0):
                    self.angle = math.pi - self.angle  # Reflect horizontally
                # Reflect vertically if hitting blue region
                if pixel_color == (0, 0, 255):
                    self.angle = -self.angle  # Reflect vertically

        # -----------------------------------------------------------------------
        elif self.state == "under_shadow":
            # Turn red and stop until no longer under shadow, then resume motion in a random direction
            self.color = [150, 0, 0]
            self.drive_speed = 0
            if not under_shadow:
                self.state = "moving"
                self.angle = random.uniform(0, 2 * math.pi)

        # -----------------------------------------------------------------------
        elif self.state == "bumping":
            # Turn blue, reverse motion for some duration, then choose a random new direction
            self.color = [0, 0, 150]

            if self.timer < self.config_reverse_time:
                self.drive_speed = -8
                # Reflect horizontally if hitting green region
                if pixel_color == (0, 150, 0):
                    self.angle = math.pi - self.angle  # Reflect horizontally
                # Reflect vertically if hitting blue region
                if pixel_color == (0, 0, 150):
                    self.angle = -self.angle  # Reflect vertically
            else:
                self.state = "moving"
                self.angle = random.uniform(0, 2 * math.pi)




                       
            

                    

    
        
