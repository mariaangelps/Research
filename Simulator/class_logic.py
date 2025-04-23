import random
import math
class Logic:
    def __init__(self, angle, start_state):
        self.state = start_state
        self.drive_speed = 5
        self.angle = angle
        self.color = [0, 0, 0]
        self.timer = 0
        self.config_reverse_time = 15
        self.max_bumping_time = 100  # New maximum time for "bumping"

    def get_drive_speed(self):
        return self.drive_speed
    
    def get_angle(self):
        return self.angle
    
    def get_color(self):
        return self.color
    
    def update(self, am_contacting, who_contacting, pixel_color, under_shadow):
        # Basic stuff
        self.timer += 1

        # Finite state machine logic
        if self.state == "moving":
            self.color = [0, 150, 0]
            self.drive_speed = 8

            if under_shadow:
                self.state = "under_shadow"
                self.timer = 0  # Reset timer when entering under_shadow state
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

        elif self.state == "under_shadow":
            self.color = [150, 0, 0]
            self.drive_speed = 0
            if not under_shadow:
                self.state = "moving"
                self.angle = random.uniform(0, 2 * math.pi)
                self.timer = 0  # Reset timer when leaving under_shadow state

        elif self.state == "arrived":
            self.color = [100, 100, 100]
            self.drive_speed = 0

        elif self.state == "bumping":
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
                # After reverse time, resume moving
                self.state = "moving"
                self.angle = random.uniform(0, 2 * math.pi)
                self.angle += math.radians(random.randint(-45, 45))
                self.timer = 0  # Reset timer when transitioning back to "moving"
