import math

class Logic:
    def __init__(self, angle):
        self.angle = angle
        self.drive_speed = 5
        self.color = [0, 150, 0]  # Always green while moving

    def get_drive_speed(self):
        return self.drive_speed

    def get_angle(self):
        return self.angle

    def get_color(self):
        return self.color

    def update(self):
        # In this simplified version, no state changes
        pass
