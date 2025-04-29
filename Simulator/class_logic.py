import random

class Logic:
    def __init__(self, angle):
        self.angle = angle
        self.drive_speed = 2.5  # Puedes ajustar

    def get_color(self):
        return (0, 150, 0)  # Verde

    def get_drive_speed(self):
        return self.drive_speed

    def get_angle(self):
        return self.angle
