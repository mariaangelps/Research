import math

class Logic:
    def __init__(self, angle):
        self.angle = angle
        self.drive_speed = 4
        self.color = [0, 150, 0]  #green while moving

    def get_drive_speed(self):
        return self.drive_speed

    def get_angle(self):
        return self.angle

    def get_color(self):
        return self.color

    def update(self, dx, dy):
        # angle towards the target
        self.angle = math.atan2(dy, dx)
        

    def reset_speed(self):
        self.drive_speed = 3

    def stop(self):
        self.drive_speed = 0
