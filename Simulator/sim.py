import pygame
from class_robot import Robot

def main():
    pygame.init()
    screen_width, screen_height = 800, 600
    screen = pygame.display.set_mode((screen_width, screen_height))
    pygame.display.set_caption("Robot Simulator")
    clock = pygame.time.Clock()

    # Create robots
    robot_radius = 20
    robots_list = [
        Robot(robot_id=i, x=100 + i * 100, y=100, radius=robot_radius)
        for i in range(5)
    ]

    running = True
    while running:
        screen.fill((255, 255, 255))  # White background

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        for robot in robots_list:
            robot.update(screen, screen_width, screen_height)

        pygame.display.flip()
        clock.tick(30)

    pygame.quit()

if __name__ == "__main__":
    print("Sim Begin")
    main()
