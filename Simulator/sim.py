import pygame
import random
from class_robot import Robot

def main():
    print("Sim Begin")

    pygame.init()
    arena_width, arena_height = 900, 400
    screen = pygame.display.set_mode((arena_width, arena_height))
    pygame.display.set_caption("Robot Arena")

    robot_radius = 20
    n_robots = 5
    robots_list = []

    for robot_id in range(n_robots):
        x = random.randint(5 * robot_radius, arena_width - 5 * robot_radius)
        y = random.randint(5 * robot_radius, arena_height - 5 * robot_radius)
        robot = Robot(robot_id, x, y, robot_radius)
        robots_list.append(robot)

    destinations = [
        (100, 100),
        (200, 200),
        (300, 300),
        (500, 100),
        (600, 300),
    ]

    # Optional: assign once at the beginning
    for i, robot in enumerate(robots_list):
        if i < len(destinations):
            robot.set_destination(*destinations[i])
            print(f"Robot {robot.robot_id} → assigned destination: {destinations[i]}")

    print("Enter Main Loop")
    running = True
    clock = pygame.time.Clock()

    while running:
        screen.fill((255, 255, 255))  # White background

        for robot in robots_list:
            my_position = (robot.x, robot.y)
            goal_position = (robot.dest_x, robot.dest_y)
            robot.update(screen, arena_width, arena_height, my_position, goal_position)

        pygame.display.flip()
        clock.tick(60)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

    pygame.quit()

if __name__ == "__main__":
    main()
