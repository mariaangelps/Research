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
    hand_shadow_radius = 75
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
        (300, 400),
        (500, 100),
        (600, 300),
    ]

    def assign_destinations(robots_list, destinations, time_in_seconds=5, fps=60):
        for i, robot in enumerate(robots_list):
            if i < len(destinations):
                dest_x, dest_y = destinations[i]
                robot.set_destination(dest_x, dest_y)

                print(f"Robot {robot.robot_id} → assigned destination: ({dest_x}, {dest_y})")

    assign_destinations(robots_list, destinations)

    print("Enter Main Loop")
    running = True
    while running:
        screen.fill((255, 255, 255))  # Plain white background

        for robot in robots_list:
            robot.update(screen, arena_width, arena_height)
            print(f"Robot {robot.robot_id}: {robot.reached_destination()}")

        pygame.display.flip()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

    pygame.quit()

if __name__ == "__main__":
    main()
