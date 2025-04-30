import pygame
import random
import math
from class_robot import Robot
#sequential, each running in order
def main():
    print("Sim Begin")

    pygame.init()
    arena_width, arena_height = 900, 400
    screen = pygame.display.set_mode((arena_width, arena_height))
    pygame.display.set_caption("Robot Arena")

    robot_radius = 10
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

    def assign_destinations(robots_list, destinations):
        for i, robot in enumerate(robots_list):
            if i < len(destinations):
                dest_x, dest_y = destinations[i]
                robot.set_destination(dest_x, dest_y)
                print(f"Robot {robot.robot_id} → assigned destination: ({dest_x}, {dest_y})")

    assign_destinations(robots_list, destinations)

    print("Enter Main Loop")
    running = True
    current_robot_index = 0  # Start with the first robot
    all_robots_completed = False  # Flag to check if all robots have completed their destinations

    while running:
        screen.fill((255, 255, 255))

        # Only update the current robot that is in the process of moving
        robots_list[current_robot_index].update(screen, arena_width, arena_height, destinations)

        # Draw all robots on screen
        for robot in robots_list:
            robot.draw(screen)

        # Check if the current robot has finished its destination
        if robots_list[current_robot_index].ready_for_next:
            # Assign a new destination to the current robot
            robots_list[current_robot_index].assign_new_destination(destinations)

            # Move to the next robot in the list (sequentially)
            current_robot_index = (current_robot_index + 1) % len(robots_list)

        # Check if all robots have completed their journey
        all_robots_completed = all(robot.ready_for_next for robot in robots_list)

        if all_robots_completed:
            print("All robots have completed their destinations. Exiting simulation.")
            running = False

        pygame.display.flip()

        # Check for quit event
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

    pygame.quit()

if __name__ == "__main__":
    main()
