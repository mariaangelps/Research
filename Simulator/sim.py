import pygame
import random
import math
from class_robot import Robot

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
    waiting_for_robot = False  # Flag to check if we're waiting for robot to finish

    while running:
        screen.fill((255, 255, 255))

        # Update the current robot
        robots_list[current_robot_index].update(screen, arena_width, arena_height, destinations)

        # Draw all robots on screen
        for robot in robots_list:
            robot.draw(screen)

        # If robot is ready, move to the next robot
        if robots_list[current_robot_index].ready_for_next and not waiting_for_robot:
            current_robot_index = (current_robot_index + 1) % len(robots_list)
            robots_list[current_robot_index].assign_new_destination(destinations)
            waiting_for_robot = True

        # Check if the current robot is ready and if so, start waiting for 1.5 sec
        if robots_list[current_robot_index].waiting and pygame.time.get_ticks() - robots_list[current_robot_index].wait_start_time >= 1500:
            robots_list[current_robot_index].ready_for_next = True
            waiting_for_robot = False  # Robot is ready for the next destination

        pygame.display.flip()

        # Check for quit event
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

    pygame.quit()

if __name__ == "__main__":
    main()
