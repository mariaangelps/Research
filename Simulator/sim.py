import pygame
import random
import math
from class_robot import Robot  # Usa class_robot que tú tengas

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
    current_robot_index = 0  # Solo uno se mueve a la vez

    while running:
        screen.fill((255, 255, 255))  

        # Actualizar solo el robot actual
        if current_robot_index < len(robots_list):
            robot = robots_list[current_robot_index]
            robot.update(screen, arena_width, arena_height, destinations)

            if robot.ready_for_next:
                print(f"Robot {robot.robot_id} reached its destination.")
                current_robot_index += 1  # Pasar al siguiente robot

        # Dibujar todos los robots (solo uno se mueve, los otros se quedan quietos)
        for robot in robots_list:
            robot.draw(screen)

        pygame.display.flip()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # Si todos terminaron
        if current_robot_index >= len(robots_list):
            print("All robots completed their destinations. Exiting simulation.")
            pygame.time.wait(2000)
            running = False

    pygame.quit()

if __name__ == "__main__":
    main()
