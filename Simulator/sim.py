import pygame
import random
import math
from class_robot import Robot
#OPTION 1 : SEQUENTIAL 0,1,2,3,4, ends when i terminate the program
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

    def assign_new_random_destination(robot):
        x = random.randint(5 * robot_radius, arena_width - 5 * robot_radius)
        y = random.randint(5 * robot_radius, arena_height - 5 * robot_radius)
        robot.set_destination(x, y)
        robot.ready_for_next = False
        print(f"Robot {robot.robot_id} → new destination: ({x}, {y})")

    # Inicialmente todos los robots tienen un destino asignado, pero solo el primero se moverá
    for i, robot in enumerate(robots_list):
        assign_new_random_destination(robot)
    current_robot_index = 0

    print("Enter Main Loop")
    running = True

    while running:
        screen.fill((255, 255, 255))

        # Solo mueve el robot activo
        robot = robots_list[current_robot_index]
        robot.update(screen, arena_width, arena_height)

        # Si llegó a destino, asigna uno nuevo y pasa al siguiente robot
        if robot.ready_for_next:
            print(f"Robot {robot.robot_id} reached its destination.")
            assign_new_random_destination(robot)
            current_robot_index = (current_robot_index + 1) % len(robots_list)

        # Dibuja todos
        for r in robots_list:
            r.draw(screen)

        pygame.display.flip()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

    pygame.quit()

if __name__ == "__main__":
    main()
