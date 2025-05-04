import pygame
import random
import math
from class_robot import Robot
from class_source_and_demand import Source, Demand

#OPTION 1 : SEQUENTIAL 0,1,2,3,4, ends when I exit the program
def distance(a, b):
    return math.hypot(a.x - b.x, a.y - b.y)

def propagate_connection(robot, visited):
    if robot in visited:
        return
    visited.add(robot)
    robot.connected = True
    for neighbor in robot.connected_to:
        propagate_connection(neighbor, visited)

def main():
    pygame.init()
    arena_width, arena_height = 900, 400
    screen = pygame.display.set_mode((arena_width, arena_height))
    pygame.display.set_caption("Robot Arena")

    robot_radius = 10
    connection_distance = 30
    n_robots = 5
    robots_list = []

    source_robot = Source("S1", 100, 100, 15)
    demand_robot = Demand("D1", 800, 300, 15)

    for robot_id in range(n_robots):
        x = random.randint(50, arena_width - 50)
        y = random.randint(50, arena_height - 50)
        robot = Robot(robot_id, x, y, robot_radius)
        robots_list.append(robot)

    def assign_new_random_destination(robot):
        x = random.randint(50, arena_width - 50)
        y = random.randint(50, arena_height - 50)
        robot.set_destination(x, y)
        robot.ready_for_next = False

    for robot in robots_list:
        assign_new_random_destination(robot)

    current_robot_index = 0
    running = True

    while running:
        screen.fill((255, 255, 255))

        source_robot.draw(screen)
        demand_robot.draw(screen)

        robot = robots_list[current_robot_index]
        robot.update()

        # Check for nearby connections (to source or other robots)
        if robot.ready_for_next:
            if distance(robot, source_robot) < connection_distance:
                robot.connected_to.append(source_robot)
                robot.connected = True
            for other in robots_list:
                if other != robot and distance(robot, other) < connection_distance:
                    robot.connected_to.append(other)
                    other.connected_to.append(robot)
            if distance(robot, demand_robot) < connection_distance:
                robot.connected_to.append(demand_robot)

            # Propagate connection from source
            visited = set()
            for r in robots_list:
                r.connected = False  # reset
            propagate_connection(source_robot, visited)

            assign_new_random_destination(robot)
            current_robot_index = (current_robot_index + 1) % len(robots_list)

        # Draw connections as lines
        for robot in robots_list:
            for neighbor in robot.connected_to:
                pygame.draw.line(screen, (0, 200, 0), (robot.x, robot.y), (neighbor.x, neighbor.y), 2)

        # Draw robots
        for r in robots_list:
            r.draw(screen)

        pygame.display.flip()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

    pygame.quit()

if __name__ == "__main__":
    main()