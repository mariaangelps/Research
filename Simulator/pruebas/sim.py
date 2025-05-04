import pygame
import random
import math
from class_robot import Robot
from class_source_and_demand import Source, Demand

def distance(a, b):
    return math.hypot(a.x - b.x, a.y - b.y)

def propagate_connection_from_source(robot, visited):
    if robot in visited:
        return
    visited.add(robot)
    robot.connected_to_source = True
    for neighbor in robot.connected_to:
        if isinstance(neighbor, Robot):
            propagate_connection_from_source(neighbor, visited)

def propagate_connection_from_demand(robot, visited):
    if robot in visited:
        return
    visited.add(robot)
    robot.connected_to_demand = True
    for neighbor in robot.connected_to:
        if isinstance(neighbor, Robot):
            propagate_connection_from_demand(neighbor, visited)

def main():
    pygame.init()
    screen = pygame.display.set_mode((1000, 500))
    pygame.display.set_caption("Robot Chain Connection")

    robot_radius = 20
    connection_distance = 60
    n_robots = 5
    robots_list = []

    source = Source("Source", 100, 250, 25)
    demand = Demand("Demand", 900, 250, 25)

    positions = [(200, 250), (300, 250), (400, 250), (500, 250), (600, 250)]
    for i in range(n_robots):
        x, y = positions[i]
        robot = Robot(i, x, y, robot_radius)
        robots_list.append(robot)

    # Force robot 0 to be connected to source and stop moving
    robots_list[0].connected_to.append(source)
    robots_list[0].moving = False

    # Force robot 4 to be connected to demand and stop moving
    robots_list[4].connected_to.append(demand)
    robots_list[4].moving = False

    running = True
    while running:
        screen.fill((255, 255, 255))
        source.draw(screen)
        demand.draw(screen)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # Update positions
        for robot in robots_list:
            robot.update()

        # Clear previous connection flags
        for robot in robots_list:
            robot.connected_to_source = False
            robot.connected_to_demand = False

        # Propagate source and demand connectivity
        propagate_connection_from_source(robots_list[0], set())
        propagate_connection_from_demand(robots_list[4], set())

        # Handle connections
        for i, robot in enumerate(robots_list):
            if not robot.moving:
                continue
            for j, other in enumerate(robots_list):
                if i != j and distance(robot, other) < connection_distance:
                    if other not in robot.connected_to:
                        robot.connected_to.append(other)
                        other.connected_to.append(robot)
                        print(f"Robot {i} connected to Robot {j}")

        # Stop robots if they're connected to both source and demand
        for robot in robots_list:
            if robot.connected_to_source and robot.connected_to_demand:
                robot.moving = False

        # Draw connection lines
        for robot in robots_list:
            for neighbor in robot.connected_to:
                pygame.draw.line(screen, (0, 200, 0), (robot.x, robot.y), (neighbor.x, neighbor.y), 2)

        # Draw robots
        for robot in robots_list:
            robot.draw(screen)

        pygame.display.flip()

    pygame.quit()

if __name__ == "__main__":
    main()
