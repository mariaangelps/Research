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
        robot.set_new_destination()
        robots_list.append(robot)

    running = True
    while running:
        screen.fill((255, 255, 255))
        source.draw(screen)
        demand.draw(screen)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # Update robot positions
        for robot in robots_list:
            if robot.moving:
                robot.update()

        # Connect robot 0 to source if close enough
        if distance(robots_list[0], source) < connection_distance:
            if source not in robots_list[0].connected_to:
                robots_list[0].connected_to.append(source)
                robots_list[0].moving = False
                print("Robot 0 connected to Source")

        # Connect robot 4 to demand if close enough
        if distance(robots_list[4], demand) < connection_distance:
            if demand not in robots_list[4].connected_to:
                robots_list[4].connected_to.append(demand)
                robots_list[4].moving = False
                print("Robot 4 connected to Demand")

        # Clear old connectivity status
        for robot in robots_list:
            robot.connected_to_source = False
            robot.connected_to_demand = False

        # Propagate source/demand connectivity
        propagate_connection_from_source(robots_list[0], set())
        propagate_connection_from_demand(robots_list[4], set())

        # Try to connect each robot to its neighbors
        for i in range(1, len(robots_list)):
            curr_robot = robots_list[i]
            prev_robot = robots_list[i - 1]

            if distance(curr_robot, prev_robot) < connection_distance:
                if prev_robot not in curr_robot.connected_to:
                    curr_robot.connected_to.append(prev_robot)
                    prev_robot.connected_to.append(curr_robot)
                    print(f"Robot {curr_robot.robot_id} connected to Robot {prev_robot.robot_id}")

                # If previous is already connected to source or demand, stop moving
                if prev_robot.connected_to_source or prev_robot.connected_to_demand:
                    curr_robot.moving = False

        # Draw connection lines only if robots are close and connected
        for robot in robots_list:
            for neighbor in robot.connected_to:
                if isinstance(neighbor, Robot) and distance(robot, neighbor) < connection_distance:
                    pygame.draw.line(screen, (0, 200, 0), (robot.x, robot.y), (neighbor.x, neighbor.y), 2)
                if isinstance(neighbor, Source) or isinstance(neighbor, Demand):
                    pygame.draw.line(screen, (0, 0, 0), (robot.x, robot.y), (neighbor.x, neighbor.y), 2)

        # Draw robots: green if connected to source, else blue
        for robot in robots_list:
            color = (0, 255, 0) if robot.connected_to_source else (0, 0, 255)
            robot.draw(screen, color=color)

            # Draw the robot's ID number near the robot
            font = pygame.font.Font(None, 36)
            text = font.render(str(robot.robot_id), True, (0, 0, 0))
            screen.blit(text, (robot.x - robot_radius / 2, robot.y - robot_radius / 2))

        pygame.display.flip()

    pygame.quit()

if __name__ == "__main__":
    main()
